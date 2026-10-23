/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wang Kang
 * Standalone Bluetooth LE advertising and Apple Find My scanner for BirriaSDR.
 */
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <iomanip>
#include <iostream>
#include <limits>
#include <netdb.h>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <vector>
#include <memory>
#include "birria_native.hpp"
#include "birria_tui.hpp"
#include <csignal>

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double BLE_RATE = 1000000.0;
constexpr unsigned SAMPLES_PER_SYMBOL = 4;
constexpr double CHANNEL_RATE = BLE_RATE * SAMPLES_PER_SYMBOL;
constexpr uint32_t ADV_ACCESS_ADDRESS = UINT32_C(0x8e89bed6);
constexpr uint32_t ADV_CRC_INIT = UINT32_C(0x555555);
constexpr size_t POST_FILTER_TAPS = 25;
constexpr size_t MAX_ADV_PAYLOAD = 63;

std::atomic<bool> interrupted{false};

struct Options {
    std::string bind = "0.0.0.0";
    std::string port = "1234";
    std::string file;
    uint64_t center_frequency = UINT64_C(2426000000);
    uint32_t sample_rate = 40000000;
    uint32_t device_index = 0;
    uint32_t recovery_restarts = 3;
    uint32_t dwell_ms = 1000;
    std::vector<unsigned> channels;
    bool udp = false;
    bool manual_gain = false;
    bool sweep = false;
    bool frequency_set = false;
    bool dwell_set = false;
    bool compact = false;
    bool interactive = false;
    bool verbose = false;
    bool include_invalid = false;
    bool self_test = false;
};

void usage(const char *program)
{
    std::cout
        << "Usage: " << program << " [options]\n"
        << "  --udp                   Receive UDP instead of native USB capture\n"
        << "  --bind ADDRESS          UDP local address (default 0.0.0.0)\n"
        << "  --port PORT             UDP local port (default 1234)\n"
        << "  --file PATH             Read little-endian CS16 instead of USB or UDP\n"
        << "  --frequency HZ          Native tuning frequency or input center\n"
        << "  --channel 37|38|39      Advertising channel, repeat for more than one\n"
        << "  --device INDEX          Native USB device index (default 0)\n"
        << "  --sample-rate RATE      40000000, 80000000 or 160000000\n"
        << "  --manual                Use native gain codes LNA 2, filter 8, digital 0\n"
        << "  --recovery COUNT        Native restart budget (default 3)\n"
        << "  --sweep                 Monitor all primary advertising channels\n"
        << "  --dwell-ms MS           Sweep dwell per channel (default 1000)\n"
        << "  --include-invalid       Print access-address hits with a bad CRC\n"
        << "  --compact               Print one compact line per packet\n"
        << "  --interactive           Show a deduplicated live terminal table\n"
        << "  --verbose               Print rejected packet diagnostics to stderr\n"
        << "  --self-test             Test PHY, CRC, whitening and AirTag parsing\n"
        << "  --help                  Show this help\n"
        << "Native USB capture is the default. The 80 MSps mode is experimental.\n"
        << "The 160 MSps mode is experimental and expected to overrun CS16 USB.\n";
}

bool parse_u64(const char *text, uint64_t maximum, bool allow_zero, uint64_t &value)
{
    char *end = nullptr;
    errno = 0;
    if (!text || *text < '0' || *text > '9')
        return false;
    unsigned long long number = std::strtoull(text, &end, 10);
    if (errno || *end || (!allow_zero && number == 0) || number > maximum)
        return false;
    value = static_cast<uint64_t>(number);
    return true;
}

uint64_t channel_frequency(unsigned channel)
{
    switch (channel) {
    case 37: return UINT64_C(2402000000);
    case 38: return UINT64_C(2426000000);
    case 39: return UINT64_C(2480000000);
    default: return 0;
    }
}

uint64_t frequency_distance(uint64_t first, uint64_t second)
{
    return first >= second ? first - second : second - first;
}

bool parse_options(int argc, char **argv, Options &options)
{
    for (int i = 1; i < argc; ++i) {
        std::string option = argv[i];
        auto value = [&]() -> const char * { return ++i < argc ? argv[i] : nullptr; };
        if (option == "--help") {
            usage(argv[0]);
            std::exit(0);
        } else if (option == "--verbose") {
            options.verbose = true;
        } else if (option == "--compact") {
            options.compact = true;
        } else if (option == "--interactive") {
            options.interactive = true;
        } else if (option == "--udp") {
            options.udp = true;
        } else if (option == "--manual") {
            options.manual_gain = true;
        } else if (option == "--sweep") {
            options.sweep = true;
        } else if (option == "--include-invalid") {
            options.include_invalid = true;
        } else if (option == "--self-test") {
            options.self_test = true;
        } else if (option == "--bind") {
            const char *arg = value();
            if (!arg)
                return false;
            options.bind = arg;
        } else if (option == "--port") {
            const char *arg = value();
            uint64_t port = 0;
            if (!arg || !parse_u64(arg, 65535, false, port))
                return false;
            options.port = arg;
        } else if (option == "--file") {
            const char *arg = value();
            if (!arg)
                return false;
            options.file = arg;
        } else if (option == "--frequency") {
            const char *arg = value();
            if (!arg || !parse_u64(arg, UINT64_MAX, false, options.center_frequency))
                return false;
            options.frequency_set = true;
        } else if (option == "--channel") {
            const char *arg = value();
            uint64_t channel = 0;
            if (!arg || !parse_u64(arg, 39, false, channel) || channel < 37)
                return false;
            options.channels.push_back(static_cast<unsigned>(channel));
        } else if (option == "--device") {
            const char *arg = value();
            uint64_t number = 0;
            if (!arg || !parse_u64(arg, UINT32_MAX, true, number))
                return false;
            options.device_index = static_cast<uint32_t>(number);
        } else if (option == "--sample-rate") {
            const char *arg = value();
            uint64_t number = 0;
            if (!arg || !parse_u64(arg, UINT32_MAX, false, number)
                || !birria_native::valid_sample_rate(static_cast<uint32_t>(number)))
                return false;
            options.sample_rate = static_cast<uint32_t>(number);
        } else if (option == "--recovery") {
            const char *arg = value();
            uint64_t number = 0;
            if (!arg || !parse_u64(arg, UINT32_MAX, true, number))
                return false;
            options.recovery_restarts = static_cast<uint32_t>(number);
        } else if (option == "--dwell-ms") {
            const char *arg = value();
            uint64_t number = 0;
            if (!arg || !parse_u64(arg, UINT32_MAX, false, number))
                return false;
            options.dwell_ms = static_cast<uint32_t>(number);
            options.dwell_set = true;
        } else {
            return false;
        }
    }
    std::sort(options.channels.begin(), options.channels.end());
    options.channels.erase(std::unique(options.channels.begin(), options.channels.end()),
                           options.channels.end());
    if (options.compact && options.interactive)
        return false;
    if (options.interactive && options.verbose)
        return false;
    if (options.udp && !options.file.empty())
        return false;
    if (options.sweep && (options.udp || !options.file.empty()
        || options.frequency_set || !options.channels.empty()))
        return false;
    if (options.dwell_set && !options.sweep)
        return false;
    if ((options.manual_gain || options.device_index || options.recovery_restarts != 3)
        && (options.udp || !options.file.empty()))
        return false;
    return true;
}

std::string hex_bytes(const uint8_t *data, size_t length)
{
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (size_t i = 0; i < length; ++i)
        output << std::setw(2) << static_cast<unsigned>(data[i]);
    return output.str();
}

std::string quoted(const uint8_t *data, size_t length)
{
    std::ostringstream output;
    output << '"';
    for (size_t i = 0; i < length; ++i) {
        unsigned value = data[i];
        if (value == '\\' || value == '"')
            output << '\\' << static_cast<char>(value);
        else if (value >= 0x20 && value <= 0x7e)
            output << static_cast<char>(value);
        else
            output << "\\x" << std::hex << std::setw(2) << std::setfill('0') << value
                   << std::dec;
    }
    output << '"';
    return output.str();
}

std::string address_string(const uint8_t *air_order)
{
    std::ostringstream output;
    output << std::hex << std::setfill('0');
    for (int i = 5; i >= 0; --i) {
        if (i != 5)
            output << ':';
        output << std::setw(2) << static_cast<unsigned>(air_order[i]);
    }
    return output.str();
}

std::string random_address_kind(const uint8_t *air_order)
{
    switch (air_order[5] >> 6) {
    case 0: return "non-resolvable-private";
    case 1: return "resolvable-private";
    case 3: return "static";
    default: return "reserved";
    }
}

const char *pdu_name(unsigned type)
{
    static constexpr const char *names[16] = {
        "ADV_IND", "ADV_DIRECT_IND", "ADV_NONCONN_IND", "SCAN_REQ",
        "SCAN_RSP", "CONNECT_IND", "ADV_SCAN_IND", "ADV_EXT_IND",
        "AUX_CONNECT_RSP", "RFU", "RFU", "RFU", "RFU", "RFU", "RFU", "RFU"
    };
    return names[type & 15];
}

const char *ad_type_name(unsigned type)
{
    switch (type) {
    case 0x01: return "flags";
    case 0x02: return "uuid16_incomplete";
    case 0x03: return "uuid16_complete";
    case 0x04: return "uuid32_incomplete";
    case 0x05: return "uuid32_complete";
    case 0x06: return "uuid128_incomplete";
    case 0x07: return "uuid128_complete";
    case 0x08: return "short_name";
    case 0x09: return "complete_name";
    case 0x0a: return "tx_power";
    case 0x12: return "connection_interval";
    case 0x16: return "service_data16";
    case 0x19: return "appearance";
    case 0x1a: return "advertising_interval";
    case 0x20: return "service_data32";
    case 0x21: return "service_data128";
    case 0x24: return "uri";
    case 0xff: return "manufacturer";
    default: return "unknown";
    }
}

uint16_t get_le16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0] | static_cast<uint16_t>(data[1]) << 8);
}

uint32_t get_le24(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0] | static_cast<uint32_t>(data[1]) << 8
        | static_cast<uint32_t>(data[2]) << 16);
}

uint32_t get_le32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0] | static_cast<uint32_t>(data[1]) << 8
        | static_cast<uint32_t>(data[2]) << 16 | static_cast<uint32_t>(data[3]) << 24);
}

uint8_t reverse_bits(uint8_t value)
{
    value = static_cast<uint8_t>((value >> 4) | (value << 4));
    value = static_cast<uint8_t>(((value & 0xcc) >> 2) | ((value & 0x33) << 2));
    return static_cast<uint8_t>(((value & 0xaa) >> 1) | ((value & 0x55) << 1));
}

std::array<uint8_t, 3> ble_crc(const uint8_t *data, size_t length,
                               uint32_t initialization = ADV_CRC_INIT)
{
    uint32_t state = initialization & UINT32_C(0xffffff);
    for (size_t byte = 0; byte < length; ++byte) {
        for (unsigned bit = 0; bit < 8; ++bit) {
            bool feedback = ((state >> 23) ^ (data[byte] >> bit)) & 1;
            state = (state << 1) & UINT32_C(0xffffff);
            if (feedback)
                state ^= UINT32_C(0x00065b);
        }
    }
    return {{reverse_bits(static_cast<uint8_t>(state >> 16)),
             reverse_bits(static_cast<uint8_t>(state >> 8)),
             reverse_bits(static_cast<uint8_t>(state))}};
}

void whiten(std::vector<uint8_t> &bytes, unsigned channel)
{
    uint8_t lfsr = static_cast<uint8_t>(channel | 0x40);
    for (uint8_t &byte : bytes) {
        uint8_t output = 0;
        for (unsigned bit = 0; bit < 8; ++bit) {
            uint8_t whitening_bit = lfsr & 1;
            output |= static_cast<uint8_t>(((byte >> bit) & 1) ^ whitening_bit) << bit;
            lfsr = static_cast<uint8_t>((lfsr >> 1) ^ (whitening_bit ? 0x44 : 0));
        }
        byte = output;
    }
}

std::string uuid128_string(const uint8_t *little_endian)
{
    std::array<uint8_t, 16> value{};
    std::reverse_copy(little_endian, little_endian + 16, value.begin());
    std::string text = hex_bytes(value.data(), value.size());
    return text.substr(0, 8) + "-" + text.substr(8, 4) + "-" + text.substr(12, 4)
        + "-" + text.substr(16, 4) + "-" + text.substr(20);
}

std::string uuid128_network_string(const uint8_t *network_order)
{
    std::string text = hex_bytes(network_order, 16);
    return text.substr(0, 8) + "-" + text.substr(8, 4) + "-" + text.substr(12, 4)
        + "-" + text.substr(16, 4) + "-" + text.substr(20);
}

std::string flags_text(uint8_t flags)
{
    std::vector<std::string> names;
    if (flags & 0x01) names.emplace_back("limited-discoverable");
    if (flags & 0x02) names.emplace_back("general-discoverable");
    if (flags & 0x04) names.emplace_back("br-edr-unsupported");
    if (flags & 0x08) names.emplace_back("simultaneous-controller");
    if (flags & 0x10) names.emplace_back("simultaneous-host");
    std::ostringstream output;
    for (size_t i = 0; i < names.size(); ++i) {
        if (i) output << ',';
        output << names[i];
    }
    return output.str();
}

std::string decode_ad_element(uint8_t type, const uint8_t *data, size_t length)
{
    std::ostringstream output;
    output << "ad{type=0x" << std::hex << std::setw(2) << std::setfill('0')
           << static_cast<unsigned>(type) << std::dec << " name=" << ad_type_name(type)
           << " data=" << hex_bytes(data, length);
    if (type == 0x01 && length >= 1) {
        std::string names = flags_text(data[0]);
        output << " flags=0x" << std::hex << std::setw(2) << static_cast<unsigned>(data[0])
               << std::dec << " flag_names=" << quoted(
                      reinterpret_cast<const uint8_t *>(names.data()), names.size());
    } else if ((type == 0x08 || type == 0x09) && length) {
        output << " text=" << quoted(data, length);
    } else if (type == 0x0a && length >= 1) {
        output << " dbm=" << static_cast<int>(static_cast<int8_t>(data[0]));
    } else if ((type == 0x02 || type == 0x03) && length >= 2) {
        output << " uuids=";
        for (size_t i = 0; i + 2 <= length; i += 2) {
            if (i) output << ',';
            output << "0x" << std::hex << std::setw(4) << get_le16(data + i) << std::dec;
        }
    } else if ((type == 0x04 || type == 0x05) && length >= 4) {
        output << " uuids=";
        for (size_t i = 0; i + 4 <= length; i += 4) {
            if (i) output << ',';
            output << "0x" << std::hex << std::setw(8) << get_le32(data + i) << std::dec;
        }
    } else if ((type == 0x06 || type == 0x07) && length >= 16) {
        output << " uuids=";
        for (size_t i = 0; i + 16 <= length; i += 16) {
            if (i) output << ',';
            output << uuid128_string(data + i);
        }
    } else if (type == 0x12 && length >= 4) {
        output << " min=" << get_le16(data) << " max=" << get_le16(data + 2)
               << " units=1.25ms";
    } else if (type == 0x16 && length >= 2) {
        output << " uuid=0x" << std::hex << std::setw(4) << get_le16(data) << std::dec
               << " service_data=" << hex_bytes(data + 2, length - 2);
    } else if (type == 0x19 && length >= 2) {
        output << " appearance=" << get_le16(data);
    } else if (type == 0x1a && length >= 2) {
        output << " interval=" << get_le16(data) << " units=0.625ms";
    } else if (type == 0x20 && length >= 4) {
        output << " uuid=0x" << std::hex << std::setw(8) << get_le32(data) << std::dec
               << " service_data=" << hex_bytes(data + 4, length - 4);
    } else if (type == 0x21 && length >= 16) {
        output << " uuid=" << uuid128_string(data)
               << " service_data=" << hex_bytes(data + 16, length - 16);
    } else if (type == 0xff && length >= 2) {
        uint16_t company = get_le16(data);
        output << " company=0x" << std::hex << std::setw(4) << company << std::dec;
        if (company == 0x004c) {
            output << " company_name=Apple";
            const uint8_t *apple = data + 2;
            size_t apple_length = length - 2;
            if (apple_length >= 2)
                output << " apple_type=0x" << std::hex << std::setw(2)
                       << static_cast<unsigned>(apple[0]) << " apple_length=0x" << std::setw(2)
                       << static_cast<unsigned>(apple[1]) << std::dec;
            if (apple_length >= 26 && apple[0] == 0x12 && apple[1] == 0x19) {
                output << " apple_find_my=true airtag_candidate=true"
                       << " status=0x" << std::hex << std::setw(2)
                       << static_cast<unsigned>(apple[2]) << std::dec
                       << " key_fragment=" << hex_bytes(apple + 3, 22)
                       << " hint=0x" << std::hex << std::setw(2)
                       << static_cast<unsigned>(apple[25]) << std::dec;
                if (apple_length > 26)
                    output << " extra=" << hex_bytes(apple + 26, apple_length - 26);
            } else if (apple_length >= 23 && apple[0] == 0x02 && apple[1] == 0x15) {
                output << " ibeacon=true uuid=" << uuid128_network_string(apple + 2)
                       << " major=" << (static_cast<unsigned>(apple[18]) << 8 | apple[19])
                       << " minor=" << (static_cast<unsigned>(apple[20]) << 8 | apple[21])
                       << " measured_power_dbm="
                       << static_cast<int>(static_cast<int8_t>(apple[22]));
            }
        }
    }
    output << '}';
    return output.str();
}

std::vector<std::string> parse_ad(const uint8_t *data, size_t length)
{
    std::vector<std::string> elements;
    size_t offset = 0;
    while (offset < length) {
        uint8_t field_length = data[offset++];
        if (!field_length)
            break;
        if (field_length > length - offset) {
            std::ostringstream output;
            output << "ad{malformed=true declared=" << static_cast<unsigned>(field_length)
                   << " remaining=" << (length - offset)
                   << " data=" << hex_bytes(data + offset, length - offset) << '}';
            elements.push_back(output.str());
            break;
        }
        uint8_t type = data[offset];
        elements.push_back(decode_ad_element(type, data + offset + 1, field_length - 1));
        offset += field_length;
    }
    return elements;
}


struct CompactAdInfo {
    std::string name;
    int company = -1;
    int apple_type = -1;
    int find_my_status = -1;
    bool find_my = false;
    bool ibeacon = false;
};

CompactAdInfo compact_ad_info(const uint8_t *data, size_t length)
{
    CompactAdInfo info;
    size_t offset = 0;
    while (offset < length) {
        uint8_t field_length = data[offset++];
        if (!field_length || field_length > length - offset)
            break;
        uint8_t type = data[offset];
        const uint8_t *field = data + offset + 1;
        size_t field_bytes = field_length - 1;
        if ((type == 0x08 || type == 0x09) && field_bytes)
            info.name = quoted(field, field_bytes);
        if (type == 0xff && field_bytes >= 2) {
            info.company = get_le16(field);
            if (info.company == 0x004c && field_bytes >= 4) {
                info.apple_type = field[2];
                if (field_bytes >= 28 && field[2] == 0x12 && field[3] == 0x19) {
                    info.find_my = true;
                    info.find_my_status = field[4];
                } else if (field_bytes >= 25 && field[2] == 0x02 && field[3] == 0x15) {
                    info.ibeacon = true;
                }
            }
        }
        offset += field_length;
    }
    return info;
}

struct PacketMetrics {
    uint64_t input_sample = 0;
    double rssi_dbfs = 0;
    double cfo_hz = 0;
};

class ChannelDecoder {
public:
    ChannelDecoder(unsigned channel, uint64_t center_frequency, uint32_t input_rate,
                   uint64_t input_base, bool compact, bool verbose,
                   bool include_invalid, bool quiet, birria_tui::Screen *screen)
        : channel_(channel), center_frequency_(center_frequency), input_rate_(input_rate),
          decimation_(input_rate / static_cast<uint32_t>(CHANNEL_RATE)),
          input_base_(input_base), compact_(compact), verbose_(verbose),
          include_invalid_(include_invalid),
          quiet_(quiet), screen_(screen)
    {
        double offset = static_cast<double>(channel_frequency(channel_))
            - static_cast<double>(center_frequency_);
        oscillator_step_ = std::polar(1.0f,
            static_cast<float>(-2.0 * PI * offset / input_rate_));
        make_filter();
    }

    void feed(std::complex<float> sample, uint64_t input_index)
    {
        std::complex<float> mixed = sample * oscillator_;
        oscillator_ *= oscillator_step_;
        if ((++oscillator_count_ & 4095u) == 0)
            oscillator_ /= std::abs(oscillator_);
        stage_one_sum_ += mixed;
        if (++stage_one_count_ != decimation_)
            return;
        std::complex<float> decimated = stage_one_sum_ / static_cast<float>(decimation_);
        stage_one_sum_ = {};
        stage_one_count_ = 0;
        post_ring_[post_position_] = decimated;
        post_position_ = (post_position_ + 1) % POST_FILTER_TAPS;
        std::complex<float> filtered{};
        size_t position = post_position_;
        for (size_t tap = 0; tap < POST_FILTER_TAPS; ++tap) {
            filtered += post_taps_[tap] * post_ring_[position];
            position = (position + 1) % POST_FILTER_TAPS;
        }
        append_filtered(filtered, input_index);
    }

    uint64_t packets() const { return packets_; }
    uint64_t crc_failures() const { return crc_failures_; }
    unsigned channel() const { return channel_; }
    bool saw_find_my() const { return saw_find_my_; }

private:
    struct Candidate {
        uint64_t access_end = 0;
        bool inverted = false;
        float quality = 0;
    };

    void make_filter()
    {
        constexpr double cutoff = 800000.0 / CHANNEL_RATE;
        double total = 0;
        for (size_t i = 0; i < POST_FILTER_TAPS; ++i) {
            double x = static_cast<double>(i) - static_cast<double>(POST_FILTER_TAPS - 1) / 2.0;
            double sinc = x == 0 ? 2.0 * cutoff
                : std::sin(2.0 * PI * cutoff * x) / (PI * x);
            double window = 0.54 - 0.46 * std::cos(2.0 * PI * i / (POST_FILTER_TAPS - 1));
            post_taps_[i] = static_cast<float>(sinc * window);
            total += post_taps_[i];
        }
        for (float &tap : post_taps_)
            tap = static_cast<float>(tap / total);
    }

    float discriminator(std::complex<float> previous, std::complex<float> current) const
    {
        std::complex<float> product = std::conj(previous) * current;
        float denominator = std::sqrt(std::norm(previous) * std::norm(current)) + 1.0e-12f;
        return product.imag() / denominator;
    }

    float disc_at(uint64_t index) const
    {
        return discs_[static_cast<size_t>(index - base_)];
    }

    const std::complex<float> &sample_at(uint64_t index) const
    {
        return samples_[static_cast<size_t>(index - base_)];
    }

    float timing_quality(uint64_t access_end) const
    {
        float quality = 0;
        for (unsigned symbol = 0; symbol < 32; ++symbol) {
            uint64_t end = access_end - (31 - symbol) * SAMPLES_PER_SYMBOL;
            float sum = 0;
            for (unsigned i = 0; i < SAMPLES_PER_SYMBOL; ++i)
                sum += disc_at(end - (SAMPLES_PER_SYMBOL - 1) + i);
            quality += std::abs(sum);
        }
        return quality;
    }

    uint8_t demod_bit(uint64_t first_disc, bool inverted) const
    {
        float sum = 0;
        for (unsigned i = 0; i < SAMPLES_PER_SYMBOL; ++i)
            sum += disc_at(first_disc + i);
        uint8_t bit = sum >= 0 ? 1 : 0;
        return static_cast<uint8_t>(bit ^ inverted);
    }

    std::vector<uint8_t> demod_bytes(const Candidate &candidate, size_t count) const
    {
        std::vector<uint8_t> bytes(count);
        uint64_t first = candidate.access_end + 1;
        for (size_t byte = 0; byte < count; ++byte)
            for (unsigned bit = 0; bit < 8; ++bit)
                bytes[byte] |= demod_bit(first + (byte * 8 + bit) * SAMPLES_PER_SYMBOL,
                                         candidate.inverted) << bit;
        whiten(bytes, channel_);
        return bytes;
    }

    PacketMetrics metrics(const Candidate &candidate, size_t bytes) const
    {
        PacketMetrics result;
        uint64_t preamble_first = candidate.access_end >= 159 ? candidate.access_end - 159 : base_;
        uint64_t final_disc = candidate.access_end + bytes * 8 * SAMPLES_PER_SYMBOL;
        double power = 0;
        size_t power_count = 0;
        for (uint64_t i = preamble_first; i <= final_disc && i >= base_
             && i < base_ + samples_.size(); ++i) {
            power += std::norm(sample_at(i));
            ++power_count;
        }
        double mean_power = power_count ? power / power_count : 0;
        result.rssi_dbfs = 10.0 * std::log10(std::max(mean_power, 1.0e-30)
                                             / (32768.0 * 32768.0));
        double positive_phase = 0;
        double negative_phase = 0;
        size_t positive_count = 0;
        size_t negative_count = 0;
        uint64_t first_symbol_end = candidate.access_end >= 156
            ? candidate.access_end - 156 : base_ + 3;
        for (uint64_t end = first_symbol_end; end <= final_disc; end += SAMPLES_PER_SYMBOL) {
            double symbol_phase = 0;
            for (unsigned i = 0; i < SAMPLES_PER_SYMBOL; ++i) {
                uint64_t current = end - (SAMPLES_PER_SYMBOL - 1) + i;
                if (current <= base_ || current >= base_ + samples_.size())
                    continue;
                symbol_phase += std::arg(std::conj(sample_at(current - 1))
                                         * sample_at(current));
            }
            if (symbol_phase >= 0) {
                positive_phase += symbol_phase / SAMPLES_PER_SYMBOL;
                ++positive_count;
            } else {
                negative_phase += symbol_phase / SAMPLES_PER_SYMBOL;
                ++negative_count;
            }
        }
        if (positive_count && negative_count) {
            double high = positive_phase / positive_count;
            double low = negative_phase / negative_count;
            result.cfo_hz = (high + low) * 0.5 * CHANNEL_RATE / (2.0 * PI);
        }
        int64_t filter_delay = static_cast<int64_t>(POST_FILTER_TAPS / 2 * decimation_)
            + static_cast<int64_t>(decimation_ / 2);
        int64_t input = static_cast<int64_t>(input_base_ + preamble_first * decimation_)
            - filter_delay;
        result.input_sample = input > 0 ? static_cast<uint64_t>(input) : 0;
        return result;
    }

    void append_address(std::ostringstream &output, const char *name, const uint8_t *address,
                        bool random) const
    {
        output << ' ' << name << '=' << address_string(address)
               << ' ' << name << "_type=" << (random ? "random" : "public");
        if (random)
            output << ' ' << name << "_random_kind=" << random_address_kind(address);
    }

    void append_ad(std::ostringstream &output, const uint8_t *data, size_t length)
    {
        auto elements = parse_ad(data, length);
        for (const std::string &element : elements) {
            output << ' ' << element;
            if (element.find("apple_find_my=true") != std::string::npos)
                saw_find_my_ = true;
        }
    }

    void append_extended(std::ostringstream &output, const uint8_t *payload, size_t length,
                         bool tx_random, bool rx_random)
    {
        if (!length) {
            output << " extended_malformed=true";
            return;
        }
        size_t header_length = payload[0] & 0x3f;
        unsigned mode = payload[0] >> 6;
        output << " adv_mode=" << mode << " extended_header_length=" << header_length;
        if (header_length > length - 1) {
            output << " extended_malformed=true";
            return;
        }
        if (!header_length) {
            append_ad(output, payload + 1, length - 1);
            return;
        }
        uint8_t flags = payload[1];
        size_t offset = 2;
        size_t end = 1 + header_length;
        output << " extended_flags=0x" << std::hex << std::setw(2) << std::setfill('0')
               << static_cast<unsigned>(flags) << std::dec;
        auto take = [&](size_t count) -> const uint8_t * {
            if (offset + count > end)
                return nullptr;
            const uint8_t *field = payload + offset;
            offset += count;
            return field;
        };
        if (flags & 0x01) {
            const uint8_t *field = take(6);
            if (field) append_address(output, "advertiser", field, tx_random);
        }
        if (flags & 0x02) {
            const uint8_t *field = take(6);
            if (field) append_address(output, "target", field, rx_random);
        }
        if (flags & 0x04) {
            const uint8_t *field = take(1);
            if (field) output << " cte_info=0x" << std::hex << std::setw(2)
                              << static_cast<unsigned>(*field) << std::dec;
        }
        if (flags & 0x08) {
            const uint8_t *field = take(2);
            if (field) output << " adi=0x" << std::hex << std::setw(4) << get_le16(field)
                              << std::dec << " sid=" << (field[1] >> 4)
                              << " did=" << (get_le16(field) & 0x0fff);
        }
        if (flags & 0x10) {
            const uint8_t *field = take(3);
            if (field) {
                unsigned aux_channel = field[0] & 0x3f;
                unsigned offset_units = (field[0] >> 7) ? 300 : 30;
                unsigned aux_offset = (static_cast<unsigned>(field[2] & 0x1f) << 8)
                    | field[1];
                output << " aux_channel=" << aux_channel
                       << " aux_offset_us=" << aux_offset * offset_units
                       << " aux_phy=" << ((field[2] >> 5) & 7)
                       << " aux_clock_accuracy=" << ((field[0] >> 6) & 1);
            }
        }
        if (flags & 0x20) {
            const uint8_t *field = take(18);
            if (field) output << " sync_info=" << hex_bytes(field, 18);
        }
        if (flags & 0x40) {
            const uint8_t *field = take(1);
            if (field) output << " tx_power_dbm="
                              << static_cast<int>(static_cast<int8_t>(*field));
        }
        if (offset < end)
            output << " acad=" << hex_bytes(payload + offset, end - offset);
        if (end < length)
            append_ad(output, payload + end, length - end);
    }

    void append_connection(std::ostringstream &output, const uint8_t *data, size_t length)
    {
        if (length < 22) {
            output << " connection_data_malformed=true";
            return;
        }
        output << " connection_access_address=0x" << std::hex << std::setw(8)
               << get_le32(data) << " connection_crc_init=0x" << std::setw(6)
               << get_le24(data + 4) << std::dec
               << " window_size=" << static_cast<unsigned>(data[7])
               << " window_size_ms=" << static_cast<double>(data[7]) * 1.25
               << " window_offset=" << get_le16(data + 8)
               << " window_offset_ms=" << static_cast<double>(get_le16(data + 8)) * 1.25
               << " interval=" << get_le16(data + 10)
               << " interval_ms=" << static_cast<double>(get_le16(data + 10)) * 1.25
               << " latency=" << get_le16(data + 12)
               << " timeout=" << get_le16(data + 14)
               << " timeout_ms=" << static_cast<unsigned>(get_le16(data + 14)) * 10u
               << " channel_map=" << hex_bytes(data + 16, 5)
               << " hop=" << (data[21] & 0x1f)
               << " sca=" << (data[21] >> 5);
    }

    void print_packet(const Candidate &candidate, const std::vector<uint8_t> &bytes,
                      bool crc_valid, const std::array<uint8_t, 3> &calculated)
    {
        unsigned type = bytes[0] & 0x0f;
        bool chsel = (bytes[0] & 0x20) != 0;
        bool tx_random = (bytes[0] & 0x40) != 0;
        bool rx_random = (bytes[0] & 0x80) != 0;
        size_t length = bytes[1] & 0x3f;
        PacketMetrics packet_metrics = metrics(candidate, bytes.size());
        const uint8_t *payload = bytes.data() + 2;
        if (compact_ || screen_) {
            std::ostringstream compact;
            std::string row_key = std::string(pdu_name(type)) + "|";
            std::string primary_address;
            CompactAdInfo ad_info;
            bool have_ad_info = false;
            compact << std::fixed << std::setprecision(6)
                    << "ble time=" << static_cast<double>(packet_metrics.input_sample) / input_rate_
                    << "s sample=" << packet_metrics.input_sample
                    << " ch=" << channel_ << " pdu=" << pdu_name(type)
                    << " len=" << length;
            const uint8_t *ad = nullptr;
            size_t ad_length = 0;
            if ((type == 0 || type == 2 || type == 4 || type == 6) && length >= 6) {
                std::string address = address_string(payload);
                compact << " addr=" << address;
                row_key = "device|" + address;
                primary_address = address;
                ad = payload + 6;
                ad_length = length - 6;
            } else if (type == 1 && length >= 12) {
                std::string address = address_string(payload);
                std::string target = address_string(payload + 6);
                compact << " addr=" << address << " target=" << target;
                row_key += address + "|" + target;
                primary_address = address;
            } else if (type == 3 && length >= 12) {
                std::string scanner = address_string(payload);
                std::string address = address_string(payload + 6);
                compact << " scanner=" << scanner << " addr=" << address;
                row_key += scanner + "|" + address;
                primary_address = address;
            } else if (type == 5 && length >= 12) {
                std::string initiator = address_string(payload);
                std::string address = address_string(payload + 6);
                compact << " initiator=" << initiator << " addr=" << address;
                row_key += initiator + "|" + address;
                primary_address = address;
            }
            if (ad) {
                ad_info = compact_ad_info(ad, ad_length);
                have_ad_info = true;
                if (!ad_info.name.empty())
                    compact << " name=" << ad_info.name;
                if (ad_info.company >= 0)
                    compact << " company=0x" << std::hex << std::setw(4)
                            << std::setfill('0') << ad_info.company << std::dec;
                if (ad_info.apple_type >= 0)
                    compact << " apple=0x" << std::hex << std::setw(2)
                            << ad_info.apple_type << std::dec;
                if (ad_info.find_my)
                    compact << " find_my=1 status=0x" << std::hex << std::setw(2)
                            << ad_info.find_my_status << std::dec;
                if (ad_info.ibeacon)
                    compact << " ibeacon=1";
            }
            compact << " rssi=" << std::setprecision(1) << packet_metrics.rssi_dbfs
                    << "dBFS crc=" << (crc_valid ? "ok" : "bad");
            if (screen_) {
                if (row_key.back() == '|')
                    row_key += hex_bytes(payload, std::min<size_t>(length, 8));
                std::ostringstream center;
                center << std::fixed << std::setprecision(3)
                       << static_cast<double>(channel_frequency(channel_)) / 1000000.0;
                std::ostringstream rssi;
                rssi << std::fixed << std::setprecision(1) << packet_metrics.rssi_dbfs;
                std::ostringstream details;
                details << "len=" << length;
                if (have_ad_info && !ad_info.name.empty())
                    details << " name=" << ad_info.name;
                if (have_ad_info && ad_info.company >= 0)
                    details << " company=0x" << std::hex << std::setw(4)
                            << std::setfill('0') << ad_info.company << std::dec;
                if (have_ad_info && ad_info.apple_type >= 0)
                    details << " apple=0x" << std::hex << std::setw(2)
                            << ad_info.apple_type << std::dec;
                if (have_ad_info && ad_info.find_my)
                    details << " find_my=1 status=0x" << std::hex << std::setw(2)
                            << ad_info.find_my_status << std::dec;
                if (have_ad_info && ad_info.ibeacon)
                    details << " ibeacon=1";
                details << " sample=" << packet_metrics.input_sample
                        << " cfo=" << std::fixed << std::setprecision(0)
                        << packet_metrics.cfo_hz << "Hz crc="
                        << (crc_valid ? "ok" : "bad");
                screen_->update(row_key, {center.str(), std::to_string(channel_),
                    pdu_name(type), rssi.str(), primary_address, details.str()});
            } else if (!quiet_) {
                std::cout << compact.str() << '\n';
                std::cout.flush();
            }
            return;
        }
        std::ostringstream output;
        output << std::fixed << std::setprecision(6)
               << "ble time=" << static_cast<double>(packet_metrics.input_sample) / input_rate_
               << "s sample=" << packet_metrics.input_sample
               << " channel=" << channel_
               << " frequency=" << channel_frequency(channel_)
               << " phy=LE1M rssi_dbfs=" << std::setprecision(1) << packet_metrics.rssi_dbfs
               << " cfo_hz=" << std::setprecision(0) << packet_metrics.cfo_hz
               << " access_address=0x" << std::hex << std::setw(8) << std::setfill('0')
               << ADV_ACCESS_ADDRESS << std::dec
               << " crc=" << (crc_valid ? "ok" : "bad")
               << " pdu=" << pdu_name(type) << " pdu_type=" << type
               << " length=" << length
               << " tx_add=" << (tx_random ? "random" : "public")
               << " rx_add=" << (rx_random ? "random" : "public")
               << " chsel=" << chsel;
        if ((type == 0 || type == 2 || type == 4 || type == 6) && length >= 6) {
            append_address(output, "advertiser", payload, tx_random);
            append_ad(output, payload + 6, length - 6);
        } else if (type == 1 && length >= 12) {
            append_address(output, "advertiser", payload, tx_random);
            append_address(output, "target", payload + 6, rx_random);
        } else if (type == 3 && length >= 12) {
            append_address(output, "scanner", payload, tx_random);
            append_address(output, "advertiser", payload + 6, rx_random);
        } else if (type == 5 && length >= 34) {
            append_address(output, "initiator", payload, tx_random);
            append_address(output, "advertiser", payload + 6, rx_random);
            append_connection(output, payload + 12, length - 12);
        } else if (type == 7) {
            append_extended(output, payload, length, tx_random, rx_random);
        } else if (length) {
            output << " payload=" << hex_bytes(payload, length);
        }
        output << " pdu_hex=" << hex_bytes(bytes.data(), 2 + length)
               << " crc_received=" << hex_bytes(bytes.data() + 2 + length, 3)
               << " crc_calculated=" << hex_bytes(calculated.data(), calculated.size());
        if (!quiet_) {
            std::cout << output.str() << '\n';
            std::cout.flush();
        }
    }

    bool process_candidate(const Candidate &candidate, uint64_t newest)
    {
        constexpr size_t header_bytes = 2;
        uint64_t header_end = candidate.access_end
            + header_bytes * 8 * SAMPLES_PER_SYMBOL;
        if (newest < header_end)
            return false;
        std::vector<uint8_t> header = demod_bytes(candidate, header_bytes);
        size_t length = header[1] & 0x3f;
        if (length > MAX_ADV_PAYLOAD)
            return true;
        size_t total = header_bytes + length + 3;
        uint64_t packet_end = candidate.access_end + total * 8 * SAMPLES_PER_SYMBOL;
        if (newest < packet_end)
            return false;
        std::vector<uint8_t> bytes = demod_bytes(candidate, total);
        auto calculated = ble_crc(bytes.data(), header_bytes + length);
        bool valid = std::equal(calculated.begin(), calculated.end(),
                                bytes.begin() + static_cast<std::ptrdiff_t>(header_bytes + length));
        bool duplicate = valid && last_valid_ != std::numeric_limits<uint64_t>::max()
            && candidate.access_end <= last_valid_ + SAMPLES_PER_SYMBOL * 2;
        if (!valid) {
            ++crc_failures_;
            if (verbose_)
                std::cerr << "Rejected BLE candidate on channel " << channel_
                          << " at input sample " << metrics(candidate, total).input_sample
                          << ": CRC mismatch\n";
        } else if (!duplicate) {
            ++packets_;
            last_valid_ = candidate.access_end;
        }
        if ((!duplicate && valid) || (!valid && include_invalid_))
            print_packet(candidate, bytes, valid, calculated);
        return true;
    }

    void append_filtered(std::complex<float> sample, uint64_t input_index)
    {
        (void)input_index;
        uint64_t index = next_filtered_++;
        if (samples_.empty())
            base_ = index;
        float disc = samples_.empty() ? 0.0f : discriminator(samples_.back(), sample);
        samples_.push_back(sample);
        discs_.push_back(disc);
        if (index >= base_ + 3) {
            float sum = disc_at(index) + disc_at(index - 1) + disc_at(index - 2)
                + disc_at(index - 3);
            unsigned phase = static_cast<unsigned>(index % SAMPLES_PER_SYMBOL);
            uint8_t bit = sum >= 0 ? 1 : 0;
            shifts_[phase] = (shifts_[phase] >> 1) | (static_cast<uint32_t>(bit) << 31);
            if (shifts_[phase] == ADV_ACCESS_ADDRESS
                || shifts_[phase] == ~ADV_ACCESS_ADDRESS) {
                Candidate candidate{index, shifts_[phase] == ~ADV_ACCESS_ADDRESS,
                                    timing_quality(index)};
                if (!candidates_.empty()
                    && index <= candidates_.back().access_end + SAMPLES_PER_SYMBOL) {
                    if (candidate.quality > candidates_.back().quality)
                        candidates_.back() = candidate;
                } else {
                    candidates_.push_back(candidate);
                }
            }
        }
        for (auto candidate = candidates_.begin(); candidate != candidates_.end();) {
            if (process_candidate(*candidate, index))
                candidate = candidates_.erase(candidate);
            else
                ++candidate;
        }
        if (samples_.size() > 32768) {
            uint64_t keep_from = index - 8192;
            if (!candidates_.empty())
                keep_from = std::min(keep_from,
                    candidates_.front().access_end > 192 ? candidates_.front().access_end - 192 : 0);
            if (keep_from > base_) {
                size_t remove = static_cast<size_t>(keep_from - base_);
                samples_.erase(samples_.begin(), samples_.begin() + static_cast<std::ptrdiff_t>(remove));
                discs_.erase(discs_.begin(), discs_.begin() + static_cast<std::ptrdiff_t>(remove));
                base_ = keep_from;
            }
        }
    }

    unsigned channel_;
    uint64_t center_frequency_;
    uint32_t input_rate_;
    unsigned decimation_;
    uint64_t input_base_;
    bool compact_;
    bool verbose_;
    bool include_invalid_;
    bool quiet_;
    birria_tui::Screen *screen_;
    std::complex<float> oscillator_{1.0f, 0.0f};
    std::complex<float> oscillator_step_{1.0f, 0.0f};
    uint64_t oscillator_count_ = 0;
    std::complex<float> stage_one_sum_{};
    unsigned stage_one_count_ = 0;
    std::array<float, POST_FILTER_TAPS> post_taps_{};
    std::array<std::complex<float>, POST_FILTER_TAPS> post_ring_{};
    size_t post_position_ = 0;
    std::vector<std::complex<float>> samples_;
    std::vector<float> discs_;
    uint64_t base_ = 0;
    uint64_t next_filtered_ = 0;
    std::array<uint32_t, SAMPLES_PER_SYMBOL> shifts_{};
    std::deque<Candidate> candidates_;
    uint64_t last_valid_ = std::numeric_limits<uint64_t>::max();
    uint64_t packets_ = 0;
    uint64_t crc_failures_ = 0;
    bool saw_find_my_ = false;
};

class Scanner {
public:
    Scanner(const Options &options, uint64_t input_base = 0, bool quiet = false,
            birria_tui::Screen *screen = nullptr)
    {
        for (unsigned channel : options.channels)
            decoders_.emplace_back(channel, options.center_frequency, options.sample_rate,
                                   input_base, options.compact, options.verbose,
                                   options.include_invalid, quiet, screen);
    }

    void feed_cs16(const uint8_t *bytes, size_t length)
    {
        if (length % 4) {
            ++bad_datagrams_;
            return;
        }
        for (size_t offset = 0; offset < length; offset += 4)
            feed_scalar(bytes + offset);
    }

    void feed_iq(const int16_t *iq, uint32_t count)
    {
        for (uint32_t i = 0; i < count; ++i)
            feed_sample({static_cast<float>(iq[2 * i]), static_cast<float>(iq[2 * i + 1])});
    }

    void feed_cs16_stream(const uint8_t *bytes, size_t length)
    {
        size_t offset = 0;
        if (byte_count_) {
            while (byte_count_ < 4 && offset < length)
                byte_buffer_[byte_count_++] = bytes[offset++];
            if (byte_count_ == 4) {
                feed_scalar(byte_buffer_.data());
                byte_count_ = 0;
            }
        }
        while (offset + 4 <= length) {
            feed_scalar(bytes + offset);
            offset += 4;
        }
        while (offset < length)
            byte_buffer_[byte_count_++] = bytes[offset++];
    }

    uint64_t input_samples() const { return input_samples_; }
    uint64_t bad_datagrams() const { return bad_datagrams_; }
    size_t trailing_bytes() const { return byte_count_; }
    uint64_t packets() const
    {
        uint64_t total = 0;
        for (const auto &decoder : decoders_) total += decoder.packets();
        return total;
    }
    uint64_t crc_failures() const
    {
        uint64_t total = 0;
        for (const auto &decoder : decoders_) total += decoder.crc_failures();
        return total;
    }
    bool saw_find_my() const
    {
        return std::any_of(decoders_.begin(), decoders_.end(),
                           [](const ChannelDecoder &decoder) { return decoder.saw_find_my(); });
    }

private:
    void feed_scalar(const uint8_t *bytes)
    {
        int16_t i = static_cast<int16_t>(bytes[0] | static_cast<uint16_t>(bytes[1]) << 8);
        int16_t q = static_cast<int16_t>(bytes[2] | static_cast<uint16_t>(bytes[3]) << 8);
        feed_sample({static_cast<float>(i), static_cast<float>(q)});
    }

    void feed_sample(std::complex<float> sample)
    {
        for (auto &decoder : decoders_)
            decoder.feed(sample, input_samples_);
        ++input_samples_;
    }

    std::vector<ChannelDecoder> decoders_;
    std::array<uint8_t, 4> byte_buffer_{};
    size_t byte_count_ = 0;
    uint64_t input_samples_ = 0;
    uint64_t bad_datagrams_ = 0;
};

int open_udp_receiver(const Options &options)
{
    addrinfo hints{};
    hints.ai_flags = AI_PASSIVE;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo *addresses = nullptr;
    int status = getaddrinfo(options.bind.c_str(), options.port.c_str(), &hints, &addresses);
    if (status) {
        std::cerr << "Bind address: " << gai_strerror(status) << '\n';
        return -1;
    }
    int fd = -1;
    for (addrinfo *address = addresses; address; address = address->ai_next) {
        fd = socket(address->ai_family, address->ai_socktype, address->ai_protocol);
        if (fd < 0)
            continue;
        int reuse = 1;
        (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        int receive_buffer = 16 * 1024 * 1024;
        (void)setsockopt(fd, SOL_SOCKET, SO_RCVBUF, &receive_buffer, sizeof(receive_buffer));
        timeval timeout{0, 200000};
        (void)setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        if (bind(fd, address->ai_addr, address->ai_addrlen) == 0)
            break;
        close(fd);
        fd = -1;
    }
    freeaddrinfo(addresses);
    if (fd < 0)
        std::perror("UDP bind");
    return fd;
}

std::vector<uint8_t> make_find_my_pdu()
{
    std::vector<uint8_t> pdu;
    pdu.push_back(0x42);
    pdu.push_back(37);
    const uint8_t address[6] = {0x21, 0x32, 0x43, 0x54, 0x65, 0xf6};
    pdu.insert(pdu.end(), address, address + 6);
    pdu.push_back(0x1e);
    pdu.push_back(0xff);
    pdu.push_back(0x4c);
    pdu.push_back(0x00);
    pdu.push_back(0x12);
    pdu.push_back(0x19);
    pdu.push_back(0x00);
    for (unsigned i = 0; i < 22; ++i)
        pdu.push_back(static_cast<uint8_t>(0x80 + i));
    pdu.push_back(0x02);
    pdu.push_back(0x00);
    return pdu;
}

std::vector<uint8_t> synthesize_packet(const std::vector<uint8_t> &pdu, unsigned channel,
                                       uint32_t input_rate, double carrier_offset_hz = 0)
{
    std::vector<uint8_t> packet = pdu;
    auto crc = ble_crc(pdu.data(), pdu.size());
    packet.insert(packet.end(), crc.begin(), crc.end());
    whiten(packet, channel);
    std::vector<uint8_t> bits;
    auto append_byte = [&](uint8_t byte) {
        for (unsigned bit = 0; bit < 8; ++bit)
            bits.push_back((byte >> bit) & 1);
    };
    append_byte(0xaa);
    append_byte(0xd6);
    append_byte(0xbe);
    append_byte(0x89);
    append_byte(0x8e);
    for (uint8_t byte : packet)
        append_byte(byte);
    unsigned input_samples_per_symbol = static_cast<unsigned>(input_rate / BLE_RATE);
    size_t leading_silence = 400;
    size_t trailing_silence = 12000;
    std::vector<uint8_t> cs16((leading_silence + trailing_silence
        + bits.size() * input_samples_per_symbol) * 4);
    double phase = 0;
    size_t sample_index = leading_silence;
    for (uint8_t bit : bits) {
        double frequency = carrier_offset_hz + (bit ? 250000.0 : -250000.0);
        double step = 2.0 * PI * frequency / input_rate;
        for (unsigned i = 0; i < input_samples_per_symbol; ++i) {
            phase += step;
            int16_t in_phase = static_cast<int16_t>(std::lround(5000.0 * std::cos(phase)));
            int16_t quadrature = static_cast<int16_t>(std::lround(5000.0 * std::sin(phase)));
            size_t offset = sample_index++ * 4;
            cs16[offset] = static_cast<uint8_t>(in_phase);
            cs16[offset + 1] = static_cast<uint8_t>(static_cast<uint16_t>(in_phase) >> 8);
            cs16[offset + 2] = static_cast<uint8_t>(quadrature);
            cs16[offset + 3] = static_cast<uint8_t>(static_cast<uint16_t>(quadrature) >> 8);
        }
    }
    return cs16;
}

bool self_test(uint32_t sample_rate)
{
    static const uint8_t crc_input[] = {0x42, 0x06, 1, 2, 3, 4, 5, 6};
    auto crc = ble_crc(crc_input, sizeof(crc_input));
    if (crc != std::array<uint8_t, 3>{{0xd8, 0x82, 0x98}}) {
        std::cerr << "self-test: CRC vector mismatch, got "
                  << hex_bytes(crc.data(), crc.size()) << '\n';
        return false;
    }
    std::vector<uint8_t> whitening_test{0x42, 0x25, 0x01, 0x02, 0x03};
    std::vector<uint8_t> original = whitening_test;
    whiten(whitening_test, 38);
    whiten(whitening_test, 38);
    if (whitening_test != original) {
        std::cerr << "self-test: whitening round trip mismatch\n";
        return false;
    }
    Options options;
    options.sample_rate = sample_rate;
    options.center_frequency = UINT64_C(2414000000);
    options.channels = {38};
    Scanner scanner(options, 0, true);
    double offset = static_cast<double>(channel_frequency(38) - options.center_frequency) + 3750.0;
    auto packet = synthesize_packet(make_find_my_pdu(), 38, options.sample_rate, offset);
    scanner.feed_cs16_stream(packet.data(), packet.size());
    if (scanner.packets() != 1 || !scanner.saw_find_my()) {
        std::cerr << "self-test: PHY or Apple Find My decode mismatch, packets="
                  << scanner.packets() << ", CRC failures=" << scanner.crc_failures() << '\n';
        return false;
    }
    return true;
}

void handle_signal(int)
{
    interrupted = true;
}

bool select_channels(Options &options)
{
    uint64_t maximum_offset = options.sample_rate / 2u - 1000000u;
    if (options.channels.empty()) {
        for (unsigned channel = 37; channel <= 39; ++channel) {
            uint64_t offset = frequency_distance(channel_frequency(channel),
                                                 options.center_frequency);
            if (offset <= maximum_offset)
                options.channels.push_back(channel);
        }
    }
    if (options.channels.empty()) {
        std::cerr << "No BLE advertising channel is within the usable span of the I/Q center.\n";
        return false;
    }
    for (unsigned channel : options.channels) {
        uint64_t offset = frequency_distance(channel_frequency(channel),
                                             options.center_frequency);
        if (offset > maximum_offset) {
            std::cerr << "BLE channel " << channel << " at " << channel_frequency(channel)
                      << " Hz is outside the supported channelizer offset at "
                      << options.sample_rate << " S/s.\n";
            return false;
        }
    }
    return true;
}

struct ScannerStats {
    uint64_t input_samples = 0;
    uint64_t packets = 0;
    uint64_t crc_failures = 0;
    uint64_t bad_datagrams = 0;
    bool saw_find_my = false;

    ScannerStats &operator+=(const ScannerStats &other)
    {
        input_samples += other.input_samples;
        packets += other.packets;
        crc_failures += other.crc_failures;
        bad_datagrams += other.bad_datagrams;
        saw_find_my = saw_find_my || other.saw_find_my;
        return *this;
    }
};

ScannerStats scanner_stats(const Scanner &scanner)
{
    return {scanner.input_samples(), scanner.packets(), scanner.crc_failures(),
            scanner.bad_datagrams(), scanner.saw_find_my()};
}

class NativeBleContext {
public:
    NativeBleContext(const Options &options, birria_tui::Screen *screen)
        : options_(options), screen_(screen), settle_samples_(options.sample_rate / 10),
          software_dwell_samples_(static_cast<uint64_t>(options.sample_rate)
                                  * options.dwell_ms / 1000),
          worker_(&NativeBleContext::worker_loop, this)
    {
    }

    static void samples(const int16_t *iq, uint32_t count, uint64_t first_sample,
                        uint64_t frequency, uint64_t epoch, void *context)
    {
        static_cast<NativeBleContext *>(context)->enqueue(
            iq, count, first_sample, frequency, epoch);
    }

    static void gap(const birriasdr_recovery_event *event, uint64_t frequency,
                    uint64_t epoch, void *context)
    {
        NativeBleContext *native = static_cast<NativeBleContext *>(context);
        std::ostringstream message;
        message << "Gap before sample " << event->next_sample << ", epoch " << epoch
                << ", center " << frequency << " Hz: "
                << birriasdr_strerror(event->error);
        if (native->screen_)
            native->screen_->set_status(message.str());
        else
            std::cerr << message.str() << '\n';
    }

    bool dropped() const { return dropped_samples_ != 0; }

    ScannerStats finish()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (worker_.joinable())
            worker_.join();
        if (scanner_)
            totals_ += scanner_stats(*scanner_);
        scanner_.reset();
        if (dropped_samples_) {
            std::ostringstream message;
            message << "Native decoder queue dropped " << dropped_samples_ << " samples.";
            if (screen_)
                screen_->set_status(message.str());
            else
                std::cerr << message.str() << '\n';
        }
        return totals_;
    }

private:
    struct Chunk {
        std::vector<int16_t> iq;
        uint32_t count = 0;
        uint64_t first_sample = 0;
        uint64_t frequency = 0;
        uint64_t epoch = 0;
        bool discontinuity = false;
    };

    void enqueue(const int16_t *iq, uint32_t count, uint64_t first_sample,
                 uint64_t frequency, uint64_t epoch)
    {
        Chunk chunk;
        chunk.iq.assign(iq, iq + static_cast<size_t>(count) * 2);
        chunk.count = count;
        chunk.first_sample = first_sample;
        chunk.frequency = frequency;
        chunk.epoch = epoch;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (queue_.size() >= 256) {
                for (const Chunk &queued : queue_)
                    dropped_samples_ += queued.count;
                queue_.clear();
                chunk.discontinuity = true;
            }
            queue_.push_back(std::move(chunk));
        }
        condition_.notify_one();
    }

    void worker_loop()
    {
        for (;;) {
            Chunk chunk;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] { return stopping_ || !queue_.empty(); });
                if (queue_.empty()) {
                    if (stopping_)
                        break;
                    continue;
                }
                if (!stopping_ && queue_.size() < 8)
                    condition_.wait_for(lock, std::chrono::milliseconds(2));
                chunk = std::move(queue_.front());
                queue_.pop_front();
                chunk.iq.reserve(131072 * 2);
                while (!queue_.empty() && !queue_.front().discontinuity
                       && queue_.front().epoch == chunk.epoch
                       && queue_.front().frequency == chunk.frequency
                       && chunk.count + queue_.front().count <= 131072) {
                    Chunk next = std::move(queue_.front());
                    queue_.pop_front();
                    chunk.iq.insert(chunk.iq.end(), next.iq.begin(), next.iq.end());
                    chunk.count += next.count;
                }
            }
            if (chunk.discontinuity)
                epoch_ = std::numeric_limits<uint64_t>::max();
            feed(chunk.iq.data(), chunk.count, chunk.first_sample,
                 chunk.frequency, chunk.epoch);
        }
    }

    std::unique_ptr<Scanner> make_scanner(uint64_t frequency, uint64_t input_base)
    {
        Options active = options_;
        active.center_frequency = frequency;
        if (active.sweep) {
            active.channels.clear();
            if (active.sample_rate >= 80000000u) {
                active.channels.push_back(software_channel_);
            } else {
                for (unsigned channel = 37; channel <= 39; ++channel)
                    if (channel_frequency(channel) == frequency)
                        active.channels.push_back(channel);
            }
        }
        if (!select_channels(active)) {
            failed_ = true;
            return {};
        }
        return std::make_unique<Scanner>(active, input_base, false, screen_);
    }

    void feed(const int16_t *iq, uint32_t count, uint64_t first_sample,
              uint64_t frequency, uint64_t epoch)
    {
        if (!scanner_) {
            scanner_ = make_scanner(frequency, first_sample);
            epoch_ = epoch;
        } else if (epoch != epoch_) {
            totals_ += scanner_stats(*scanner_);
            scanner_.reset();
            epoch_ = epoch;
            pending_frequency_ = frequency;
            discard_remaining_ = settle_samples_;
        }
        uint32_t skip = static_cast<uint32_t>(std::min<uint64_t>(count, discard_remaining_));
        discard_remaining_ -= skip;
        iq += static_cast<size_t>(skip) * 2;
        count -= skip;
        first_sample += skip;
        if (!count || failed_)
            return;
        if (!scanner_)
            scanner_ = make_scanner(pending_frequency_, first_sample);
        if (!scanner_)
            return;
        if (!options_.sweep || options_.sample_rate < 80000000u) {
            scanner_->feed_iq(iq, count);
            return;
        }
        uint32_t done = 0;
        while (done < count) {
            uint64_t remaining = software_dwell_samples_ - software_samples_;
            uint32_t take = static_cast<uint32_t>(std::min<uint64_t>(count - done, remaining));
            scanner_->feed_iq(iq + static_cast<size_t>(done) * 2, take);
            done += take;
            software_samples_ += take;
            if (software_samples_ == software_dwell_samples_) {
                totals_ += scanner_stats(*scanner_);
                scanner_.reset();
                software_samples_ = 0;
                software_channel_ = software_channel_ == 39 ? 37 : software_channel_ + 1;
                if (done < count)
                    scanner_ = make_scanner(frequency, first_sample + done);
            }
        }
    }

    Options options_;
    birria_tui::Screen *screen_;
    uint64_t settle_samples_;
    uint64_t software_dwell_samples_;
    uint64_t software_samples_ = 0;
    unsigned software_channel_ = 37;
    uint64_t discard_remaining_ = 0;
    uint64_t pending_frequency_ = 0;
    uint64_t epoch_ = std::numeric_limits<uint64_t>::max();
    bool failed_ = false;
    std::unique_ptr<Scanner> scanner_;
    ScannerStats totals_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<Chunk> queue_;
    bool stopping_ = false;
    uint64_t dropped_samples_ = 0;
    std::thread worker_;
};

} // namespace

int main(int argc, char **argv)
{
    Options options;
    if (!parse_options(argc, argv, options)) {
        usage(argv[0]);
        return 2;
    }
    if (options.self_test) {
        bool passed = self_test(options.sample_rate);
        std::cout << (passed ? "BLE scanner self-test passed\n" : "BLE scanner self-test failed\n");
        return passed ? 0 : 1;
    }
    ::signal(SIGINT, handle_signal);
    ::signal(SIGTERM, handle_signal);
    std::unique_ptr<birria_tui::Screen> screen;
    if (options.interactive) {
        if (!birria_tui::Screen::terminal_available()) {
            std::cerr << "Interactive mode requires a terminal on standard output.\n";
            return 2;
        }
        screen = std::make_unique<birria_tui::Screen>("BirriaSDR BLE monitor",
            std::vector<birria_tui::Column>{
                {"Center/MHz", 10, true}, {"Ch", 3, true},
                {"PDU", 17, false}, {"RSSI/dBFS", 10, true},
                {"Address", 17, false}, {"Details", 0, false}
            });
        screen->set_status("Starting capture. Ctrl-C to stop.");
    }
    auto show_status = [&](const std::string &message) {
        if (screen)
            screen->set_status(message);
        else
            std::cerr << message << '\n';
    };

    ScannerStats totals;
    bool input_failed = false;
    std::array<uint8_t, 65536> buffer{};
    if (!options.file.empty()) {
        if (!select_channels(options))
            return 2;
        Scanner scanner(options, 0, false, screen.get());
        FILE *file = std::fopen(options.file.c_str(), "rb");
        if (!file) {
            std::perror("Input file");
            return 1;
        }
        {
            std::ostringstream message;
            message << "Reading " << options.sample_rate << " S/s little-endian CS16 from "
                    << options.file << " at " << options.center_frequency << " Hz.";
            show_status(message.str());
        }
        while (!interrupted) {
            size_t count = std::fread(buffer.data(), 1, buffer.size(), file);
            if (count)
                scanner.feed_cs16_stream(buffer.data(), count);
            if (count < buffer.size()) {
                if (std::ferror(file)) {
                    std::perror("Input file");
                    input_failed = true;
                }
                break;
            }
        }
        if (std::fclose(file)) {
            std::perror("Input file close");
            input_failed = true;
        }
        if (scanner.trailing_bytes()) {
            std::cerr << "Input file ends with " << scanner.trailing_bytes()
                      << " incomplete CS16 byte(s).\n";
            input_failed = true;
        }
        totals = scanner_stats(scanner);
    } else if (options.udp) {
        if (!select_channels(options))
            return 2;
        Scanner scanner(options, 0, false, screen.get());
        int fd = open_udp_receiver(options);
        if (fd < 0)
            return 1;
        {
            std::ostringstream message;
            message << "Listening for " << options.sample_rate << " S/s little-endian CS16 on "
                    << options.bind << ':' << options.port << " at "
                    << options.center_frequency << " Hz.";
            show_status(message.str());
        }
        while (!interrupted) {
            ssize_t count = recv(fd, buffer.data(), buffer.size(), 0);
            if (count > 0)
                scanner.feed_cs16(buffer.data(), static_cast<size_t>(count));
            else if (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                std::perror("UDP receive");
                input_failed = true;
                break;
            }
        }
        close(fd);
        totals = scanner_stats(scanner);
    } else {
        if (!options.interactive)
            birria_native::print_rate_warning(options.sample_rate);
        std::vector<uint64_t> frequencies;
        if (options.sweep && options.sample_rate == 40000000u) {
            frequencies = {UINT64_C(2402000000), UINT64_C(2426000000),
                           UINT64_C(2480000000)};
        } else if (options.sweep) {
            frequencies = {UINT64_C(2441000000)};
        } else {
            frequencies = {options.center_frequency};
        }
        {
            std::ostringstream message;
            message << "Native USB device " << options.device_index
                    << " at " << options.sample_rate << " S/s";
            if (options.sweep && options.sample_rate == 40000000u)
                message << ", RF-sweeping all BLE advertising channels every "
                        << options.dwell_ms << " ms";
            else if (options.sweep)
                message << ", software-sweeping all BLE advertising channels every "
                        << options.dwell_ms << " ms";
            else
                message << ", center " << frequencies.front() << " Hz";
            if (options.sample_rate != 40000000u)
                message << " (experimental rate)";
            show_status(message.str());
        }
        NativeBleContext context(options, screen.get());
        birria_native::Options native_options;
        native_options.device_index = options.device_index;
        native_options.sample_rate = options.sample_rate;
        native_options.recovery_restarts = options.recovery_restarts;
        native_options.manual_gain = options.manual_gain;
        native_options.quiet = options.interactive;
        birria_native::Callbacks callbacks;
        callbacks.samples = NativeBleContext::samples;
        callbacks.gap = NativeBleContext::gap;
        callbacks.context = &context;
        birria_native::Session session(native_options, frequencies, options.dwell_ms,
                                       interrupted, callbacks);
        birria_native::Result result = session.run();
        totals = context.finish();
        bool decoder_dropped = context.dropped();
        if (!options.interactive) {
            std::cerr << "Native stream=" << birriasdr_strerror(result.stream_error)
                      << ", control=" << birriasdr_strerror(result.control_error)
                      << ", epochs=" << result.epochs
                      << ", firmware state=" << result.stats.firmware_state
                      << ", stage=" << result.stats.stage
                      << ", reset=" << birriasdr_strerror(result.stats.reset_error) << '\n';
        }
        input_failed = result.stream_error || result.control_error || result.close_error
            || decoder_dropped;
    }
    if (screen)
        screen.reset();
    std::cerr << "Input samples=" << totals.input_samples
              << ", valid BLE packets=" << totals.packets
              << ", CRC failures=" << totals.crc_failures
              << ", malformed datagrams=" << totals.bad_datagrams << '\n';
    return input_failed ? 1 : 0;
}
