/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wang Kang
 * Standalone legacy 802.11 OFDM decoder for BirriaSDR CS16 UDP streams.
 */
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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

namespace {

constexpr double PI = 3.14159265358979323846;
constexpr double WIFI_RATE = 20000000.0;
constexpr uint64_t DEFAULT_NATIVE_FREQUENCY = UINT64_C(2437000000);
constexpr size_t FFT_SIZE = 64;
constexpr size_t DETECT_DELAY = 16;
constexpr size_t DETECT_WINDOW = 64;
constexpr size_t DETECT_RUN = 96;
constexpr float DETECT_THRESHOLD = 0.57f;
constexpr float LTF_THRESHOLD = 0.43f;
constexpr size_t MAX_PSDU = 4095;

std::atomic<bool> interrupted{false};

struct Options {
    std::string bind = "0.0.0.0";
    std::string port = "1234";
    std::string file;
    std::string csi_file;
    std::string csi_format = "binary";
    uint64_t center_frequency = 0;
    uint32_t sample_rate = 40000000;
    uint32_t device_index = 0;
    uint32_t recovery_restarts = 3;
    uint32_t dwell_ms = 500;
    bool udp = false;
    bool manual_gain = false;
    bool sweep = false;
    bool frequency_set = false;
    bool dwell_set = false;
    bool compact = false;
    bool interactive = false;
    bool verbose = false;
    bool csi_invalid = false;
    bool csi_normalize = false;
    bool self_test = false;
};

struct Rate {
    std::array<uint8_t, 4> signal_bits;
    unsigned mbps;
    unsigned bits_per_subcarrier;
    unsigned coded_bits_per_symbol;
    unsigned data_bits_per_symbol;
    std::vector<uint8_t> puncture;
};

const std::array<Rate, 8> rates{{
    {{{1, 1, 0, 1}}, 6, 1, 48, 24, {1, 1}},
    {{{1, 1, 1, 1}}, 9, 1, 48, 36, {1, 1, 1, 0, 0, 1}},
    {{{0, 1, 0, 1}}, 12, 2, 96, 48, {1, 1}},
    {{{0, 1, 1, 1}}, 18, 2, 96, 72, {1, 1, 1, 0, 0, 1}},
    {{{1, 0, 0, 1}}, 24, 4, 192, 96, {1, 1}},
    {{{1, 0, 1, 1}}, 36, 4, 192, 144, {1, 1, 1, 0, 0, 1}},
    {{{0, 0, 0, 1}}, 48, 6, 288, 192, {1, 1, 1, 0}},
    {{{0, 0, 1, 1}}, 54, 6, 288, 216, {1, 1, 1, 0, 0, 1}},
}};

const std::array<int, 4> pilot_bins{{-21, -7, 7, 21}};
const std::array<float, 4> pilot_base{{1.0f, 1.0f, 1.0f, -1.0f}};

std::array<int8_t, 52> csi_subcarriers()
{
    std::array<int8_t, 52> subcarriers{};
    size_t output = 0;
    for (int subcarrier = -26; subcarrier <= 26; ++subcarrier)
        if (subcarrier != 0)
            subcarriers[output++] = static_cast<int8_t>(subcarrier);
    return subcarriers;
}

const std::array<int8_t, 52> csi_bins = csi_subcarriers();

inline float magnitude_squared(const std::complex<float> &value)
{
    return value.real() * value.real() + value.imag() * value.imag();
}

inline std::complex<float> delayed_product(const std::complex<float> &first,
                                           const std::complex<float> &second)
{
    return {first.real() * second.real() + first.imag() * second.imag(),
            first.real() * second.imag() - first.imag() * second.real()};
}

void usage(const char *program)
{
    std::cout
        << "Usage: " << program << " [options]\n"
        << "  --udp                   Receive UDP instead of native USB capture\n"
        << "  --bind ADDRESS          UDP local address (default 0.0.0.0)\n"
        << "  --port PORT             UDP local port (default 1234)\n"
        << "  --file PATH             Read little-endian CS16 instead of USB or UDP\n"
        << "  --frequency HZ          Native tuning frequency or input label\n"
        << "  --device INDEX          Native USB device index (default 0)\n"
        << "  --sample-rate RATE      40000000, 80000000 or 160000000\n"
        << "  --manual                Use LNA 2 below 4.5 GHz or LNA 5 above, filter 8, digital 0\n"
        << "  --recovery COUNT        Native restart budget (default 3)\n"
        << "  --sweep                 Sweep Wi-Fi channels 1 through 13, native only\n"
        << "  --dwell-ms MS           Sweep dwell per channel (default 500)\n"
        << "  --csi PATH              Write CSI records, or - for standard output\n"
        << "  --csi-format FORMAT     binary or jsonl (default binary)\n"
        << "  --csi-invalid           Include SIGNAL-valid frames with failed FCS\n"
        << "  --csi-normalize         Include an RMS-normalized CSI vector\n"
        << "  --compact               Print one compact line per valid frame\n"
        << "  --interactive           Show a deduplicated live terminal table\n"
        << "  --verbose               Print rejected PSDU diagnostics\n"
        << "  --self-test             Test coding, CSI, CRC and MAC parsing\n"
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

bool parse_options(int argc, char **argv, Options &options)
{
    for (int i = 1; i < argc; ++i) {
        std::string option = argv[i];
        auto value = [&]() -> const char * {
            return ++i < argc ? argv[i] : nullptr;
        };
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
        } else if (option == "--csi-invalid") {
            options.csi_invalid = true;
        } else if (option == "--csi-normalize") {
            options.csi_normalize = true;
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
        } else if (option == "--csi") {
            const char *arg = value();
            if (!arg)
                return false;
            options.csi_file = arg;
        } else if (option == "--csi-format") {
            const char *arg = value();
            if (!arg || (std::strcmp(arg, "binary") && std::strcmp(arg, "jsonl")))
                return false;
            options.csi_format = arg;
        } else if (option == "--frequency") {
            const char *arg = value();
            if (!arg || !parse_u64(arg, UINT64_MAX, false, options.center_frequency))
                return false;
            options.frequency_set = true;
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
    if (options.csi_file.empty()
        && (options.csi_invalid || options.csi_normalize || options.csi_format != "binary"))
        return false;
    if (!options.file.empty() && options.file == options.csi_file)
        return false;
    if (options.compact && options.interactive)
        return false;
    if (options.interactive && options.verbose)
        return false;
    if (options.interactive && options.csi_file == "-")
        return false;
    if (options.udp && !options.file.empty())
        return false;
    if (options.sweep && (options.udp || !options.file.empty() || options.frequency_set))
        return false;
    if (options.dwell_set && !options.sweep)
        return false;
    if ((options.manual_gain || options.device_index || options.recovery_restarts != 3)
        && (options.udp || !options.file.empty()))
        return false;
    return true;
}

void fft64(std::array<std::complex<float>, FFT_SIZE> &values, bool inverse = false)
{
    for (size_t i = 1, j = 0; i < FFT_SIZE; ++i) {
        size_t bit = FFT_SIZE >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(values[i], values[j]);
    }
    for (size_t length = 2; length <= FFT_SIZE; length <<= 1) {
        float angle = static_cast<float>((inverse ? 2.0 : -2.0) * PI / length);
        std::complex<float> step = std::polar(1.0f, angle);
        for (size_t base = 0; base < FFT_SIZE; base += length) {
            std::complex<float> factor(1.0f, 0.0f);
            for (size_t i = 0; i < length / 2; ++i) {
                std::complex<float> even = values[base + i];
                std::complex<float> odd = factor * values[base + i + length / 2];
                values[base + i] = even + odd;
                values[base + i + length / 2] = even - odd;
                factor *= step;
            }
        }
    }
    if (inverse)
        for (auto &value : values)
            value /= static_cast<float>(FFT_SIZE);
}

int bin_index(int subcarrier)
{
    return subcarrier < 0 ? subcarrier + static_cast<int>(FFT_SIZE) : subcarrier;
}

std::array<std::complex<float>, FFT_SIZE> make_ltf_frequency()
{
    static constexpr std::array<int, FFT_SIZE> shifted{{
        0, 0, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1,
        1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1,
        0, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1,
        1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 0, 0, 0, 0, 0
    }};
    std::array<std::complex<float>, FFT_SIZE> frequency{};
    for (int subcarrier = -32; subcarrier < 32; ++subcarrier)
        frequency[bin_index(subcarrier)] = static_cast<float>(shifted[subcarrier + 32]);
    return frequency;
}

const std::array<std::complex<float>, FFT_SIZE> ltf_frequency = make_ltf_frequency();

std::array<std::complex<float>, FFT_SIZE> make_ltf_time()
{
    auto time = ltf_frequency;
    fft64(time, true);
    return time;
}

const std::array<std::complex<float>, FFT_SIZE> ltf_time = make_ltf_time();

uint8_t parity(uint32_t value)
{
#if defined(__GNUC__)
    return static_cast<uint8_t>(__builtin_parity(value));
#else
    value ^= value >> 16;
    value ^= value >> 8;
    value ^= value >> 4;
    return static_cast<uint8_t>((0x6996 >> (value & 0xf)) & 1);
#endif
}

std::vector<float> deinterleave(const std::vector<float> &input,
                                unsigned coded_bits, unsigned bits_per_subcarrier)
{
    std::vector<float> output(coded_bits);
    unsigned s = std::max(bits_per_subcarrier / 2, 1u);
    for (unsigned k = 0; k < coded_bits; ++k) {
        unsigned i = (coded_bits / 16) * (k % 16) + k / 16;
        unsigned j = s * (i / s) + (i + coded_bits - (16 * i) / coded_bits) % s;
        output[k] = input[j];
    }
    return output;
}

std::vector<float> depuncture(const std::vector<float> &input, size_t input_bits,
                             const std::vector<uint8_t> &pattern)
{
    std::vector<float> output;
    output.reserve(input_bits * 2);
    size_t consumed = 0;
    for (size_t i = 0; output.size() < input_bits * 2; ++i) {
        if (pattern[i % pattern.size()]) {
            if (consumed >= input.size())
                return {};
            output.push_back(input[consumed++]);
        } else {
            output.push_back(0.0f);
        }
    }
    return output;
}

struct ViterbiResult {
    std::vector<uint8_t> bits;
    float metric = -std::numeric_limits<float>::infinity();
};

ViterbiResult viterbi(const std::vector<float> &soft, bool force_zero_state = true)
{
    constexpr float negative = -1.0e30f;
    if (soft.size() % 2)
        return {};
    size_t bit_count = soft.size() / 2;
    std::array<float, 64> paths{};
    paths.fill(negative);
    paths[0] = 0.0f;
    std::vector<std::array<uint8_t, 64>> previous(bit_count);
    std::vector<std::array<uint8_t, 64>> inputs(bit_count);
    for (size_t time = 0; time < bit_count; ++time) {
        std::array<float, 64> next{};
        next.fill(negative);
        for (unsigned state = 0; state < 64; ++state) {
            if (paths[state] <= negative / 2)
                continue;
            for (unsigned bit = 0; bit < 2; ++bit) {
                unsigned reg = (bit << 6) | state;
                unsigned next_state = reg >> 1;
                unsigned first = parity(reg & 0133u);
                unsigned second = parity(reg & 0171u);
                float metric = paths[state] + (first ? -soft[2 * time] : soft[2 * time])
                    + (second ? -soft[2 * time + 1] : soft[2 * time + 1]);
                if (metric > next[next_state]) {
                    next[next_state] = metric;
                    previous[time][next_state] = static_cast<uint8_t>(state);
                    inputs[time][next_state] = static_cast<uint8_t>(bit);
                }
            }
        }
        paths = next;
    }
    unsigned state = 0;
    if (!force_zero_state)
        state = static_cast<unsigned>(std::max_element(paths.begin(), paths.end()) - paths.begin());
    ViterbiResult result;
    result.metric = paths[state];
    result.bits.resize(bit_count);
    for (size_t time = bit_count; time-- > 0;) {
        result.bits[time] = inputs[time][state];
        state = previous[time][state];
    }
    return result;
}

uint32_t crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (crc & 1 ? UINT32_C(0xedb88320) : 0);
    }
    return crc ^ UINT32_MAX;
}

uint16_t get_le16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0] | static_cast<uint16_t>(data[1]) << 8);
}

uint32_t get_le32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) | static_cast<uint32_t>(data[1]) << 8
        | static_cast<uint32_t>(data[2]) << 16 | static_cast<uint32_t>(data[3]) << 24;
}

std::string mac_string(const uint8_t *address)
{
    std::ostringstream text;
    text << std::hex << std::setfill('0');
    for (unsigned i = 0; i < 6; ++i) {
        if (i)
            text << ':';
        text << std::setw(2) << static_cast<unsigned>(address[i]);
    }
    return text.str();
}

std::string escaped_text(const uint8_t *data, size_t length)
{
    std::ostringstream text;
    text << '"';
    for (size_t i = 0; i < length; ++i) {
        unsigned value = data[i];
        if (value == '\\' || value == '"')
            text << '\\' << static_cast<char>(value);
        else if (value >= 32 && value < 127)
            text << static_cast<char>(value);
        else
            text << "\\x" << std::hex << std::setw(2) << std::setfill('0') << value << std::dec;
    }
    text << '"';
    return text.str();
}

const char *type_name(unsigned type)
{
    static constexpr const char *names[] = {"management", "control", "data", "extension"};
    return names[type & 3];
}

const char *subtype_name(unsigned type, unsigned subtype)
{
    static constexpr const char *management[] = {
        "association-request", "association-response", "reassociation-request",
        "reassociation-response", "probe-request", "probe-response", "timing-advertisement",
        "reserved", "beacon", "atim", "disassociation", "authentication",
        "deauthentication", "action", "action-no-ack", "reserved"
    };
    static constexpr const char *control[] = {
        "reserved", "reserved", "trigger", "tack", "beamforming-report-poll",
        "vht-ndp-announcement", "control-frame-extension", "control-wrapper",
        "block-ack-request", "block-ack", "ps-poll", "rts", "cts", "ack",
        "cf-end", "cf-end-cf-ack"
    };
    if (type == 0)
        return management[subtype & 15];
    if (type == 1)
        return control[subtype & 15];
    return type == 2 ? "data" : "extension";
}

struct FrameInfo {
    unsigned type = 0;
    unsigned subtype = 0;
    std::string destination;
    std::string source;
    std::string bssid;
    std::string ssid;
    int channel = -1;
};

FrameInfo parse_frame(const std::vector<uint8_t> &frame)
{
    FrameInfo info;
    if (frame.size() < 10)
        return info;
    uint16_t control = get_le16(frame.data());
    info.type = (control >> 2) & 3;
    info.subtype = (control >> 4) & 15;
    info.destination = mac_string(frame.data() + 4);
    if (info.type == 0 && frame.size() >= 28) {
        info.source = mac_string(frame.data() + 10);
        info.bssid = mac_string(frame.data() + 16);
        if ((info.subtype == 8 || info.subtype == 5) && frame.size() >= 40) {
            size_t end = frame.size() - 4;
            for (size_t offset = 36; offset + 2 <= end;) {
                unsigned id = frame[offset];
                unsigned length = frame[offset + 1];
                offset += 2;
                if (offset + length > end)
                    break;
                if (id == 0)
                    info.ssid = length ? escaped_text(frame.data() + offset, length)
                                       : std::string();
                else if (id == 3 && length == 1)
                    info.channel = frame[offset];
                offset += length;
            }
        }
    } else if (info.type == 1 && frame.size() >= 20 &&
               (info.subtype == 8 || info.subtype == 9 || info.subtype == 11)) {
        info.source = mac_string(frame.data() + 10);
    } else if (info.type == 2 && frame.size() >= 28) {
        bool to_ds = control & (1u << 8);
        bool from_ds = control & (1u << 9);
        if (!to_ds && !from_ds) {
            info.destination = mac_string(frame.data() + 4);
            info.source = mac_string(frame.data() + 10);
            info.bssid = mac_string(frame.data() + 16);
        } else if (to_ds && !from_ds) {
            info.bssid = mac_string(frame.data() + 4);
            info.source = mac_string(frame.data() + 10);
            info.destination = mac_string(frame.data() + 16);
        } else if (!to_ds && from_ds) {
            info.destination = mac_string(frame.data() + 4);
            info.bssid = mac_string(frame.data() + 10);
            info.source = mac_string(frame.data() + 16);
        } else if (frame.size() >= 34) {
            info.destination = mac_string(frame.data() + 16);
            info.source = mac_string(frame.data() + 24);
        }
    }
    return info;
}

struct CsiRecord {
    uint64_t input_sample = 0;
    uint64_t center_frequency = 0;
    float cfo_hz = 0;
    float ltf_correlation = 0;
    float snr_db = 0;
    unsigned rate_mbps = 0;
    unsigned psdu_length = 0;
    bool fcs_valid = false;
    bool mac_valid = false;
    FrameInfo frame;
    std::array<std::complex<float>, 52> channel{};
};

void append_u16(std::vector<uint8_t> &output, uint16_t value)
{
    output.push_back(static_cast<uint8_t>(value));
    output.push_back(static_cast<uint8_t>(value >> 8));
}

void append_u32(std::vector<uint8_t> &output, uint32_t value)
{
    for (unsigned byte = 0; byte < 4; ++byte)
        output.push_back(static_cast<uint8_t>(value >> (8 * byte)));
}

void append_u64(std::vector<uint8_t> &output, uint64_t value)
{
    for (unsigned byte = 0; byte < 8; ++byte)
        output.push_back(static_cast<uint8_t>(value >> (8 * byte)));
}

void append_f32(std::vector<uint8_t> &output, float value)
{
    uint32_t word;
    static_assert(sizeof(word) == sizeof(value), "float32 is required");
    static_assert(std::numeric_limits<float>::is_iec559, "IEEE 754 float32 is required");
    std::memcpy(&word, &value, sizeof(word));
    append_u32(output, word);
}

std::array<uint8_t, 6> mac_bytes(const std::string &text)
{
    std::array<uint8_t, 6> output{};
    if (text.size() != 17)
        return output;
    for (size_t i = 0; i < output.size(); ++i) {
        char *end = nullptr;
        std::string part = text.substr(i * 3, 2);
        unsigned long value = std::strtoul(part.c_str(), &end, 16);
        if (!end || *end || value > 255)
            return {};
        output[i] = static_cast<uint8_t>(value);
    }
    return output;
}

std::array<std::complex<float>, 52> normalize_csi(
    const std::array<std::complex<float>, 52> &channel)
{
    double power = 0;
    for (const auto &value : channel)
        power += magnitude_squared(value);
    float rms = static_cast<float>(std::sqrt(power / channel.size()));
    std::array<std::complex<float>, 52> output{};
    if (rms > 1.0e-20f)
        for (size_t i = 0; i < output.size(); ++i)
            output[i] = channel[i] / rms;
    return output;
}

std::vector<uint8_t> encode_binary_csi(const CsiRecord &record, uint64_t sequence,
                                       bool include_normalized)
{
    constexpr uint16_t version = 1;
    constexpr uint16_t header_bytes = 96;
    constexpr uint32_t vector_bytes = 52 * 2 * sizeof(float);
    uint32_t record_bytes = header_bytes + vector_bytes
        + (include_normalized ? vector_bytes : 0);
    uint32_t flags = UINT32_C(1) << 2;
    if (record.fcs_valid)
        flags |= UINT32_C(1);
    if (include_normalized)
        flags |= UINT32_C(1) << 1;
    if (record.mac_valid)
        flags |= UINT32_C(1) << 3;
    std::vector<uint8_t> output;
    output.reserve(record_bytes);
    output.insert(output.end(), {'B', 'C', 'S', 'I'});
    append_u16(output, version);
    append_u16(output, header_bytes);
    append_u32(output, record_bytes);
    append_u32(output, flags);
    append_u64(output, sequence);
    append_u64(output, record.input_sample);
    append_u64(output, record.center_frequency);
    append_f32(output, record.cfo_hz);
    append_f32(output, record.ltf_correlation);
    append_f32(output, 0.0f);
    append_u16(output, static_cast<uint16_t>(record.rate_mbps));
    append_u16(output, static_cast<uint16_t>(record.psdu_length));
    output.push_back(static_cast<uint8_t>(record.frame.type));
    output.push_back(static_cast<uint8_t>(record.frame.subtype));
    output.push_back(record.fcs_valid ? 1 : 0);
    output.push_back(52);
    auto destination = mac_bytes(record.frame.destination);
    auto source = mac_bytes(record.frame.source);
    auto bssid = mac_bytes(record.frame.bssid);
    output.insert(output.end(), destination.begin(), destination.end());
    output.insert(output.end(), source.begin(), source.end());
    output.insert(output.end(), bssid.begin(), bssid.end());
    output.insert(output.end(), 18, 0);
    if (output.size() != header_bytes)
        return {};
    for (const auto &value : record.channel) {
        append_f32(output, value.real());
        append_f32(output, value.imag());
    }
    if (include_normalized) {
        auto normalized = normalize_csi(record.channel);
        for (const auto &value : normalized) {
            append_f32(output, value.real());
            append_f32(output, value.imag());
        }
    }
    return output.size() == record_bytes ? output : std::vector<uint8_t>{};
}

class CsiWriter {
public:
    ~CsiWriter()
    {
        if (file_ && file_ != stdout)
            std::fclose(file_);
    }

    bool open(const Options &options)
    {
        if (options.csi_file.empty())
            return true;
        json_ = options.csi_format == "jsonl";
        include_invalid_ = options.csi_invalid;
        normalize_ = options.csi_normalize;
        stdout_ = options.csi_file == "-";
        file_ = stdout_ ? stdout : std::fopen(options.csi_file.c_str(), "wb");
        if (!file_) {
            std::perror("CSI output");
            return false;
        }
        return true;
    }

    bool enabled() const { return file_ != nullptr; }
    bool include_invalid() const { return include_invalid_; }
    bool uses_stdout() const { return stdout_; }
    bool failed() const { return failed_; }
    uint64_t records() const { return sequence_; }

    bool write(const CsiRecord &record)
    {
        if (!file_)
            return true;
        bool success = json_ ? write_json(record) : write_binary(record);
        if (!success) {
            failed_ = true;
            if (!errno)
                errno = EIO;
            std::perror("CSI output");
        }
        return success;
    }

private:
    bool write_binary(const CsiRecord &record)
    {
        auto output = encode_binary_csi(record, sequence_++, normalize_);
        return !output.empty()
            && std::fwrite(output.data(), 1, output.size(), file_) == output.size()
            && std::fflush(file_) == 0;
    }

    static void json_string(std::ostringstream &output, const std::string &value)
    {
        output << '"';
        for (unsigned char character : value) {
            if (character == '\\' || character == '"')
                output << '\\' << static_cast<char>(character);
            else if (character >= 32 && character < 127)
                output << static_cast<char>(character);
            else
                output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                       << static_cast<unsigned>(character) << std::dec << std::setfill(' ');
        }
        output << '"';
    }

    bool write_json(const CsiRecord &record)
    {
        std::ostringstream output;
        output << std::setprecision(9)
               << "{\"version\":1,\"sequence\":" << sequence_++
               << ",\"input_sample\":" << record.input_sample
               << ",\"center_frequency_hz\":" << record.center_frequency
               << ",\"cfo_hz\":" << record.cfo_hz
               << ",\"ltf_correlation\":" << record.ltf_correlation
               << ",\"snr_db\":" << record.snr_db
               << ",\"rate_mbps\":" << record.rate_mbps
               << ",\"psdu_length\":" << record.psdu_length
               << ",\"fcs_valid\":" << (record.fcs_valid ? "true" : "false")
               << ",\"mac_valid\":" << (record.mac_valid ? "true" : "false")
               << ",\"type\":";
        if (record.mac_valid)
            output << record.frame.type << ",\"subtype\":" << record.frame.subtype;
        else
            output << "null,\"subtype\":null";
        auto named = [&](const char *name, const std::string &value) {
            output << ",\"" << name << "\":";
            if (value.empty())
                output << "null";
            else
                json_string(output, value);
        };
        named("destination", record.frame.destination);
        named("source", record.frame.source);
        named("bssid", record.frame.bssid);
        output << ",\"subcarriers\":[";
        for (size_t i = 0; i < csi_bins.size(); ++i) {
            if (i)
                output << ',';
            output << static_cast<int>(csi_bins[i]);
        }
        auto vector = [&](const char *name,
                          const std::array<std::complex<float>, 52> &channel) {
            output << "],\"" << name << "\":[";
            for (size_t i = 0; i < channel.size(); ++i) {
                if (i)
                    output << ',';
                output << '[' << channel[i].real() << ',' << channel[i].imag() << ']';
            }
        };
        vector("csi", record.channel);
        if (normalize_)
            vector("csi_normalized", normalize_csi(record.channel));
        output << "]}\n";
        std::string text = output.str();
        return std::fwrite(text.data(), 1, text.size(), file_) == text.size()
            && std::fflush(file_) == 0;
    }

    FILE *file_ = nullptr;
    bool json_ = false;
    bool include_invalid_ = false;
    bool normalize_ = false;
    bool stdout_ = false;
    bool failed_ = false;
    uint64_t sequence_ = 0;
};

std::array<std::complex<float>, FFT_SIZE> estimate_channel(
    const std::array<std::complex<float>, FFT_SIZE> &first,
    const std::array<std::complex<float>, FFT_SIZE> &second)
{
    std::array<std::complex<float>, FFT_SIZE> channel{};
    for (size_t bin = 0; bin < FFT_SIZE; ++bin)
        if (magnitude_squared(ltf_frequency[bin]) > 0)
            channel[bin] = (first[bin] + second[bin]) / (2.0f * ltf_frequency[bin]);
    return channel;
}

std::array<std::complex<float>, 52> active_csi(
    const std::array<std::complex<float>, FFT_SIZE> &channel)
{
    std::array<std::complex<float>, 52> output{};
    for (size_t i = 0; i < csi_bins.size(); ++i)
        output[i] = channel[bin_index(csi_bins[i])];
    return output;
}

struct DecoderStats {
    uint64_t input_samples = 0;
    uint64_t preamble_candidates = 0;
    uint64_t ltf_rejections = 0;
    uint64_t signal_rejections = 0;
    uint64_t signal_headers = 0;
    uint64_t data_rejections = 0;
    uint64_t frames = 0;
    uint64_t fcs_failures = 0;
    uint64_t bad_datagrams = 0;

    DecoderStats &operator+=(const DecoderStats &other)
    {
        input_samples += other.input_samples;
        preamble_candidates += other.preamble_candidates;
        ltf_rejections += other.ltf_rejections;
        signal_rejections += other.signal_rejections;
        signal_headers += other.signal_headers;
        data_rejections += other.data_rejections;
        frames += other.frames;
        fcs_failures += other.fcs_failures;
        bad_datagrams += other.bad_datagrams;
        return *this;
    }
};

class Decoder {
public:
    Decoder(uint64_t center_frequency, uint64_t input_center_frequency,
            uint32_t input_rate, uint64_t input_base, bool compact, bool verbose,
            CsiWriter *csi, birria_tui::Screen *screen)
        : center_frequency_(center_frequency), input_rate_(input_rate),
          input_decimation_(input_rate / static_cast<uint32_t>(WIFI_RATE)),
          input_base_(input_base), compact_(compact), verbose_(verbose), csi_(csi),
          screen_(screen)
    {
        double offset = static_cast<double>(center_frequency)
            - static_cast<double>(input_center_frequency);
        oscillator_step_ = std::polar(1.0f,
            static_cast<float>(-2.0 * PI * offset / input_rate));
        for (int subcarrier = -26; subcarrier <= 26; ++subcarrier)
            if (subcarrier != 0 && std::find(pilot_bins.begin(), pilot_bins.end(), subcarrier)
                    == pilot_bins.end())
                data_bins_.push_back(subcarrier);
    }

    void feed_cs16(const uint8_t *bytes, size_t length)
    {
        if (length % 4) {
            ++bad_datagrams_;
            return;
        }
        for (size_t offset = 0; offset < length; offset += 4)
            feed_scalar(bytes + offset);
        process();
    }

    void feed_iq(const int16_t *iq, uint32_t count)
    {
        for (uint32_t i = 0; i < count; ++i)
            feed_sample({static_cast<float>(iq[2 * i]), static_cast<float>(iq[2 * i + 1])});
        process();
    }

    void feed_cs16_stream(const uint8_t *bytes, size_t length)
    {
        size_t offset = 0;
        if (have_byte_) {
            while (byte_count_ < 4 && offset < length)
                byte_buffer_[byte_count_++] = bytes[offset++];
            if (byte_count_ == 4) {
                feed_scalar(byte_buffer_.data());
                have_byte_ = false;
                byte_count_ = 0;
            }
        }
        while (offset + 4 <= length) {
            feed_scalar(bytes + offset);
            offset += 4;
        }
        if (offset < length) {
            have_byte_ = true;
            while (offset < length)
                byte_buffer_[byte_count_++] = bytes[offset++];
        }
        process();
    }

    uint64_t frames() const { return frames_; }
    uint64_t bad_datagrams() const { return bad_datagrams_; }
    uint64_t signal_headers() const { return signal_headers_; }
    uint64_t fcs_failures() const { return fcs_failures_; }
    uint64_t preamble_candidates() const { return preamble_candidates_; }
    uint64_t ltf_rejections() const { return ltf_rejections_; }
    uint64_t signal_rejections() const { return signal_rejections_; }
    uint64_t data_rejections() const { return data_rejections_; }
    uint64_t input_samples() const { return input_pairs_; }
    size_t trailing_bytes() const { return byte_count_; }
    DecoderStats stats() const
    {
        return {input_pairs_, preamble_candidates_, ltf_rejections_, signal_rejections_,
                signal_headers_, data_rejections_, frames_, fcs_failures_, bad_datagrams_};
    }

private:
    enum class DecodeResult { NeedMore, Rejected, Consumed };

    void feed_scalar(const uint8_t *bytes)
    {
        int16_t i = static_cast<int16_t>(bytes[0] | static_cast<uint16_t>(bytes[1]) << 8);
        int16_t q = static_cast<int16_t>(bytes[2] | static_cast<uint16_t>(bytes[3]) << 8);
        feed_sample({static_cast<float>(i), static_cast<float>(q)});
    }

    void feed_sample(std::complex<float> sample)
    {
        sample *= oscillator_;
        oscillator_ *= oscillator_step_;
        if ((input_pairs_ & 4095u) == 4095u)
            oscillator_ /= std::abs(oscillator_);
        ++input_pairs_;
        pending_ += sample;
        if (++pending_count_ == input_decimation_) {
            samples_.push_back(pending_ / static_cast<float>(input_decimation_));
            pending_ = {};
            pending_count_ = 0;
        }
    }

    std::complex<float> corrected(size_t index, size_t reference, float omega) const
    {
        float phase = -omega * static_cast<float>(static_cast<int64_t>(index) - static_cast<int64_t>(reference));
        return samples_[index] * std::polar(1.0f, phase);
    }

    std::array<std::complex<float>, FFT_SIZE> symbol_fft(size_t start, size_t reference,
                                                        float omega) const
    {
        std::array<std::complex<float>, FFT_SIZE> values{};
        for (size_t i = 0; i < FFT_SIZE; ++i)
            values[i] = corrected(start + i, reference, omega);
        fft64(values);
        return values;
    }

    float ltf_correlation(size_t start, size_t reference, float omega) const
    {
        std::complex<float> correlation{};
        double energy = 0;
        double pattern_energy = 0;
        for (size_t i = 0; i < FFT_SIZE; ++i) {
            std::complex<float> sample = corrected(start + i, reference, omega);
            correlation += std::conj(ltf_time[i]) * sample;
            energy += magnitude_squared(sample);
            pattern_energy += magnitude_squared(ltf_time[i]);
        }
        return static_cast<float>(std::abs(correlation) /
            (std::sqrt(energy * pattern_energy) + 1.0e-12));
    }

    struct SignalMetrics {
        float rssi_dbfs = 0;
        float snr_db = 0;
    };

    SignalMetrics measure_signal(
        size_t ltf_start,
        const std::array<std::complex<float>, FFT_SIZE> &first,
        const std::array<std::complex<float>, FFT_SIZE> &second) const
    {
        SignalMetrics metrics;
        double time_power = 0;
        for (size_t i = 0; i < FFT_SIZE * 2; ++i)
            time_power += magnitude_squared(samples_[ltf_start + i]);
        double mean_power = time_power / (FFT_SIZE * 2);
        metrics.rssi_dbfs = static_cast<float>(10.0 * std::log10(
            std::max(mean_power, 1.0e-30) / (32768.0 * 32768.0)));

        double mean_frequency_power = 0;
        double noise_power = 0;
        for (int subcarrier = -26; subcarrier <= 26; ++subcarrier) {
            if (subcarrier == 0)
                continue;
            int bin = bin_index(subcarrier);
            std::complex<float> mean = 0.5f * (first[bin] + second[bin]);
            std::complex<float> difference = first[bin] - second[bin];
            mean_frequency_power += magnitude_squared(mean);
            noise_power += 0.5 * magnitude_squared(difference);
        }
        double signal_power = std::max(mean_frequency_power - 0.5 * noise_power,
                                       1.0e-30);
        metrics.snr_db = static_cast<float>(10.0 * std::log10(
            signal_power / std::max(noise_power, 1.0e-30)));
        return metrics;
    }

    float pilot_phase(const std::array<std::complex<float>, FFT_SIZE> &symbol,
                      const std::array<std::complex<float>, FFT_SIZE> &channel) const
    {
        std::complex<float> sum{};
        for (size_t i = 0; i < pilot_bins.size(); ++i) {
            int bin = bin_index(pilot_bins[i]);
            if (magnitude_squared(channel[bin]) > 1.0e-12f)
                sum += symbol[bin] / channel[bin] * pilot_base[i];
        }
        return std::arg(sum);
    }

    static float unwrap_mod_pi(float phase, float previous)
    {
        float turns = std::round((previous - phase) / static_cast<float>(PI));
        float best = phase + turns * static_cast<float>(PI);
        float alternate = best + (best < previous ? static_cast<float>(PI) : -static_cast<float>(PI));
        return std::abs(alternate - previous) < std::abs(best - previous) ? alternate : best;
    }

    std::vector<float> demap(const std::array<std::complex<float>, FFT_SIZE> &symbol,
                             const std::array<std::complex<float>, FFT_SIZE> &channel,
                             float phase, unsigned bits_per_subcarrier) const
    {
        std::vector<std::complex<float>> values;
        values.reserve(48);
        std::complex<float> rotation = std::polar(1.0f, -phase);
        float power = 0;
        for (int subcarrier : data_bins_) {
            int bin = bin_index(subcarrier);
            float channel_power = magnitude_squared(channel[bin]);
            std::complex<float> value{};
            if (channel_power > 1.0e-12f)
                value = symbol[bin] * std::conj(channel[bin]) / channel_power * rotation;
            values.push_back(value);
            power += magnitude_squared(value);
        }
        float scale = std::sqrt(power / values.size());
        if (scale < 1.0e-12f)
            scale = 1.0f;
        std::vector<float> output;
        output.reserve(48 * bits_per_subcarrier);
        for (std::complex<float> value : values) {
            value /= scale;
            if (bits_per_subcarrier == 1) {
                output.push_back(value.real());
            } else if (bits_per_subcarrier == 2) {
                output.push_back(value.real());
                output.push_back(value.imag());
            } else if (bits_per_subcarrier == 4) {
                output.push_back(value.real());
                output.push_back(value.imag());
                output.push_back(std::abs(value.real()) - 0.6324555f);
                output.push_back(std::abs(value.imag()) - 0.6324555f);
            } else {
                float real_magnitude = std::abs(value.real());
                float imag_magnitude = std::abs(value.imag());
                output.push_back(value.real());
                output.push_back(value.imag());
                output.push_back(real_magnitude - 0.6172134f);
                output.push_back(imag_magnitude - 0.6172134f);
                output.push_back(std::abs(real_magnitude - 0.6172134f) - 0.3086067f);
                output.push_back(std::abs(imag_magnitude - 0.6172134f) - 0.3086067f);
            }
        }
        return output;
    }

    const Rate *decode_signal(size_t signal_start,
                              const std::array<std::complex<float>, FFT_SIZE> &channel,
                              size_t reference, float omega, size_t &length,
                              float &phase)
    {
        auto symbol = symbol_fft(signal_start, reference, omega);
        float raw_phase = pilot_phase(symbol, channel);
        const Rate *selected = nullptr;
        float selected_metric = -std::numeric_limits<float>::infinity();
        for (unsigned inversion = 0; inversion < 2; ++inversion) {
            float candidate_phase = raw_phase + inversion * static_cast<float>(PI);
            auto soft = deinterleave(demap(symbol, channel, candidate_phase, 1), 48, 1);
            ViterbiResult decoded = viterbi(soft);
            if (decoded.bits.size() != 24)
                continue;
            const Rate *rate = nullptr;
            for (const Rate &item : rates)
                if (std::equal(item.signal_bits.begin(), item.signal_bits.end(), decoded.bits.begin()))
                    rate = &item;
            unsigned parity_sum = 0;
            for (unsigned i = 0; i < 18; ++i)
                parity_sum ^= decoded.bits[i];
            bool tail_zero = std::none_of(decoded.bits.begin() + 18, decoded.bits.end(),
                                          [](uint8_t bit) { return bit != 0; });
            size_t candidate_length = 0;
            for (unsigned i = 0; i < 12; ++i)
                candidate_length |= static_cast<size_t>(decoded.bits[5 + i]) << i;
            if (rate && parity_sum == 0 && tail_zero && candidate_length >= 4
                && candidate_length <= MAX_PSDU && decoded.metric > selected_metric) {
                selected = rate;
                selected_metric = decoded.metric;
                length = candidate_length;
                phase = candidate_phase;
            }
        }
        return selected;
    }

    std::vector<uint8_t> descramble(const std::vector<uint8_t> &bits) const
    {
        for (unsigned seed = 1; seed < 128; ++seed) {
            unsigned state = seed;
            std::vector<uint8_t> output(bits.size());
            for (size_t i = 0; i < bits.size(); ++i) {
                unsigned feedback = ((state >> 6) ^ (state >> 3)) & 1;
                output[i] = static_cast<uint8_t>(bits[i] ^ feedback);
                state = ((state << 1) & 0x7e) | feedback;
            }
            if (output.size() >= 7 && std::none_of(output.begin(), output.begin() + 7,
                                                   [](uint8_t bit) { return bit != 0; }))
                return output;
        }
        return {};
    }

    void print_frame(const std::vector<uint8_t> &frame, const Rate &rate,
                     size_t sample_index, float omega, float ltf_score,
                     const SignalMetrics &metrics)
    {
        FrameInfo info = parse_frame(frame);
        uint64_t input_sample = input_base_ + sample_index * input_decimation_;
        double time = static_cast<double>(input_sample) / input_rate_;
        double cfo = omega * WIFI_RATE / (2.0 * PI);
        std::ostream &output = csi_ && csi_->uses_stdout() ? std::cerr : std::cout;
        if (compact_ || screen_) {
            std::ostringstream line;
            line << std::fixed << std::setprecision(6)
                   << "wifi time=" << time << "s sample=" << input_sample;
            if (center_frequency_)
                line << " center=" << center_frequency_ << "Hz";
            line << " rate=" << rate.mbps << "Mbps"
                   << " pdu=" << subtype_name(info.type, info.subtype)
                   << " len=" << frame.size();
            if (!info.source.empty())
                line << " src=" << info.source;
            if (!info.destination.empty())
                line << " dst=" << info.destination;
            if (!info.bssid.empty() && info.bssid != info.source)
                line << " bssid=" << info.bssid;
            if (!info.ssid.empty())
                line << " ssid=" << info.ssid;
            if (info.channel >= 0)
                line << " channel=" << info.channel;
            line << " rssi=" << std::setprecision(1) << metrics.rssi_dbfs << "dBFS"
                   << " snr=" << metrics.snr_db << "dB fcs=ok";
            if (screen_) {
                std::string key;
                if (info.type == 0 && (info.subtype == 5 || info.subtype == 8)
                    && !info.bssid.empty())
                    key = "ap|" + info.bssid;
                else
                    key = std::string(subtype_name(info.type, info.subtype)) + "|"
                        + info.source + "|" + info.destination + "|" + info.bssid;
                std::ostringstream center;
                center << std::fixed << std::setprecision(3)
                       << static_cast<double>(center_frequency_) / 1000000.0;
                std::ostringstream rssi;
                rssi << std::fixed << std::setprecision(1) << metrics.rssi_dbfs;
                std::ostringstream snr;
                snr << std::fixed << std::setprecision(1) << metrics.snr_db;
                std::string address = !info.bssid.empty() ? info.bssid
                    : !info.source.empty() ? info.source : info.destination;
                std::ostringstream details;
                if (!info.ssid.empty())
                    details << "ssid=" << info.ssid << ' ';
                if (info.channel >= 0)
                    details << "ch=" << info.channel << ' ';
                details << "len=" << frame.size() << " sample=" << input_sample;
                if (!info.source.empty() && info.source != address)
                    details << " src=" << info.source;
                if (!info.destination.empty() && info.destination != address)
                    details << " dst=" << info.destination;
                details << " cfo=" << std::fixed << std::setprecision(1) << cfo
                        << "Hz ltf=" << std::setprecision(3) << ltf_score << " fcs=ok";
                screen_->update(key, {center.str(), std::to_string(rate.mbps),
                    subtype_name(info.type, info.subtype), rssi.str(), snr.str(),
                    address, details.str()});
            } else {
                output << line.str() << '\n';
                output.flush();
            }
            return;
        }
        output << std::fixed << std::setprecision(6)
               << "frame time=" << time << "s sample=" << input_sample;
        if (center_frequency_)
            output << " center=" << center_frequency_ << "Hz";
        output << " rate=" << rate.mbps << "Mbps length=" << frame.size()
               << " type=" << type_name(info.type)
               << " subtype=" << subtype_name(info.type, info.subtype)
               << " dst=" << info.destination;
        if (!info.source.empty())
            output << " src=" << info.source;
        if (!info.bssid.empty())
            output << " bssid=" << info.bssid;
        output << " cfo=" << std::setprecision(1) << cfo << "Hz"
               << " rssi=" << metrics.rssi_dbfs << "dBFS"
               << " snr=" << metrics.snr_db << "dB"
               << " ltf=" << std::setprecision(3) << ltf_score
               << " fcs=ok\n";
        if (info.type == 0 && (info.subtype == 8 || info.subtype == 5)) {
            output << (info.subtype == 8 ? "beacon" : "probe-response")
                   << " bssid=" << info.bssid << " ssid="
                   << (info.ssid.empty() ? "<hidden>" : info.ssid);
            if (info.channel >= 0)
                output << " channel=" << info.channel;
            if (frame.size() >= 36) {
                double interval_ms = get_le16(frame.data() + 32) * 1.024;
                output << " interval=" << std::fixed << std::setprecision(3)
                       << interval_ms << "ms capability=0x" << std::hex
                       << std::setw(4) << std::setfill('0') << get_le16(frame.data() + 34)
                       << std::dec << std::setfill(' ');
            }
            output << '\n';
        }
        output.flush();
    }

    DecodeResult decode_candidate(size_t run_start, std::complex<float> correlation)
    {
        if (samples_.size() < run_start + 420)
            return DecodeResult::NeedMore;
        float coarse_omega = std::arg(correlation) / static_cast<float>(DETECT_DELAY);
        size_t search_begin = run_start + 60;
        size_t search_end = std::min(samples_.size() - 128, run_start + 300);
        struct CoarseCandidate { float score = 0; size_t start = 0; };
        std::array<CoarseCandidate, 2> coarse{};
        for (size_t start = search_begin; start <= search_end; start += 2) {
            CoarseCandidate candidate{ltf_correlation(start, run_start, coarse_omega), start};
            auto position = std::min_element(coarse.begin(), coarse.end(),
                [](const CoarseCandidate &a, const CoarseCandidate &b) {
                    return a.score < b.score;
                });
            if (candidate.score > position->score)
                *position = candidate;
        }
        size_t ltf_start = search_begin;
        float best_score = 0;
        for (const CoarseCandidate &candidate : coarse) {
            size_t refine_begin = candidate.start > 3 ? candidate.start - 3 : search_begin;
            size_t refine_end = std::min(search_end, candidate.start + 3);
            for (size_t start = std::max(search_begin, refine_begin);
                 start <= refine_end; ++start) {
                float first = ltf_correlation(start, run_start, coarse_omega);
                float second = ltf_correlation(start + 64, run_start, coarse_omega);
                float score = std::min(first, second);
                if (score > best_score) {
                    best_score = score;
                    ltf_start = start;
                }
            }
        }
        if (best_score < LTF_THRESHOLD) {
            ++ltf_rejections_;
            return DecodeResult::Rejected;
        }
        std::complex<float> repeated{};
        for (size_t i = 0; i < 64; ++i)
            repeated += std::conj(corrected(ltf_start + i, ltf_start, coarse_omega))
                * corrected(ltf_start + 64 + i, ltf_start, coarse_omega);
        float omega = coarse_omega + std::arg(repeated) / 64.0f;
        auto first = symbol_fft(ltf_start, ltf_start, omega);
        auto second = symbol_fft(ltf_start + 64, ltf_start, omega);
        auto channel = estimate_channel(first, second);
        SignalMetrics signal_metrics = measure_signal(ltf_start, first, second);
        size_t length = 0;
        float phase = 0;
        const Rate *rate = decode_signal(ltf_start + 144, channel, ltf_start, omega, length, phase);
        if (!rate) {
            ++signal_rejections_;
            return DecodeResult::Rejected;
        }
        size_t bit_count = 16 + 8 * length + 6;
        size_t symbols = (bit_count + rate->data_bits_per_symbol - 1) / rate->data_bits_per_symbol;
        size_t frame_end = ltf_start + 224 + 80 * symbols;
        if (samples_.size() < frame_end)
            return DecodeResult::NeedMore;
        ++signal_headers_;
        std::vector<float> coded;
        coded.reserve(symbols * rate->coded_bits_per_symbol);
        float previous_phase = phase;
        for (size_t symbol_index = 0; symbol_index < symbols; ++symbol_index) {
            auto symbol = symbol_fft(ltf_start + 224 + 80 * symbol_index,
                                     ltf_start, omega);
            float current_phase = unwrap_mod_pi(pilot_phase(symbol, channel), previous_phase);
            previous_phase = current_phase;
            auto soft = demap(symbol, channel, current_phase, rate->bits_per_subcarrier);
            auto ordered = deinterleave(soft, rate->coded_bits_per_symbol,
                                        rate->bits_per_subcarrier);
            coded.insert(coded.end(), ordered.begin(), ordered.end());
        }
        size_t decoded_bits = symbols * rate->data_bits_per_symbol;
        auto full_rate = depuncture(coded, decoded_bits, rate->puncture);
        /* Scrambled pad bits follow the zero tail and can leave a nonzero final state. */
        ViterbiResult decoded = viterbi(full_rate, false);
        auto plain = descramble(decoded.bits);
        if (plain.size() < 16 + 8 * length) {
            ++data_rejections_;
            scan_cursor_ = frame_end;
            return DecodeResult::Consumed;
        }
        std::vector<uint8_t> frame(length);
        for (size_t byte = 0; byte < length; ++byte)
            for (unsigned bit = 0; bit < 8; ++bit)
                frame[byte] |= static_cast<uint8_t>(plain[16 + 8 * byte + bit] << bit);
        uint32_t expected_fcs = get_le32(frame.data() + frame.size() - 4);
        uint32_t actual_fcs = crc32(frame.data(), frame.size() - 4);
        bool fcs_valid = expected_fcs == actual_fcs;
        FrameInfo frame_info;
        if (fcs_valid)
            frame_info = parse_frame(frame);
        if (csi_ && csi_->enabled() && (fcs_valid || csi_->include_invalid())) {
            CsiRecord record;
            record.input_sample = input_base_
                + (stream_base_ + ltf_start) * input_decimation_;
            record.center_frequency = center_frequency_;
            record.cfo_hz = omega * static_cast<float>(WIFI_RATE / (2.0 * PI));
            record.ltf_correlation = best_score;
            record.snr_db = signal_metrics.snr_db;
            record.rate_mbps = rate->mbps;
            record.psdu_length = static_cast<unsigned>(length);
            record.fcs_valid = fcs_valid;
            record.mac_valid = fcs_valid;
            record.frame = frame_info;
            record.channel = active_csi(channel);
            if (!csi_->write(record))
                interrupted = true;
        }
        if (fcs_valid) {
            ++frames_;
            print_frame(frame, *rate, stream_base_ + ltf_start, omega, best_score,
                        signal_metrics);
        } else {
            ++fcs_failures_;
            if (verbose_)
                std::cerr << "Rejected PSDU at input sample "
                          << input_base_ + (stream_base_ + ltf_start) * input_decimation_
                          << ": FCS mismatch, rate=" << rate->mbps
                          << " Mbps, length=" << length << '\n';
        }
        scan_cursor_ = frame_end;
        return DecodeResult::Consumed;
    }

    void initialize_rolling()
    {
        rolling_correlation_ = {};
        rolling_energy_ = 0;
        for (size_t i = 0; i < DETECT_WINDOW; ++i) {
            rolling_correlation_ += delayed_product(samples_[scan_cursor_ + i],
                samples_[scan_cursor_ + i + DETECT_DELAY]);
            rolling_energy_ += magnitude_squared(samples_[scan_cursor_ + i + DETECT_DELAY]);
        }
        rolling_valid_ = true;
    }

    void process()
    {
        for (;;) {
            if (pending_candidate_) {
                DecodeResult result = decode_candidate(candidate_start_, candidate_correlation_);
                if (result == DecodeResult::NeedMore)
                    break;
                pending_candidate_ = false;
                detect_count_ = 0;
                rolling_valid_ = false;
                if (result == DecodeResult::Rejected)
                    scan_cursor_ = candidate_start_ + 160;
            }
            if (scan_cursor_ + DETECT_WINDOW + DETECT_DELAY >= samples_.size())
                break;
            if (!rolling_valid_)
                initialize_rolling();
            float denominator = static_cast<float>(rolling_energy_ * rolling_energy_ + 1.0e-20);
            float metric = magnitude_squared(rolling_correlation_) / denominator;
            if (metric > DETECT_THRESHOLD && rolling_energy_ > 1.0f) {
                if (detect_count_++ == 0) {
                    candidate_start_ = scan_cursor_;
                    candidate_correlation_ = rolling_correlation_;
                }
                if (detect_count_ == DETECT_RUN) {
                    ++preamble_candidates_;
                    pending_candidate_ = true;
                    continue;
                }
            } else {
                detect_count_ = 0;
            }
            size_t old = scan_cursor_;
            rolling_correlation_ -= delayed_product(samples_[old], samples_[old + DETECT_DELAY]);
            rolling_correlation_ += delayed_product(samples_[old + DETECT_WINDOW],
                samples_[old + DETECT_WINDOW + DETECT_DELAY]);
            rolling_energy_ -= magnitude_squared(samples_[old + DETECT_DELAY]);
            rolling_energy_ += magnitude_squared(samples_[old + DETECT_WINDOW + DETECT_DELAY]);
            ++scan_cursor_;
        }
        if (scan_cursor_ > 262144 && !pending_candidate_) {
            size_t remove = scan_cursor_ - 512;
            samples_.erase(samples_.begin(), samples_.begin() + static_cast<std::ptrdiff_t>(remove));
            stream_base_ += remove;
            scan_cursor_ -= remove;
            candidate_start_ = candidate_start_ > remove ? candidate_start_ - remove : 0;
            rolling_valid_ = false;
        }
    }

    uint64_t center_frequency_;
    uint32_t input_rate_;
    size_t input_decimation_;
    uint64_t input_base_;
    std::complex<float> oscillator_{1.0f, 0.0f};
    std::complex<float> oscillator_step_{1.0f, 0.0f};
    bool compact_;
    bool verbose_;
    CsiWriter *csi_;
    birria_tui::Screen *screen_;
    std::vector<int> data_bins_;
    std::vector<std::complex<float>> samples_;
    std::complex<float> pending_{};
    size_t pending_count_ = 0;
    std::array<uint8_t, 4> byte_buffer_{};
    size_t byte_count_ = 0;
    bool have_byte_ = false;
    size_t scan_cursor_ = 0;
    uint64_t stream_base_ = 0;
    bool rolling_valid_ = false;
    std::complex<float> rolling_correlation_{};
    double rolling_energy_ = 0;
    size_t detect_count_ = 0;
    bool pending_candidate_ = false;
    size_t candidate_start_ = 0;
    std::complex<float> candidate_correlation_{};
    uint64_t input_pairs_ = 0;
    uint64_t frames_ = 0;
    uint64_t bad_datagrams_ = 0;
    uint64_t signal_headers_ = 0;
    uint64_t preamble_candidates_ = 0;
    uint64_t ltf_rejections_ = 0;
    uint64_t signal_rejections_ = 0;
    uint64_t data_rejections_ = 0;
    uint64_t fcs_failures_ = 0;
};

class NativeWifiContext {
public:
    NativeWifiContext(const Options &options, CsiWriter *csi,
                      birria_tui::Screen *screen)
        : options_(options), csi_(csi), screen_(screen), settle_samples_(options.sample_rate / 10),
          software_dwell_samples_(static_cast<uint64_t>(options.sample_rate)
                                  * options.dwell_ms / 1000),
          worker_(&NativeWifiContext::worker_loop, this)
    {
    }

    static void samples(const int16_t *iq, uint32_t count, uint64_t first_sample,
                        uint64_t frequency, uint64_t epoch, void *context)
    {
        static_cast<NativeWifiContext *>(context)->enqueue(
            iq, count, first_sample, frequency, epoch);
    }

    static void gap(const birriasdr_recovery_event *event, uint64_t frequency,
                    uint64_t epoch, void *context)
    {
        NativeWifiContext *native = static_cast<NativeWifiContext *>(context);
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

    DecoderStats finish()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        condition_.notify_all();
        if (worker_.joinable())
            worker_.join();
        if (decoder_)
            totals_ += decoder_->stats();
        decoder_.reset();
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

    std::unique_ptr<Decoder> make_decoder(uint64_t input_frequency,
                                          uint64_t input_base)
    {
        uint64_t decode_frequency = input_frequency;
        if (options_.sweep && options_.sample_rate >= 80000000u)
            decode_frequency = UINT64_C(2412000000)
                + UINT64_C(5000000) * (software_channel_ - 1);
        return std::make_unique<Decoder>(decode_frequency, input_frequency,
            options_.sample_rate, input_base, options_.compact, options_.verbose,
            csi_, screen_);
    }

    void feed(const int16_t *iq, uint32_t count, uint64_t first_sample,
              uint64_t frequency, uint64_t epoch)
    {
        if (!decoder_) {
            decoder_ = make_decoder(frequency, first_sample);
            epoch_ = epoch;
        } else if (epoch != epoch_) {
            totals_ += decoder_->stats();
            decoder_.reset();
            epoch_ = epoch;
            pending_frequency_ = frequency;
            discard_remaining_ = settle_samples_;
        }
        uint32_t skip = static_cast<uint32_t>(std::min<uint64_t>(count, discard_remaining_));
        discard_remaining_ -= skip;
        iq += static_cast<size_t>(skip) * 2;
        count -= skip;
        first_sample += skip;
        if (!count)
            return;
        if (!decoder_)
            decoder_ = make_decoder(pending_frequency_, first_sample);
        if (!options_.sweep || options_.sample_rate < 80000000u) {
            decoder_->feed_iq(iq, count);
            return;
        }
        uint32_t done = 0;
        while (done < count) {
            uint64_t remaining = software_dwell_samples_ - software_samples_;
            uint32_t take = static_cast<uint32_t>(std::min<uint64_t>(count - done, remaining));
            decoder_->feed_iq(iq + static_cast<size_t>(done) * 2, take);
            done += take;
            software_samples_ += take;
            if (software_samples_ == software_dwell_samples_) {
                totals_ += decoder_->stats();
                decoder_.reset();
                software_samples_ = 0;
                software_channel_ = software_channel_ == 13 ? 1 : software_channel_ + 1;
                if (done < count)
                    decoder_ = make_decoder(frequency, first_sample + done);
            }
        }
    }

    const Options &options_;
    CsiWriter *csi_;
    birria_tui::Screen *screen_;
    uint64_t settle_samples_;
    uint64_t software_dwell_samples_;
    uint64_t software_samples_ = 0;
    unsigned software_channel_ = 1;
    uint64_t discard_remaining_ = 0;
    uint64_t pending_frequency_ = 0;
    uint64_t epoch_ = std::numeric_limits<uint64_t>::max();
    std::unique_ptr<Decoder> decoder_;
    DecoderStats totals_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<Chunk> queue_;
    bool stopping_ = false;
    uint64_t dropped_samples_ = 0;
    std::thread worker_;
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

bool coding_self_test()
{
    std::vector<uint8_t> source(240);
    for (size_t i = 0; i < source.size(); ++i)
        source[i] = static_cast<uint8_t>((i * 29 + i / 7) & 1);
    source[source.size() - 6] = 0;
    source[source.size() - 5] = 0;
    source[source.size() - 4] = 0;
    source[source.size() - 3] = 0;
    source[source.size() - 2] = 0;
    source[source.size() - 1] = 0;
    std::vector<float> encoded;
    unsigned state = 0;
    for (uint8_t bit : source) {
        unsigned reg = (static_cast<unsigned>(bit) << 6) | state;
        encoded.push_back(parity(reg & 0133u) ? -1.0f : 1.0f);
        encoded.push_back(parity(reg & 0171u) ? -1.0f : 1.0f);
        state = reg >> 1;
    }
    auto decoded = viterbi(encoded);
    if (decoded.bits != source) {
        std::cerr << "self-test: Viterbi mismatch\n";
        return false;
    }
    std::vector<uint8_t> padded_source(240);
    for (size_t i = 0; i < padded_source.size(); ++i)
        padded_source[i] = static_cast<uint8_t>((i * 17 + i / 5) & 1);
    for (size_t i = 200; i < 206; ++i)
        padded_source[i] = 0;
    encoded.clear();
    state = 0;
    for (uint8_t bit : padded_source) {
        unsigned reg = (static_cast<unsigned>(bit) << 6) | state;
        encoded.push_back(parity(reg & 0133u) ? -1.0f : 1.0f);
        encoded.push_back(parity(reg & 0171u) ? -1.0f : 1.0f);
        state = reg >> 1;
    }
    decoded = viterbi(encoded, false);
    if (decoded.bits != padded_source) {
        std::cerr << "self-test: Viterbi best-final-state mismatch\n";
        return false;
    }
    static const uint8_t check[] = "123456789";
    if (crc32(check, sizeof(check) - 1) != UINT32_C(0xcbf43926)) {
        std::cerr << "self-test: CRC mismatch\n";
        return false;
    }
    std::vector<uint8_t> ack(14);
    ack[0] = 0xd4;
    const uint8_t address[] = {0xdd, 0x0e, 0xe0, 0x3e, 0x44, 0x08};
    std::copy(address, address + 6, ack.begin() + 4);
    uint32_t fcs = crc32(ack.data(), 10);
    for (unsigned i = 0; i < 4; ++i)
        ack[10 + i] = static_cast<uint8_t>(fcs >> (8 * i));
    FrameInfo info = parse_frame(ack);
    bool passed = info.type == 1 && info.subtype == 13
        && info.destination == "dd:0e:e0:3e:44:08";
    if (!passed)
        std::cerr << "self-test: MAC parser mismatch\n";
    return passed;
}

bool csi_self_test()
{
    std::array<std::complex<float>, FFT_SIZE> expected{};
    std::array<std::complex<float>, FFT_SIZE> transmitted{};
    for (int subcarrier = -26; subcarrier <= 26; ++subcarrier) {
        if (subcarrier == 0)
            continue;
        int bin = bin_index(subcarrier);
        float amplitude = 0.75f + 0.01f * static_cast<float>(subcarrier + 26);
        float phase = 0.025f * static_cast<float>(subcarrier);
        expected[bin] = std::polar(amplitude, phase);
        transmitted[bin] = ltf_frequency[bin] * expected[bin];
    }
    auto time = transmitted;
    fft64(time, true);
    constexpr float omega = 0.0075f;
    std::array<std::complex<float>, FFT_SIZE> first{};
    std::array<std::complex<float>, FFT_SIZE> second{};
    for (size_t i = 0; i < FFT_SIZE; ++i) {
        std::complex<float> first_received = time[i] * std::polar(1.0f, omega * i);
        std::complex<float> second_received = time[i]
            * std::polar(1.0f, omega * static_cast<float>(i + FFT_SIZE));
        first[i] = first_received * std::polar(1.0f, -omega * i);
        second[i] = second_received
            * std::polar(1.0f, -omega * static_cast<float>(i + FFT_SIZE));
    }
    fft64(first);
    fft64(second);
    auto estimated = estimate_channel(first, second);
    for (int subcarrier = -26; subcarrier <= 26; ++subcarrier) {
        if (subcarrier == 0)
            continue;
        int bin = bin_index(subcarrier);
        if (std::abs(estimated[bin] - expected[bin]) > 2.0e-4f) {
            std::cerr << "self-test: CSI estimate mismatch at subcarrier "
                      << subcarrier << '\n';
            return false;
        }
    }
    auto normalized = normalize_csi(active_csi(estimated));
    double power = 0;
    for (const auto &value : normalized)
        power += magnitude_squared(value);
    if (std::abs(power / normalized.size() - 1.0) > 1.0e-5) {
        std::cerr << "self-test: CSI normalization mismatch\n";
        return false;
    }
    CsiRecord record;
    record.input_sample = UINT64_C(0x0102030405060708);
    record.center_frequency = UINT64_C(2437000000);
    record.cfo_hz = -1234.5f;
    record.ltf_correlation = 0.75f;
    record.snr_db = 18.25f;
    record.rate_mbps = 24;
    record.psdu_length = 128;
    record.fcs_valid = true;
    record.mac_valid = true;
    record.frame.type = 1;
    record.frame.subtype = 13;
    record.frame.destination = "dd:0e:e0:3e:44:08";
    record.channel = active_csi(estimated);
    auto binary = encode_binary_csi(record, 7, true);
    if (binary.size() != 928 || std::memcmp(binary.data(), "BCSI", 4)
        || get_le16(binary.data() + 4) != 1 || get_le16(binary.data() + 6) != 96
        || get_le32(binary.data() + 8) != binary.size()
        || get_le32(binary.data() + 12) != 15 || binary[59] != 52) {
        std::cerr << "self-test: CSI binary serialization mismatch\n";
        return false;
    }
    return true;
}

void handle_signal(int)
{
    interrupted = true;
}

} // namespace

int main(int argc, char **argv)
{
    Options options;
    if (!parse_options(argc, argv, options)) {
        usage(argv[0]);
        return 2;
    }
    if (options.self_test) {
        bool passed = coding_self_test() && csi_self_test();
        std::cout << (passed ? "Wi-Fi decoder self-test passed\n" : "Wi-Fi decoder self-test failed\n");
        return passed ? 0 : 1;
    }
    CsiWriter csi;
    if (!csi.open(options))
        return 1;
    ::signal(SIGINT, handle_signal);
    ::signal(SIGTERM, handle_signal);
    std::unique_ptr<birria_tui::Screen> screen;
    if (options.interactive) {
        if (!birria_tui::Screen::terminal_available()) {
            std::cerr << "Interactive mode requires a terminal on standard output.\n";
            return 2;
        }
        screen = std::make_unique<birria_tui::Screen>("BirriaSDR Wi-Fi monitor",
            std::vector<birria_tui::Column>{
                {"Center/MHz", 10, true}, {"Rate/Mbps", 9, true},
                {"PDU", 14, false}, {"RSSI/dBFS", 10, true},
                {"SNR/dB", 8, true}, {"Address", 17, false},
                {"Details", 0, false}
            });
        screen->set_status("Starting capture. Ctrl-C to stop.");
    }
    auto show_status = [&](const std::string &message) {
        if (screen)
            screen->set_status(message);
        else
            std::cerr << message << '\n';
    };

    DecoderStats totals;
    bool input_failed = false;
    std::array<uint8_t, 65536> buffer{};
    if (!options.file.empty()) {
        Decoder decoder(options.center_frequency, options.center_frequency,
                        options.sample_rate, 0, options.compact, options.verbose, &csi,
                        screen.get());
        FILE *file = std::fopen(options.file.c_str(), "rb");
        if (!file) {
            std::perror("Input file");
            return 1;
        }
        {
            std::ostringstream message;
            message << "Reading " << options.sample_rate
                    << " S/s little-endian CS16 from " << options.file << '.';
            show_status(message.str());
        }
        while (!interrupted) {
            size_t count = std::fread(buffer.data(), 1, buffer.size(), file);
            if (count)
                decoder.feed_cs16_stream(buffer.data(), count);
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
        if (decoder.trailing_bytes()) {
            std::cerr << "Input file ends with " << decoder.trailing_bytes()
                      << " incomplete CS16 byte(s).\n";
            input_failed = true;
        }
        totals = decoder.stats();
    } else if (options.udp) {
        Decoder decoder(options.center_frequency, options.center_frequency,
                        options.sample_rate, 0, options.compact, options.verbose, &csi,
                        screen.get());
        int fd = open_udp_receiver(options);
        if (fd < 0)
            return 1;
        {
            std::ostringstream message;
            message << "Listening for " << options.sample_rate
                    << " S/s little-endian CS16 on " << options.bind << ':' << options.port
                    << ". Legacy 20 MHz OFDM only.";
            show_status(message.str());
        }
        while (!interrupted) {
            ssize_t count = recv(fd, buffer.data(), buffer.size(), 0);
            if (count > 0)
                decoder.feed_cs16(buffer.data(), static_cast<size_t>(count));
            else if (count < 0 && errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                std::perror("UDP receive");
                input_failed = true;
                break;
            }
        }
        close(fd);
        totals = decoder.stats();
    } else {
        if (!options.interactive)
            birria_native::print_rate_warning(options.sample_rate);
        std::vector<uint64_t> frequencies;
        if (options.sweep && options.sample_rate == 40000000u) {
            for (unsigned channel = 1; channel <= 13; ++channel)
                frequencies.push_back(UINT64_C(2412000000) + UINT64_C(5000000) * (channel - 1));
        } else if (options.sweep) {
            frequencies.push_back(UINT64_C(2442000000));
        } else {
            frequencies.push_back(options.frequency_set
                ? options.center_frequency : DEFAULT_NATIVE_FREQUENCY);
        }
        {
            std::ostringstream message;
            message << "Native USB device " << options.device_index
                    << " at " << options.sample_rate << " S/s";
            if (options.sweep && options.sample_rate == 40000000u)
                message << ", RF-sweeping Wi-Fi channels 1 through 13 every "
                        << options.dwell_ms << " ms";
            else if (options.sweep)
                message << ", software-sweeping Wi-Fi channels 1 through 13 every "
                        << options.dwell_ms << " ms";
            else
                message << ", center " << frequencies.front() << " Hz";
            if (options.sample_rate != 40000000u)
                message << " (experimental rate)";
            show_status(message.str());
        }
        NativeWifiContext context(options, &csi, screen.get());
        birria_native::Options native_options;
        native_options.device_index = options.device_index;
        native_options.sample_rate = options.sample_rate;
        native_options.recovery_restarts = options.recovery_restarts;
        native_options.manual_gain = options.manual_gain;
        native_options.quiet = options.interactive;
        birria_native::Callbacks callbacks;
        callbacks.samples = NativeWifiContext::samples;
        callbacks.gap = NativeWifiContext::gap;
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
              << ", preamble candidates=" << totals.preamble_candidates
              << ", LTF rejections=" << totals.ltf_rejections
              << ", SIGNAL rejections=" << totals.signal_rejections
              << ", SIGNAL headers=" << totals.signal_headers
              << ", data rejections=" << totals.data_rejections
              << ", valid frames=" << totals.frames
              << ", FCS failures=" << totals.fcs_failures
              << ", malformed datagrams=" << totals.bad_datagrams
              << ", CSI records=" << csi.records() << '\n';
    return input_failed || csi.failed() ? 1 : 0;
}
