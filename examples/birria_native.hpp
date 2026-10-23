/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wang Kang
 * Shared native BirriaSDR capture runner for standalone C++ decoders.
 */
#ifndef BIRRIA_NATIVE_HPP
#define BIRRIA_NATIVE_HPP

#include "birriasdr.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>
#include <vector>

namespace birria_native {

struct Options {
    uint32_t device_index = 0;
    uint32_t sample_rate = 40000000;
    uint32_t recovery_restarts = 3;
    bool manual_gain = false;
    bool quiet = false;
};

struct Callbacks {
    void (*samples)(const int16_t *iq, uint32_t count, uint64_t first_sample,
                    uint64_t frequency, uint64_t epoch, void *context) = nullptr;
    void (*gap)(const birriasdr_recovery_event *event, uint64_t frequency,
                uint64_t epoch, void *context) = nullptr;
    void *context = nullptr;
};

struct Result {
    int stream_error = 0;
    int control_error = 0;
    int close_error = 0;
    birriasdr_stream_stats stats{};
    uint64_t epochs = 1;
};

class Session {
public:
    Session(const Options &options, const std::vector<uint64_t> &frequencies,
            uint32_t dwell_ms, std::atomic<bool> &interrupted,
            const Callbacks &callbacks, std::vector<uint32_t> step_dwell_ms = {})
        : options_(options), frequencies_(frequencies), dwell_ms_(dwell_ms),
          step_dwell_ms_(std::move(step_dwell_ms)), interrupted_(interrupted),
          callbacks_(callbacks)
    {
    }

    Result run()
    {
        Result output;
        if (frequencies_.empty() || (!step_dwell_ms_.empty()
            && step_dwell_ms_.size() != frequencies_.size())) {
            output.stream_error = BIRRIASDR_ERROR_INVALID;
            return output;
        }
        int result = birriasdr_open(&dev_, options_.device_index);
        if (result < 0) {
            std::fprintf(stderr, "Open: %s\n", birriasdr_strerror(result));
            output.stream_error = result;
            return output;
        }
        result = birriasdr_set_sample_rate(dev_, options_.sample_rate);
        if (result < 0)
            std::fprintf(stderr, "Sample rate: %s\n", birriasdr_strerror(result));
        if (!result) {
            result = birriasdr_set_center_freq(dev_, frequencies_.front());
            if (result < 0)
                std::fprintf(stderr, "Frequency: %s\n", birriasdr_strerror(result));
        }
        if (!result && options_.manual_gain) {
            result = birriasdr_set_tuner_gain_mode(dev_, 1);
            if (!result)
                result = apply_manual_gain(frequencies_.front());
            if (result < 0)
                std::fprintf(stderr, "Gain: %s\n", birriasdr_strerror(result));
        }
        uint32_t recovery_restarts = options_.sample_rate == 160000000u
            ? 0u : options_.recovery_restarts;
        if (!result && recovery_restarts) {
            result = birriasdr_set_recovery(dev_, recovery_restarts,
                                            recovery_callback, this);
            if (result < 0)
                std::fprintf(stderr, "Recovery: %s\n", birriasdr_strerror(result));
        }
        frequency_.store(frequencies_.front());
        if (!result && frequencies_.size() > 1)
            control_thread_ = std::thread(&Session::control_loop, this);
        if (!result)
            result = birriasdr_read_async(dev_, sample_callback, this, 0, 0);
        stop_.store(true);
        condition_.notify_all();
        if (control_thread_.joinable())
            control_thread_.join();
        output.stream_error = result;
        output.control_error = control_error_.load();
        output.epochs = epoch_.load() + 1;
        if (birriasdr_get_stream_stats(dev_, &output.stats) < 0)
            output.stats = {};
        output.close_error = birriasdr_close(dev_);
        dev_ = nullptr;
        return output;
    }

private:
    int apply_manual_gain(uint64_t frequency)
    {
        uint64_t high_min = 0, high_max = 0;
        bool high_band = birriasdr_get_tuning_band(dev_, 1, &high_min, &high_max) == 0
            && frequency >= high_min && frequency <= high_max;
        const birriasdr_gain gain{static_cast<uint8_t>(high_band ? 5 : 2), 8, 0};
        return birriasdr_set_gain_codes(dev_, &gain);
    }

    static void sample_callback(const int16_t *iq, uint32_t count, uint64_t first_sample,
                                void *context)
    {
        Session *session = static_cast<Session *>(context);
        if (session->callbacks_.samples) {
            session->callbacks_.samples(iq, count, first_sample,
                session->frequency_.load(), session->epoch_.load(),
                session->callbacks_.context);
        }
        if (session->interrupted_.load() || session->stop_.load())
            (void)birriasdr_cancel_async(session->dev_);
    }

    static void recovery_callback(const birriasdr_recovery_event *event, void *context)
    {
        Session *session = static_cast<Session *>(context);
        uint64_t epoch = session->epoch_.fetch_add(1) + 1;
        if (session->callbacks_.gap) {
            session->callbacks_.gap(event, session->frequency_.load(), epoch,
                                    session->callbacks_.context);
        }
        if (session->interrupted_.load())
            (void)birriasdr_cancel_async(session->dev_);
    }

    void control_loop()
    {
        size_t position = 0;
        std::unique_lock<std::mutex> lock(mutex_);
        while (!stop_.load() && !interrupted_.load()) {
            uint32_t wait_ms = step_dwell_ms_.empty() ? dwell_ms_
                : step_dwell_ms_[position];
            if (condition_.wait_for(lock, std::chrono::milliseconds(wait_ms),
                                    [this] { return stop_.load() || interrupted_.load(); }))
                break;
            position = (position + 1) % frequencies_.size();
            uint64_t requested = frequencies_[position];
            lock.unlock();
            int result = birriasdr_set_center_freq(dev_, requested);
            if (result < 0) {
                std::fprintf(stderr, "Sweep frequency %llu Hz: %s\n",
                    static_cast<unsigned long long>(requested),
                    birriasdr_strerror(result));
                control_error_.store(result);
                stop_.store(true);
                (void)birriasdr_cancel_async(dev_);
                condition_.notify_all();
                return;
            }
            if (options_.manual_gain) {
                result = apply_manual_gain(requested);
                if (result < 0) {
                    std::fprintf(stderr, "Sweep gain at %llu Hz: %s\n",
                        static_cast<unsigned long long>(requested),
                        birriasdr_strerror(result));
                    control_error_.store(result);
                    stop_.store(true);
                    (void)birriasdr_cancel_async(dev_);
                    condition_.notify_all();
                    return;
                }
            }
            frequency_.store(requested);
            uint64_t epoch = epoch_.fetch_add(1) + 1;
            if (!options_.quiet) {
                std::fprintf(stderr, "Sweep epoch %llu tuned to %llu Hz.\n",
                    static_cast<unsigned long long>(epoch),
                    static_cast<unsigned long long>(requested));
            }
            lock.lock();
        }
        if (interrupted_.load())
            (void)birriasdr_cancel_async(dev_);
    }

    Options options_;
    std::vector<uint64_t> frequencies_;
    uint32_t dwell_ms_;
    std::vector<uint32_t> step_dwell_ms_;
    std::atomic<bool> &interrupted_;
    Callbacks callbacks_;
    birriasdr_dev_t *dev_ = nullptr;
    std::atomic<bool> stop_{false};
    std::atomic<int> control_error_{0};
    std::atomic<uint64_t> frequency_{0};
    std::atomic<uint64_t> epoch_{0};
    std::thread control_thread_;
    std::mutex mutex_;
    std::condition_variable condition_;
};

inline bool valid_sample_rate(uint32_t rate)
{
    return rate == 40000000u || rate == 80000000u || rate == 160000000u;
}

inline void print_rate_warning(uint32_t rate)
{
    if (rate == 80000000u) {
        std::fprintf(stderr,
            "WARNING: 80 MSps is experimental and requires about 320 MB/s of CS16 payload.\n");
    } else if (rate == 160000000u) {
        std::fprintf(stderr,
            "WARNING: 160 MSps CS16 is experimental and expected to overrun the current "
            "USB exporter. Recovery is disabled so the first overrun stops the run.\n");
    }
}

} // namespace birria_native

#endif
