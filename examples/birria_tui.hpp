/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Wang Kang
 * Small ANSI terminal table for the standalone protocol scanners.
 */
#ifndef BIRRIA_TUI_HPP
#define BIRRIA_TUI_HPP

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <map>
#include <mutex>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace birria_tui {

class Screen {
public:
    explicit Screen(std::string title) : title_(std::move(title))
    {
        std::fputs("\033[?1049h\033[?25l", stdout);
        std::fflush(stdout);
        worker_ = std::thread(&Screen::loop, this);
    }

    ~Screen()
    {
        stop_.store(true);
        condition_.notify_all();
        if (worker_.joinable())
            worker_.join();
        std::fputs("\033[?25h\033[?1049l", stdout);
        std::fflush(stdout);
    }

    Screen(const Screen &) = delete;
    Screen &operator=(const Screen &) = delete;

    static bool terminal_available()
    {
        return isatty(STDOUT_FILENO) != 0;
    }

    void update(const std::string &key, std::string summary)
    {
        auto now = Clock::now();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (entries_.find(key) == entries_.end() && entries_.size() >= 2048) {
                auto oldest = std::min_element(entries_.begin(), entries_.end(),
                    [](const auto &a, const auto &b) {
                        return a.second.last < b.second.last;
                    });
                if (oldest != entries_.end())
                    entries_.erase(oldest);
            }
            Entry &entry = entries_[key];
            entry.summary = std::move(summary);
            entry.last = now;
            ++entry.count;
        }
        condition_.notify_one();
    }

    void set_status(std::string status)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            status_ = std::move(status);
        }
        condition_.notify_one();
    }

private:
    using Clock = std::chrono::steady_clock;

    struct Entry {
        std::string summary;
        Clock::time_point last{};
        uint64_t count = 0;
    };

    static std::string age_text(Clock::duration age)
    {
        double seconds = std::chrono::duration<double>(age).count();
        char text[32];
        if (seconds < 10.0)
            std::snprintf(text, sizeof(text), "%.1fs", seconds);
        else if (seconds < 60.0)
            std::snprintf(text, sizeof(text), "%.0fs", seconds);
        else
            std::snprintf(text, sizeof(text), "%llum%02llus",
                static_cast<unsigned long long>(seconds) / 60,
                static_cast<unsigned long long>(seconds) % 60);
        return text;
    }

    void loop()
    {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        while (!stop_.load()) {
            render();
            condition_.wait_for(lock, std::chrono::milliseconds(250));
        }
        render();
    }

    void render()
    {
        struct winsize size{};
        unsigned columns = 120;
        unsigned lines = 30;
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0) {
            if (size.ws_col)
                columns = size.ws_col;
            if (size.ws_row)
                lines = size.ws_row;
        }
        std::vector<Entry> entries;
        std::string status;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            entries.reserve(entries_.size());
            for (const auto &item : entries_)
                entries.push_back(item.second);
            status = status_;
        }
        std::sort(entries.begin(), entries.end(), [](const Entry &a, const Entry &b) {
            return a.last > b.last;
        });
        auto now = Clock::now();
        size_t available = lines > 5 ? lines - 5 : 1;
        std::printf("\033[H\033[2J%s\n", title_.c_str());
        std::printf("%s\n", status.empty() ? "Ctrl-C to stop" : status.c_str());
        std::printf("%-10s %8s  %s\n", "LAST SEEN", "COUNT", "DETAILS");
        size_t shown = std::min(available, entries.size());
        for (size_t i = 0; i < shown; ++i) {
            std::string age = age_text(now - entries[i].last);
            size_t fixed = 22;
            size_t width = columns > fixed ? columns - fixed : 1;
            std::string summary = entries[i].summary;
            if (summary.size() > width) {
                if (width > 3)
                    summary = summary.substr(0, width - 3) + "...";
                else
                    summary = summary.substr(0, width);
            }
            std::printf("%-10s %8llu  %s\n", age.c_str(),
                static_cast<unsigned long long>(entries[i].count), summary.c_str());
        }
        for (size_t i = shown; i < available; ++i)
            std::putchar('\n');
        std::printf("%zu unique rows", entries.size());
        if (entries.size() > shown)
            std::printf(", %zu hidden by terminal height", entries.size() - shown);
        std::printf("\n");
        std::fflush(stdout);
    }

    std::string title_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
    std::mutex mutex_;
    std::map<std::string, Entry> entries_;
    std::string status_;
    std::mutex wait_mutex_;
    std::condition_variable condition_;
};

} // namespace birria_tui

#endif
