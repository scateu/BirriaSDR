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
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace birria_tui {

struct Column {
    std::string heading;
    size_t width;
    bool right = false;
};

class Screen {
public:
    Screen(std::string title, std::vector<Column> columns)
        : title_(std::move(title)), columns_(std::move(columns))
    {
        std::fputs("\033[?1049h\033[?25l\033[2J\033[H", stdout);
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

    void update(const std::string &key, std::vector<std::string> values)
    {
        auto now = Clock::now();
        values.resize(columns_.size());
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
            if (!entry.count)
                entry.order = next_order_++;
            entry.values = std::move(values);
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
        std::vector<std::string> values;
        Clock::time_point last{};
        uint64_t count = 0;
        uint64_t order = 0;
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

    static std::string fit(std::string text, size_t width, bool right)
    {
        if (text.size() > width) {
            if (width > 3)
                text = text.substr(0, width - 3) + "...";
            else
                text.resize(width);
        }
        if (text.size() < width) {
            size_t padding = width - text.size();
            if (right)
                text.insert(0, padding, ' ');
            else
                text.append(padding, ' ');
        }
        return text;
    }

    void loop()
    {
        std::unique_lock<std::mutex> lock(wait_mutex_);
        while (!stop_.load()) {
            render();
            condition_.wait_for(lock, std::chrono::milliseconds(500),
                                [this] { return stop_.load(); });
        }
    }

    void render()
    {
        struct winsize size{};
        unsigned terminal_columns = 120;
        unsigned lines = 30;
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0) {
            if (size.ws_col)
                terminal_columns = size.ws_col;
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
            if (a.count != b.count)
                return a.count > b.count;
            return a.order < b.order;
        });

        constexpr size_t age_width = 9;
        constexpr size_t count_width = 8;
        constexpr size_t prefix_width = age_width + 1 + count_width + 2;
        size_t available_width = terminal_columns > prefix_width
            ? terminal_columns - prefix_width : 1;
        std::vector<size_t> widths(columns_.size());
        size_t fixed_width = 0;
        for (size_t i = 0; i < columns_.size(); ++i) {
            widths[i] = columns_[i].width;
            if (i + 1 < columns_.size())
                fixed_width += widths[i] + 1;
        }
        if (!widths.empty()) {
            size_t minimum_last = 8;
            while (fixed_width + minimum_last > available_width) {
                bool reduced = false;
                for (size_t i = columns_.size() - 1; i-- > 0;) {
                    if (widths[i] > 4) {
                        --widths[i];
                        --fixed_width;
                        reduced = true;
                        if (fixed_width + minimum_last <= available_width)
                            break;
                    }
                }
                if (!reduced)
                    break;
            }
            widths.back() = available_width > fixed_width
                ? available_width - fixed_width : 1;
        }

        auto now = Clock::now();
        size_t available_lines = lines > 5 ? lines - 5 : 1;
        std::ostringstream frame;
        auto line = [&](const std::string &text) {
            frame << "\033[2K" << text << '\n';
        };
        frame << "\033[H";
        line(title_);
        line(status.empty() ? "Ctrl-C to stop" : status);
        std::string header = fit("LAST SEEN", age_width, false) + " "
            + fit("COUNT", count_width, true) + "  ";
        for (size_t i = 0; i < columns_.size(); ++i) {
            header += fit(columns_[i].heading, widths[i], columns_[i].right);
            if (i + 1 < columns_.size())
                header += ' ';
        }
        line(header);

        size_t shown = std::min(available_lines, entries.size());
        for (size_t row = 0; row < shown; ++row) {
            std::string text = fit(age_text(now - entries[row].last), age_width, false)
                + " " + fit(std::to_string(entries[row].count), count_width, true) + "  ";
            for (size_t column = 0; column < columns_.size(); ++column) {
                std::string value = column < entries[row].values.size()
                    ? entries[row].values[column] : std::string();
                text += fit(std::move(value), widths[column], columns_[column].right);
                if (column + 1 < columns_.size())
                    text += ' ';
            }
            line(text);
        }
        for (size_t i = shown; i < available_lines; ++i)
            line("");
        std::ostringstream footer;
        footer << entries.size() << " unique rows, sorted by count";
        if (entries.size() > shown)
            footer << ", " << entries.size() - shown << " hidden by terminal height";
        line(footer.str());
        std::string output = frame.str();
        std::fwrite(output.data(), 1, output.size(), stdout);
        std::fflush(stdout);
    }

    std::string title_;
    std::vector<Column> columns_;
    std::atomic<bool> stop_{false};
    std::thread worker_;
    std::mutex mutex_;
    std::map<std::string, Entry> entries_;
    uint64_t next_order_ = 0;
    std::string status_;
    std::mutex wait_mutex_;
    std::condition_variable condition_;
};

} // namespace birria_tui

#endif
