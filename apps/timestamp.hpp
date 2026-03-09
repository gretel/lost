// SPDX-License-Identifier: ISC
//
// timestamp.hpp — shared UTC timestamp + structured log helpers for apps.
//
// ts_now()                        → "2026-03-09T14:33:17.342Z"
// log_ts(level, tag, fmt, ...)    → writes to stderr with unified format:
//                                   "<ts>  <level>  <tag>  <message>"
//
// Level strings (caller pads to 5 chars):
//   "debug"  "info "  "warn "  "error"

#ifndef GR4_LORA_TIMESTAMP_HPP
#define GR4_LORA_TIMESTAMP_HPP

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <string>

namespace gr::lora {

/// Return current UTC time as ISO-8601 with milliseconds: "2026-03-09T14:33:17.342Z"
[[nodiscard]] inline std::string ts_now() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch()) % 1000;
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[32];
    std::snprintf(buf, sizeof(buf),
        "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
        tm.tm_hour, tm.tm_min, tm.tm_sec,
        static_cast<int>(ms.count()));
    return buf;
}

/// Log a structured line to stderr:  "<ts>  <level>  <tag>  <message>\n"
/// level: "debug", "info ", "warn ", "error"  (5 chars, caller must pad)
// NOLINTNEXTLINE(cppcoreguidelines-pro-type-vararg)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
inline void log_ts(const char* level, const char* tag, const char* fmt, ...) {
    auto ts = ts_now();
    std::fprintf(stderr, "%s  %s  %s  ", ts.c_str(), level, tag);
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, fmt, args);
    va_end(args);
    std::fputc('\n', stderr);
}
#pragma GCC diagnostic pop

}  // namespace gr::lora

#endif  // GR4_LORA_TIMESTAMP_HPP
