/**
 * @file Profiler.h
 * @brief 仅在定义 POINTLIO_ESKF_PROFILE 时生效的轻量计时器.
 *
 * 用法:
 *   PROF_SCOPE(0, "stateTransform");    // 函数入口, 析构时累加
 *   PROF_EFFECT(1, "selected", n);      // 附带一个整数统计 (如有效点数)
 *
 * 每 5 秒向 stderr 打印一次累计值. 输出含义:
 *   calls  调用次数
 *   mean   单次平均耗时 (us)
 *   max    单次最大耗时 (us)
 *   n      附带统计的样本数, mean/max/sum 是它们的均值、最大值与总和
 *
 * 未定义 POINTLIO_ESKF_PROFILE 时, 宏展开为空, 不产生任何代码.
 */

#pragma once

#ifdef POINTLIO_ESKF_PROFILE

#include <chrono>
#include <cstdint>
#include <cstdio>

namespace plprof {

inline constexpr int kSlots = 24;

struct Counter {
  const char* name{nullptr};
  std::uint64_t calls{0};
  std::uint64_t total_ns{0};
  std::uint64_t max_ns{0};
  std::uint64_t effect_calls{0};
  std::uint64_t effect_sum{0};
  std::uint64_t effect_max{0};
};

inline Counter* table() {
  static Counter t[kSlots];
  return t;
}

inline std::uint64_t nowNs() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

inline void dump();

struct Scope {
  int idx;
  std::uint64_t t0;

  Scope(int i, const char* n) : idx(i), t0(nowNs()) {
    if (table()[i].name == nullptr) {
      table()[i].name = n;
    }
  }

  ~Scope() {
    Counter& c = table()[idx];
    const std::uint64_t t = nowNs();
    const std::uint64_t dt = t - t0;
    ++c.calls;
    c.total_ns += dt;
    if (dt > c.max_ns) {
      c.max_ns = dt;
    }
    static std::uint64_t last_dump = t;
    if (t - last_dump > 5000000000ULL) {
      last_dump = t;
      dump();
    }
  }
};

inline void effect(int i, const char* n, std::uint64_t v) {
  Counter& c = table()[i];
  if (c.name == nullptr) {
    c.name = n;
  }
  ++c.effect_calls;
  c.effect_sum += v;
  if (v > c.effect_max) {
    c.effect_max = v;
  }
}

inline void dump() {
  std::fprintf(stderr, "[prof] -------------------------------------------------\n");
  for (int i = 0; i < kSlots; ++i) {
    const Counter& c = table()[i];
    if (c.name == nullptr) {
      continue;
    }
    std::fprintf(stderr, "[prof] %-20s", c.name);
    if (c.calls != 0) {
      std::fprintf(stderr, " calls=%8llu mean=%8.1f us max=%8.1f us",
                   static_cast<unsigned long long>(c.calls),
                   static_cast<double>(c.total_ns) / 1000.0 / c.calls,
                   static_cast<double>(c.max_ns) / 1000.0);
    }
    if (c.effect_calls != 0) {
      std::fprintf(stderr, " n=%8llu mean=%8.1f max=%8llu sum=%llu",
                   static_cast<unsigned long long>(c.effect_calls),
                   static_cast<double>(c.effect_sum) / c.effect_calls,
                   static_cast<unsigned long long>(c.effect_max),
                   static_cast<unsigned long long>(c.effect_sum));
    }
    std::fprintf(stderr, "\n");
  }
}

}  // namespace plprof

#define PROF_SCOPE(i, n) ::plprof::Scope _prof_scope_##i(i, n)
#define PROF_EFFECT(i, n, v) ::plprof::effect(i, n, static_cast<std::uint64_t>(v))

#else

#define PROF_SCOPE(i, n)
#define PROF_EFFECT(i, n, v)

#endif
