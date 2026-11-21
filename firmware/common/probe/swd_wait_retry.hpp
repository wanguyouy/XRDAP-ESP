// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

#include "debug/swd.hpp"

namespace XrDap
{

/// 把主机的 WAIT 重试次数映射成不小于固定时间预算的次数。
///
/// CMSIS-DAP 的 wait retry 是“次数”，同样的次数在高 SWCLK 下覆盖的时间成比例缩短：
/// Keil 在 5 MHz 下发 100 次，只够约 1 ms；目标擦写 Flash 时 AHB-AP 会 WAIT 数毫秒，
/// 探针提前把 WAIT 交回主机，Keil 就报 “Cannot access Memory (@ 0xe000edf0)”。
/// 这里在主机要求的次数之上再加一个按当前 SWCLK 折算的下限，保证覆盖 kWaitBudgetUs。
/// 主机把重试设成 0（明确要求立即返回 WAIT）时保持 0。
///
/// Maps the host's WAIT retry count to a count that covers a fixed time budget. A host
/// retry count covers a time that shrinks with a faster SWCLK: Keil sends 100 at 5 MHz,
/// which is only about 1 ms, while the AHB-AP returns WAIT for several ms while the
/// target erases or programs flash; the probe then hands WAIT back and Keil reports
/// "Cannot access Memory". This wrapper adds a clock-derived floor on top of the host
/// value so that kWaitBudgetUs is always covered. A host value of 0 (return WAIT
/// immediately) is kept.
///
/// @tparam Base 具体的 SWD 引擎 / Concrete SWD engine.
template <typename Base>
class WaitRetrySwd final : public Base
{
 public:
  using Base::Base;

  /// 一次 AP 访问的时钟数保守估计：8 位请求 + 换向 + 33 位数据 + 校验约 48 个 SWCLK，
  /// 取小值让下限偏大，宁可多等也不提前返回 WAIT / Conservative guess of the clocks
  /// per AP access: 8 request + turnaround + 33 data + parity is about 48 SWCLK; a
  /// smaller divisor makes the floor longer, which is the safe direction.
  static constexpr uint32_t kClocksPerAttempt = 32U;
  /// WAIT 重试至少要覆盖的时间 / Time the WAIT retries must cover.
  static constexpr uint32_t kWaitBudgetUs = 100000U;

  LibXR::ErrorCode SetClockHz(uint32_t hz) override
  {
    clock_hz_ = hz;
    const LibXR::ErrorCode EC = Base::SetClockHz(hz);
    Apply();
    return EC;
  }

  /// 与基类同名，覆盖其非虚的 SetTransferPolicy：记住主机下发的策略，只把折算后的次数
  /// 放进基类策略 / Same name as the non-virtual base setter: remember the host policy
  /// and put only the mapped count into the base policy.
  void SetTransferPolicy(const LibXR::Debug::Swd::TransferPolicy& policy)
  {
    host_policy_ = policy;
    Apply();
  }

 private:
  void Apply()
  {
    LibXR::Debug::Swd::TransferPolicy policy = host_policy_;
    policy.wait_retry = EffectiveWaitRetry(host_policy_.wait_retry);
    LibXR::Debug::Swd::SetTransferPolicy(policy);
  }

  /// 主机次数与时钟折算下限取大；0 保持 0 / Larger of the host count and the
  /// clock-derived floor; 0 stays 0.
  [[nodiscard]] uint16_t EffectiveWaitRetry(uint16_t host_retry) const
  {
    if (host_retry == 0U)
    {
      return 0U;
    }
    const uint64_t hz = (clock_hz_ < 1000U) ? 1000U : clock_hz_;
    uint64_t need = (hz * kWaitBudgetUs) / (static_cast<uint64_t>(kClocksPerAttempt) *
                                           1000000ULL);
    if (need > 0xFFFFU)
    {
      need = 0xFFFFU;
    }
    return (need > host_retry) ? static_cast<uint16_t>(need) : host_retry;
  }

  LibXR::Debug::Swd::TransferPolicy host_policy_{};  ///< 主机原始策略 / Host policy.
  uint32_t clock_hz_ = 1000000U;  ///< 最近一次请求的 SWCLK / Last requested SWCLK.
};

}  // namespace XrDap
