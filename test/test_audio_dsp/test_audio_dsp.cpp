// Host-native unit tests for the pure audio trigger/threshold core.
// Run with: pio test -e native
//
// These pin the exact field-tuned behaviour we kept getting wrong by hand
// (min_thr clamp, manual override, floor-only-learns-below-threshold, the
// marginal "rms barely over threshold" triggers seen in the logs).

#include <unity.h>
#include "audio_dsp.h"

using namespace audsp;

// --- adaptiveThreshold: floor*factor, clamped up to minThr ---------------------
void test_adaptive_threshold_uses_floor_times_factor(void) {
  TEST_ASSERT_EQUAL_FLOAT(3000.0f, adaptiveThreshold(1000.0f, 3.0f, 500.0f));
}
void test_adaptive_threshold_clamps_to_min_in_quiet_room(void) {
  // Quiet room: floor 100 * 3 = 300, but min_thr 2500 must win (this is the
  // fix for the noise-floor hair-trigger from the field logs).
  TEST_ASSERT_EQUAL_FLOAT(2500.0f, adaptiveThreshold(100.0f, 3.0f, 2500.0f));
}

// --- clampClipMs: NVS-load range guard (memory-safety) ----------------------
void test_clamp_clip_ms_passes_through_in_range(void) {
  TEST_ASSERT_EQUAL_UINT32(5000, clampClipMs(5000, 10000));
}
void test_clamp_clip_ms_clamps_below_minimum(void) {
  // < 500ms (incl. 0) snaps up to the 500ms floor the config handler enforces.
  TEST_ASSERT_EQUAL_UINT32(500, clampClipMs(0, 10000));
  TEST_ASSERT_EQUAL_UINT32(500, clampClipMs(100, 10000));
}
void test_clamp_clip_ms_clamps_above_max_prevents_slot_overrun(void) {
  // The latent bug: a stored value over MAX_CLIP_MS must be capped or it
  // overruns the WAV slot in finalizeClip's memcpy.
  TEST_ASSERT_EQUAL_UINT32(10000, clampClipMs(99999, 10000));
}

// --- effectiveThreshold: manual override wins -------------------------------
void test_manual_override_wins_when_set(void) {
  TEST_ASSERT_EQUAL_FLOAT(8000.0f, effectiveThreshold(3000.0f, 8000.0f));
}
void test_manual_override_ignored_when_zero(void) {
  TEST_ASSERT_EQUAL_FLOAT(3000.0f, effectiveThreshold(3000.0f, 0.0f));
}

// --- adaptFloor: estimator semantics ----------------------------------------
void test_floor_snaps_on_first_sample(void) {
  // floor==0 (uninitialised) snaps straight to the input.
  TEST_ASSERT_EQUAL_FLOAT(1234.0f, adaptFloor(0.0f, 1234.0f, 5000.0f, 0.02f, 0.005f, 0.0f));
}
void test_floor_frozen_above_threshold_when_habituation_off(void) {
  // habituation off (alphaHab=0): a loud event must NOT drag the floor up.
  float f = adaptFloor(1000.0f, 9000.0f, 3000.0f, 0.02f, 0.005f, 0.0f);
  TEST_ASSERT_EQUAL_FLOAT(1000.0f, f);
}
void test_floor_habituates_above_threshold_when_enabled(void) {
  // habituation on: above threshold the floor leaks UP slowly toward rms.
  // 1000 + 0.01*(9000-1000) = 1080
  float f = adaptFloor(1000.0f, 9000.0f, 3000.0f, 0.02f, 0.005f, 0.01f);
  TEST_ASSERT_EQUAL_FLOAT(1080.0f, f);
}
void test_floor_adapts_fast_downward(void) {
  // Room got quieter (rms < floor): use the fast (down) alpha.
  // 1000 + 0.02*(500-1000) = 990
  TEST_ASSERT_EQUAL_FLOAT(990.0f, adaptFloor(1000.0f, 500.0f, 3000.0f, 0.02f, 0.005f, 0.01f));
}
void test_floor_adapts_slow_upward(void) {
  // Creeping ambient (floor < rms < threshold): use the slow (up) alpha,
  // NOT the habituation leak (which only applies above threshold).
  // 1000 + 0.005*(2000-1000) = 1005
  TEST_ASSERT_EQUAL_FLOAT(1005.0f, adaptFloor(1000.0f, 2000.0f, 3000.0f, 0.02f, 0.005f, 0.01f));
}

// --- shouldAutoTrigger: every gate -------------------------------------------
void test_trigger_fires_when_loud_and_all_gates_open(void) {
  TEST_ASSERT_TRUE(shouldAutoTrigger(3000.0f, 2500.0f, true, true, true));
}
void test_trigger_marginal_just_over_threshold_fires(void) {
  // Documents the field behaviour: rms 2511 vs thr 2500 DOES fire (1.004x).
  TEST_ASSERT_TRUE(shouldAutoTrigger(2511.0f, 2500.0f, true, true, true));
}
void test_trigger_blocked_at_or_below_threshold(void) {
  TEST_ASSERT_FALSE(shouldAutoTrigger(2500.0f, 2500.0f, true, true, true)); // == is not >
}
void test_trigger_blocked_when_auto_disabled(void) {
  TEST_ASSERT_FALSE(shouldAutoTrigger(9999.0f, 2500.0f, false, true, true));
}
void test_trigger_blocked_during_warmup(void) {
  TEST_ASSERT_FALSE(shouldAutoTrigger(9999.0f, 2500.0f, true, false, true));
}
void test_trigger_blocked_during_holdoff(void) {
  TEST_ASSERT_FALSE(shouldAutoTrigger(9999.0f, 2500.0f, true, true, false));
}

// --- triggerThreshold: the unified value (matches what audioTask uses) -------
void test_trigger_threshold_adaptive_path(void) {
  TEST_ASSERT_EQUAL_FLOAT(3000.0f, triggerThreshold(1000.0f, 3.0f, 2500.0f, 0.0f));
}
void test_trigger_threshold_clamped_in_quiet_room(void) {
  TEST_ASSERT_EQUAL_FLOAT(2500.0f, triggerThreshold(200.0f, 3.0f, 2500.0f, 0.0f));
}
void test_trigger_threshold_manual_overrides_clamp(void) {
  // Manual value is honoured as-is, even below min_thr (consistent with the
  // actual trigger path - the bug this refactor unified away).
  TEST_ASSERT_EQUAL_FLOAT(800.0f, triggerThreshold(200.0f, 3.0f, 2500.0f, 800.0f));
}

// --- hottestBandOverThreshold: per-band spectral trigger reducer -------------
void test_no_band_over_threshold_returns_minus1(void) {
  float mag[3] = {10.0f, 20.0f, 5.0f};
  float thr[3] = {50.0f, 50.0f, 50.0f};
  TEST_ASSERT_EQUAL_INT(-1, hottestBandOverThreshold(mag, thr, 3));
}
void test_single_band_over_threshold_is_returned(void) {
  float mag[3] = {10.0f, 90.0f, 5.0f};
  float thr[3] = {50.0f, 50.0f, 50.0f};
  TEST_ASSERT_EQUAL_INT(1, hottestBandOverThreshold(mag, thr, 3));
}
void test_largest_margin_wins_when_several_over(void) {
  // band 0 over by 5, band 2 over by 40 -> band 2 wins.
  float mag[3] = {55.0f, 10.0f, 90.0f};
  float thr[3] = {50.0f, 50.0f, 50.0f};
  TEST_ASSERT_EQUAL_INT(2, hottestBandOverThreshold(mag, thr, 3));
}
void test_band_exactly_at_threshold_does_not_trigger(void) {
  float mag[2] = {50.0f, 49.9f};
  float thr[2] = {50.0f, 50.0f};
  TEST_ASSERT_EQUAL_INT(-1, hottestBandOverThreshold(mag, thr, 2));
}

// ---- scaleAlphaForDt: wall-clock correction of EMA rates (D1) ----
void test_scale_alpha_identity_at_nominal_dt(void) {
  // dt == nominal -> unchanged
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, scaleAlphaForDt(0.01f, 16.0f, 16.0f));
}
void test_scale_alpha_grows_with_longer_dt(void) {
  // twice the interval -> twice the per-update rate (same time constant)
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.02f, scaleAlphaForDt(0.01f, 32.0f, 16.0f));
}
void test_scale_alpha_shrinks_with_shorter_dt(void) {
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.005f, scaleAlphaForDt(0.01f, 8.0f, 16.0f));
}
void test_scale_alpha_clamps_ratio_low_and_high(void) {
  // ratio clamped to [0.25, 8]: a tiny dt floors at 0.25x, a huge dt caps at 8x
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f * 0.25f, scaleAlphaForDt(0.01f, 1.0f, 16.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f * 8.0f, scaleAlphaForDt(0.01f, 100000.0f, 16.0f));
}
void test_scale_alpha_never_exceeds_one(void) {
  // a full-step alpha scaled up is still capped at a single EMA step
  TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, scaleAlphaForDt(0.5f, 100.0f, 16.0f));
}

// ---- statistical threshold (mean + k*sigma) ----
void test_statistical_threshold_mean_plus_k_sigma(void) {
  // mean 100, var 400 (sd 20), k 2.5 -> 100 + 50 = 150
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 150.0f, statisticalThreshold(100.0f, 400.0f, 2.5f, 50.0f));
}
void test_statistical_threshold_clamps_to_min(void) {
  // 100 + 2.5*0 = 100, but the absolute min wins
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 300.0f, statisticalThreshold(100.0f, 0.0f, 2.5f, 300.0f));
}
void test_adapt_mean_var_seeds_on_first_sample(void) {
  float mean = 0, var = 0;
  adaptMeanVar(mean, var, 500.0f, 0.1f);
  TEST_ASSERT_EQUAL_FLOAT(500.0f, mean);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, var);
}
void test_adapt_mean_var_converges_to_constant(void) {
  float mean = 0, var = 0;
  for (int i = 0; i < 1000; i++) adaptMeanVar(mean, var, 200.0f, 0.1f);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 200.0f, mean);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, var); // steady input -> ~0 variance
}
void test_statistical_busy_room_raises_threshold_vs_quiet(void) {
  // Same mean (~100) but one stream is highly variable (a busy room): its
  // threshold must demand a much bigger jump than the steady-quiet one.
  float qm = 0, qv = 0, bm = 0, bv = 0;
  for (int i = 0; i < 3000; i++) {
    adaptMeanVar(qm, qv, 100.0f, 0.05f);                  // steady 100
    adaptMeanVar(bm, bv, (i & 1) ? 160.0f : 40.0f, 0.05f); // 40/160, mean 100, high var
  }
  float quietThr = statisticalThreshold(qm, qv, 2.5f, 0.0f);
  float busyThr = statisticalThreshold(bm, bv, 2.5f, 0.0f);
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 100.0f, quietThr);      // quiet: ~mean, sigma~0
  TEST_ASSERT_TRUE(busyThr > quietThr + 80.0f);          // busy: mean + k*big-sigma
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_adaptive_threshold_uses_floor_times_factor);
  RUN_TEST(test_adaptive_threshold_clamps_to_min_in_quiet_room);
  RUN_TEST(test_clamp_clip_ms_passes_through_in_range);
  RUN_TEST(test_clamp_clip_ms_clamps_below_minimum);
  RUN_TEST(test_clamp_clip_ms_clamps_above_max_prevents_slot_overrun);
  RUN_TEST(test_manual_override_wins_when_set);
  RUN_TEST(test_manual_override_ignored_when_zero);
  RUN_TEST(test_floor_snaps_on_first_sample);
  RUN_TEST(test_floor_frozen_above_threshold_when_habituation_off);
  RUN_TEST(test_floor_habituates_above_threshold_when_enabled);
  RUN_TEST(test_floor_adapts_fast_downward);
  RUN_TEST(test_floor_adapts_slow_upward);
  RUN_TEST(test_trigger_fires_when_loud_and_all_gates_open);
  RUN_TEST(test_trigger_marginal_just_over_threshold_fires);
  RUN_TEST(test_trigger_blocked_at_or_below_threshold);
  RUN_TEST(test_trigger_blocked_when_auto_disabled);
  RUN_TEST(test_trigger_blocked_during_warmup);
  RUN_TEST(test_trigger_blocked_during_holdoff);
  RUN_TEST(test_trigger_threshold_adaptive_path);
  RUN_TEST(test_trigger_threshold_clamped_in_quiet_room);
  RUN_TEST(test_trigger_threshold_manual_overrides_clamp);
  RUN_TEST(test_no_band_over_threshold_returns_minus1);
  RUN_TEST(test_single_band_over_threshold_is_returned);
  RUN_TEST(test_largest_margin_wins_when_several_over);
  RUN_TEST(test_band_exactly_at_threshold_does_not_trigger);
  RUN_TEST(test_scale_alpha_identity_at_nominal_dt);
  RUN_TEST(test_scale_alpha_grows_with_longer_dt);
  RUN_TEST(test_scale_alpha_shrinks_with_shorter_dt);
  RUN_TEST(test_scale_alpha_clamps_ratio_low_and_high);
  RUN_TEST(test_scale_alpha_never_exceeds_one);
  RUN_TEST(test_statistical_threshold_mean_plus_k_sigma);
  RUN_TEST(test_statistical_threshold_clamps_to_min);
  RUN_TEST(test_adapt_mean_var_seeds_on_first_sample);
  RUN_TEST(test_adapt_mean_var_converges_to_constant);
  RUN_TEST(test_statistical_busy_room_raises_threshold_vs_quiet);
  return UNITY_END();
}
