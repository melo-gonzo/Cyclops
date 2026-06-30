// Host-native unit tests for the pure thermal governor: a fixed-window moving
// average of die temp + a single hard cutoff (no hysteresis). pio test -e native

#include <unity.h>
#include "thermal.h"

using namespace thermal;

static const float CUT = 100.0f;

// ---- hard cutoff ----

void test_overcutoff_pauses_at_or_above(void) {
  TEST_ASSERT_TRUE(overCutoff(100.0f, CUT));
  TEST_ASSERT_TRUE(overCutoff(120.0f, CUT));
}
void test_overcutoff_runs_below(void) {
  TEST_ASSERT_FALSE(overCutoff(99.9f, CUT));
  TEST_ASSERT_FALSE(overCutoff(50.0f, CUT));
}

// ---- moving average ----

void test_avg_empty_then_fills(void) {
  MovingAvg<6> a;
  TEST_ASSERT_EQUAL_INT(0, a.samples());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, a.value());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 10.0f, a.push(10.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, a.push(20.0f)); // (10+20)/2
  TEST_ASSERT_EQUAL_INT(2, a.samples());
}

void test_avg_slides_window(void) {
  MovingAvg<3> a;
  a.push(90.0f); a.push(90.0f); a.push(90.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 90.0f, a.value());
  // window now {90,90,120}: the oldest 90 is still in
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, a.push(120.0f));
  // next push evicts the first 90 -> {90,120,90}
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 100.0f, a.push(90.0f));
}

void test_avg_reset(void) {
  MovingAvg<3> a;
  a.push(100.0f); a.push(100.0f);
  a.reset();
  TEST_ASSERT_EQUAL_INT(0, a.samples());
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 42.0f, a.push(42.0f));
}

// ---- governor behaviour: average + cutoff ----

void test_single_spike_rejected(void) {
  MovingAvg<6> a;
  for (int i = 0; i < 6; i++) a.push(80.0f);          // steady, cool
  // one 130C spike: avg = (80*5 + 130)/6 = 88.3 -> stays running
  TEST_ASSERT_FALSE(overCutoff(a.push(130.0f), CUT));
}

void test_sustained_heat_trips(void) {
  MovingAvg<6> a;
  for (int i = 0; i < 6; i++) a.push(80.0f);
  bool paused = false;
  for (int i = 0; i < 6 && !paused; i++) paused = overCutoff(a.push(130.0f), CUT);
  TEST_ASSERT_TRUE(paused); // average climbs over the cutoff and stays
}

void test_resumes_when_average_drops(void) {
  MovingAvg<3> a;
  a.push(110.0f); a.push(110.0f); a.push(110.0f);
  TEST_ASSERT_TRUE(overCutoff(a.value(), CUT));
  a.push(80.0f); a.push(80.0f); a.push(80.0f);        // cooled down
  TEST_ASSERT_FALSE(overCutoff(a.value(), CUT));       // no dead band: resumes
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_overcutoff_pauses_at_or_above);
  RUN_TEST(test_overcutoff_runs_below);
  RUN_TEST(test_avg_empty_then_fills);
  RUN_TEST(test_avg_slides_window);
  RUN_TEST(test_avg_reset);
  RUN_TEST(test_single_spike_rejected);
  RUN_TEST(test_sustained_heat_trips);
  RUN_TEST(test_resumes_when_average_drops);
  return UNITY_END();
}
