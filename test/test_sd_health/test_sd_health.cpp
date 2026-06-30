// Host-native unit tests for the pure SD mount/recovery policy. pio test -e native
// Pins the timing/backoff that caused an earlier field regression.

#include <unity.h>
#include "sd_health.h"

using namespace sdhealth;

// Small, readable config: probe every 5000, backoff 2000 -> 30000.
static const Config C = {5000, 2000, 30000};

void test_not_due_returns_none(void) {
  State s = {true, 10000, 2000};
  TEST_ASSERT_EQUAL_INT(NONE, nextAction(s, 9999)); // before nextProbeMs
}
void test_due_when_mounted_probes(void) {
  State s = {true, 10000, 2000};
  TEST_ASSERT_EQUAL_INT(PROBE, nextAction(s, 10000));
  TEST_ASSERT_EQUAL_INT(PROBE, nextAction(s, 12000));
}
void test_due_when_down_remounts(void) {
  State s = {false, 10000, 2000};
  TEST_ASSERT_EQUAL_INT(REMOUNT, nextAction(s, 10000));
}

void test_probe_alive_schedules_next_probe(void) {
  State s = {true, 10000, 2000};
  onProbeResult(s, C, 10000, true);
  TEST_ASSERT_TRUE(s.available);
  TEST_ASSERT_EQUAL_UINT32(15000, s.nextProbeMs); // now + probeMs
}
void test_probe_dead_marks_down_and_retries_now(void) {
  State s = {true, 10000, 8000};
  onProbeResult(s, C, 12000, false);
  TEST_ASSERT_FALSE(s.available);
  TEST_ASSERT_EQUAL_UINT32(2000, s.recoverDelayMs); // reset to min
  TEST_ASSERT_EQUAL_UINT32(12000, s.nextProbeMs);   // retry immediately
}

void test_remount_ok_resumes_probing(void) {
  State s = {false, 12000, 16000};
  onRemountResult(s, C, 12000, true);
  TEST_ASSERT_TRUE(s.available);
  TEST_ASSERT_EQUAL_UINT32(2000, s.recoverDelayMs);
  TEST_ASSERT_EQUAL_UINT32(17000, s.nextProbeMs); // now + probeMs
}
void test_remount_fail_backs_off_exponentially_to_ceiling(void) {
  State s = {false, 0, 2000};
  uint32_t now = 100000;
  onRemountResult(s, C, now, false);
  TEST_ASSERT_EQUAL_UINT32(4000, s.recoverDelayMs);
  TEST_ASSERT_EQUAL_UINT32(now + 4000, s.nextProbeMs);
  onRemountResult(s, C, now, false); TEST_ASSERT_EQUAL_UINT32(8000, s.recoverDelayMs);
  onRemountResult(s, C, now, false); TEST_ASSERT_EQUAL_UINT32(16000, s.recoverDelayMs);
  onRemountResult(s, C, now, false); TEST_ASSERT_EQUAL_UINT32(32000, s.recoverDelayMs); // 16k<30k -> *2
  onRemountResult(s, C, now, false); TEST_ASSERT_EQUAL_UINT32(30000, s.recoverDelayMs); // >=ceil -> ceil
  onRemountResult(s, C, now, false); TEST_ASSERT_EQUAL_UINT32(30000, s.recoverDelayMs); // stays
}

void test_remount_backoff_grows_from_zero_minimum(void) {
  // A 0 recoverMinMs floor used to make the backoff stick at 0 forever (0*2==0),
  // so it never grew. A 0 current delay is now treated as 1 and starts doubling.
  const Config Z = {5000, 0, 30000}; // min = 0
  State s = {false, 0, 0};           // recoverDelayMs seeded from the 0 min
  uint32_t now = 100000;
  onRemountResult(s, Z, now, false); TEST_ASSERT_EQUAL_UINT32(2, s.recoverDelayMs);  // 1*2
  TEST_ASSERT_EQUAL_UINT32(now + 2, s.nextProbeMs);
  onRemountResult(s, Z, now, false); TEST_ASSERT_EQUAL_UINT32(4, s.recoverDelayMs);
  onRemountResult(s, Z, now, false); TEST_ASSERT_EQUAL_UINT32(8, s.recoverDelayMs);  // keeps growing
}

void test_mark_down_drops_and_retries_now(void) {
  State s = {true, 50000, 16000};
  markDown(s, C, 33000);
  TEST_ASSERT_FALSE(s.available);
  TEST_ASSERT_EQUAL_UINT32(2000, s.recoverDelayMs);
  TEST_ASSERT_EQUAL_UINT32(33000, s.nextProbeMs);
}

void test_due_is_wraparound_safe(void) {
  // nextProbeMs just before a millis() wrap; now just after -> still "due"
  State s = {false, 0xFFFFFF00u, 2000};
  TEST_ASSERT_TRUE(due(s, 0x00000010u)); // (int32_t)(now - next) >= 0
}

// ---- lowSpaceNext: free-space backstop hysteresis (floor=256, hyst=128) ----
void test_low_space_trips_below_floor(void) {
  TEST_ASSERT_TRUE(lowSpaceNext(false, 200, 256, 128));  // free < floor -> low
  TEST_ASSERT_FALSE(lowSpaceNext(false, 256, 256, 128)); // exactly at floor -> not low
  TEST_ASSERT_FALSE(lowSpaceNext(false, 500, 256, 128)); // plenty free
}
void test_low_space_holds_until_recovered_past_margin(void) {
  // Once low, stays low through the hysteresis band (floor..floor+hyst).
  TEST_ASSERT_TRUE(lowSpaceNext(true, 300, 256, 128));   // recovered past floor but < floor+hyst
  TEST_ASSERT_TRUE(lowSpaceNext(true, 383, 256, 128));   // still inside the band
  TEST_ASSERT_FALSE(lowSpaceNext(true, 384, 256, 128));  // >= floor+hyst -> clears
}
void test_low_space_no_oscillation_in_band(void) {
  // A value inside the band keeps whatever state it had (no flip-flop).
  TEST_ASSERT_FALSE(lowSpaceNext(false, 300, 256, 128)); // not low + in band -> stays not low
  TEST_ASSERT_TRUE(lowSpaceNext(true, 300, 256, 128));   // low + in band -> stays low
}

void setUp(void) {}
void tearDown(void) {}

int main(int, char **) {
  UNITY_BEGIN();
  RUN_TEST(test_not_due_returns_none);
  RUN_TEST(test_due_when_mounted_probes);
  RUN_TEST(test_due_when_down_remounts);
  RUN_TEST(test_probe_alive_schedules_next_probe);
  RUN_TEST(test_probe_dead_marks_down_and_retries_now);
  RUN_TEST(test_remount_ok_resumes_probing);
  RUN_TEST(test_remount_fail_backs_off_exponentially_to_ceiling);
  RUN_TEST(test_remount_backoff_grows_from_zero_minimum);
  RUN_TEST(test_mark_down_drops_and_retries_now);
  RUN_TEST(test_due_is_wraparound_safe);
  RUN_TEST(test_low_space_trips_below_floor);
  RUN_TEST(test_low_space_holds_until_recovered_past_margin);
  RUN_TEST(test_low_space_no_oscillation_in_band);
  return UNITY_END();
}
