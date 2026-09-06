#include <unity.h>
#include "battery_lipo.hh"

void setUp(void) {}
void tearDown(void) {}

void test_full_at_or_above_4200mv() {
    TEST_ASSERT_EQUAL_UINT8(100, battery_lipo_percent(4200));
    TEST_ASSERT_EQUAL_UINT8(100, battery_lipo_percent(4300));
}

void test_segment_boundaries() {
    TEST_ASSERT_EQUAL_UINT8(90, battery_lipo_percent(4100));
    TEST_ASSERT_EQUAL_UINT8(80, battery_lipo_percent(4000));
    TEST_ASSERT_EQUAL_UINT8(70, battery_lipo_percent(3900));
    TEST_ASSERT_EQUAL_UINT8(50, battery_lipo_percent(3800));
    TEST_ASSERT_EQUAL_UINT8(30, battery_lipo_percent(3700));
    TEST_ASSERT_EQUAL_UINT8(20, battery_lipo_percent(3600));
    TEST_ASSERT_EQUAL_UINT8(10, battery_lipo_percent(3500));
    TEST_ASSERT_EQUAL_UINT8(2, battery_lipo_percent(3400));
    TEST_ASSERT_EQUAL_UINT8(1, battery_lipo_percent(3300));
}

void test_interpolates_within_segments() {
    TEST_ASSERT_EQUAL_UINT8(95, battery_lipo_percent(4150));
    TEST_ASSERT_EQUAL_UINT8(60, battery_lipo_percent(3850));
    TEST_ASSERT_EQUAL_UINT8(40, battery_lipo_percent(3750));
    TEST_ASSERT_EQUAL_UINT8(6, battery_lipo_percent(3450));
}

void test_never_reports_empty() {
    TEST_ASSERT_EQUAL_UINT8(1, battery_lipo_percent(3000));
    TEST_ASSERT_EQUAL_UINT8(1, battery_lipo_percent(0));
}

int main(int argc, char** argv) {
    UNITY_BEGIN();
    RUN_TEST(test_full_at_or_above_4200mv);
    RUN_TEST(test_segment_boundaries);
    RUN_TEST(test_interpolates_within_segments);
    RUN_TEST(test_never_reports_empty);
    return UNITY_END();
}
