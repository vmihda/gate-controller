// Minimal assertion helpers for host tests, with no external dependencies.
#pragma once

#include <math.h>
#include <stdio.h>

static int minitest_failures = 0;
static const char *minitest_current = "";

#define CHECK(condition)                                                              \
    do {                                                                              \
        if (!(condition)) {                                                           \
            printf("FAIL %s:%d in %s: %s\n", __FILE__, __LINE__, minitest_current, #condition); \
            minitest_failures++;                                                      \
        }                                                                             \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance) CHECK(fabs((double)(actual) - (double)(expected)) <= (tolerance))

#define RUN_TEST(test)                 \
    do {                               \
        minitest_current = #test;      \
        test();                        \
        printf("ran %s\n", #test);     \
    } while (0)

#define MINITEST_RESULT() (minitest_failures == 0 ? 0 : 1)
