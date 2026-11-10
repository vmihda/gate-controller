/* FLASH signal timing of the Nice controller. Bench values until the site measurement (spec step 0). */
#ifndef FLASH_TIMING_H
#define FLASH_TIMING_H

#define FLASH_SLOW_PERIOD_MS 1000u      /* opening */
#define FLASH_FAST_PERIOD_MS 400u       /* closing */
#define FLASH_BLINK_DUTY_PCT 50u
#define FLASH_PERIOD_TOLERANCE_PCT 25u
#define FLASH_GLITCH_MS 15u             /* low gaps up to this length are ignored */

#endif /* FLASH_TIMING_H */
