# test_bounds_behavior

## Summary

| Metric | Value |
| --- | --- |
| port | COM4 |
| generated_utc | 2026-06-07T06:29:41Z |
| baseline_angle_decideg | 210 |
| inside_margin | 60 |
| oob_offset | 220 |
| oob_window | 120 |
| inside_torque_ma | -16 |
| upper_oob_torque_ma | -1470 |
| lower_oob_torque_ma | 1469 |

## bounds_samples

Rows collected: 5

| label | dial_id | seq | angle_decideg | speed_decideg_s | torque_ma | foc_rate_hz | status_bits |
| --- | --- | --- | --- | --- | --- | --- | --- |
| inside | 0 | 300 | 214 | 223 | -16 | 975 | 7 |
| invalid_bounds_fault | 0 | 300 | 214 | 36 | -20 | 975 | 71 |
| upper_oob | 0 | 302 | 208 | -350 | -1470 | 975 | 39 |
| lower_oob | 0 | 303 | 212 | 491 | 1469 | 970 | 39 |
| restored | 0 | 304 | 230 | 678 | -91 | 970 | 7 |
