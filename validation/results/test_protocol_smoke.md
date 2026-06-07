# test_protocol_smoke

## Summary

| Metric | Value |
| --- | --- |
| port | COM4 |
| generated_utc | 2026-06-07T06:04:34Z |
| version_response | V,10,0.3.0 |
| identity_response | I,11,0 |
| echo_response | E,12 |
| valid_control_seq | 14 |
| invalid_control_seq | 14 |

## telemetry

Rows collected: 2

| label | dial_id | seq | angle_decideg | speed_decideg_s | torque_ma | foc_rate_hz | status_bits |
| --- | --- | --- | --- | --- | --- | --- | --- |
| valid_control | 0 | 14 | 100 | 32 | -437 | 985 | 7 |
| after_invalid_control | 0 | 14 | 99 | -44 | -433 | 985 | 71 |
