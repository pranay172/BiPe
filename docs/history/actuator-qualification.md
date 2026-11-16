# Exact AK40 V3 qualification audit

Audited 1 October 2026. The selected module is AK40-10 V3.0 KV170 with AK48-2405-2D-A2 driver. Hardware enable remains false. This is a document audit; no motor was commissioned.

| Field | Exact-variant evidence | Decision |
| --- | --- | --- |
| 190 g, Ø53 × 40.2 mm, 10:1, 24 V rating, 1.3/4.1 Nm rated/peak | [Manufacturer product](https://www.cubemars.com/de/product/ak40-10-v3-0-kv170-robotic-actuator.html), saved initial M4 HTML | Retain mechanical candidate |
| Torque constant | Product lists 0.056 Nm/A, without resolving output efficiency or Iq/current conventions | Keep output torque per Iq null |
| 6S full-pack and regeneration voltage | 24 V nominal rating is provided; working limits and permitted transient duration are absent | Keep minimum/maximum bus voltage null |
| MIT ranges and units | [AK V3.2 manual](https://www.cubemars.com/data/cms/202602/ak-series-prodcut-manual-v3-2-0-for-ak-3-0-robotic-actuator.pdf) omits this exact V3 motor from its range table | Do not copy older AK40/larger driver ranges |
| AK48 installation guide | Exact model listed on product page, but its download href is empty | Exact electrical/termination specification unavailable |
| Official repository | [Manufacturer repository](https://github.com/Darrell-Song/CubeMars/tree/main/Driver%20Board), recursive tree at commit `0be97c45aae8026c3250a4ff3cc1af5379268976`, not truncated | AK40-2410-1A-A1 guide is an older driver; no AK48 file found |
| Latest software | [V3.2.3 archive](https://www.cubemars.com/data/cms/202606/ak-series-v3-2-3-upper-computer-for-ak-3-0-motor.zip), SHA256 `38de313cd0e2eb1ab7b980431030c11c30147b3f25f9b2614c6b98b6393b9f84` | Static inspection only; Windows program not run |
| Exact software resources | UTF-16 names include `AK40_10_V3_260515_V3.bin`, `AK40_10V3_MC_DEFAULT.McParams`, `AK40_10V3_APP_DEFAULT.AppParams`. Decompressed text has AK40 V3 `p1=5, i1=15, p2=0, i2=0` | These unnamed gains do not establish MIT ranges, torque mapping, timing, or safe voltage |
| Encoder convention/startup | Exact product advertises dual 16-bit encoders and retained single-turn absolute position | Does not establish CAN zero, multi-turn wrap, sign, persistence or startup procedure |
| Local PD and command timeout | No exact-variant documented timing/mapping/timeout found | Retain unverified simulation assumptions; real mapping is blocked |

Archive inventory, static resource extraction, hashes and repository snapshot are saved under `artifacts/milestone-4/continuation/component-sources/`. The large archive stays locally cached and Git-ignored; its immutable hash and URL are recorded. `tools/audit_actuator_resources.py` performs read-only static inspection. Neither the embedded program nor its firmware was executed.

The generic packet codec is tested as a codec. It is not a qualified AK48 command implementation. A voltage or torque rating from an older motor cannot close these fields. See the prepared manufacturer questions; they have not been sent.
