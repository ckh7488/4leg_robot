# Working agreement

Read README.md, docs/STATE.md, design.json, latest docs/audits/ before edits. User language: Korean; short and concrete responses. Latest user direction (2026-10-24): short visible limbs and large lightweight character head, BLDC, quadruped locomotion plus actual biped walking and self-righting; initial external DC power tether with pull measurement. KRW 2,000,000 is a soft reference, not a hard cap; it supersedes the previous KRW 1,000,000 cap. Existing printer and training PC. No LiDAR reserve or large mission battery in V1.

Use root design.json as authoritative parameter source for the legacy XL330 model. The new exploratory configuration is concepts/short-limb-v1/design.json; do not mix its assumptions with the old model's validation. Read that file before editing the new concept. Keep joint order, units, signs, mass properties and CAD transforms consistent. Do not label stall torque as continuous, no-load RPM as bandwidth, policy frequency as reaction time, or standing as walking. Do not copy over-voltage Microduck settings to XL330.

Latest direction extends to whole-body physical layout with size and price temporarily deprioritized. Current V2 assumptions are concepts/doro-layout-v2/design.json; read before changing V2. V1 remains a historical small-limb comparator. V2 ideal-axis reach, mass allocations and sampled ankle linkage calculations are not fabrication fit, walking or self-righting validation.

After each meaningful checkpoint: run relevant reproducible checks, commit, re-read committed inputs as if seeing them first, write an audit with findings and concrete evidence, fix meaningful findings, rerun affected checks, commit fixes. Keep known unknowns visible. Do not manufacture hardware measurements or claim Isaac validation without running Isaac. Record only actual tests.

No purchases, actuator EEPROM writes, or hardware torque enabling from this repository without a separate explicit hardware command. Do not spawn agents unless explicitly requested. Preserve any user changes. Do not force-push. Do not commit environments, credentials, caches, OEM downloaded binaries, or machine-specific paths. A future reviewer should be able to reproduce checks from documented commands.

User standing workflow (2026-10-24): continue in the agreed direction without asking at each step. Complete a concrete step, run checks, commit/push, then briefly state the solved issue and remaining issue. Keep the PM-facing progress log short. The explicit hardware-command restriction above still applies.
