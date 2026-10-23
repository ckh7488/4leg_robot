# Working agreement

Read README.md, docs/STATE.md, design.json, latest docs/audits/ before edits. User language: Korean; short and concrete responses. User budget cap: KRW 1,000,000 including contingency, with existing printer and training PC. No LiDAR reserve or large mission battery in V1.

Use design.json as authoritative parameter source. Keep joint order, units, signs, mass properties and CAD transforms consistent. Do not label stall torque as continuous, no-load RPM as bandwidth, policy frequency as reaction time, or standing as walking. Do not copy over-voltage Microduck settings to XL330.

After each meaningful checkpoint: run relevant reproducible checks, commit, re-read committed inputs as if seeing them first, write an audit with findings and concrete evidence, fix meaningful findings, rerun affected checks, commit fixes. Keep known unknowns visible. Do not manufacture hardware measurements or claim Isaac validation without running Isaac. Record only actual tests.

No purchases, actuator EEPROM writes, or hardware torque enabling from this repository without a separate explicit hardware command. Do not spawn agents unless explicitly requested. Preserve any user changes. Do not force-push. Do not commit environments, credentials, caches, OEM downloaded binaries, or machine-specific paths. A future reviewer should be able to reproduce checks from documented commands.

User standing workflow (2026-10-24): continue in the agreed direction without asking at each step. Complete a concrete step, run checks, commit/push, then briefly state the solved issue and remaining issue. Keep the PM-facing progress log short. The explicit hardware-command restriction above still applies.
