# Full Bearamin

- Preserve the Sidecar, MatrixCore and DPT roles. Keep variant pin maps, UART contracts and firmware entrypoints distinct.
- Shared protocol changes require updating and testing all producers and subscribers in the same change; preserve schema/version/CRC and stale/fault behavior unless deliberately migrated.
- Use ./test.sh and ./build.sh from this child root; inspect supported targets and build-side effects before running. Keep generated output ignored.
- Do not flash during cleanup or publication. Establish exact device identity before any separately authorized hardware write.
- Hardware source, build and package checks are not measured voltage, timing, gesture or musical acceptance.

## Agent-assisted collaboration

These instructions make AI-assisted development and review visible. Preserve accurate human, agent and upstream attribution; do not imply sole authorship or manufacture contributor identities. Keep secrets, personal details, device identities and private operational records out of the published source.
