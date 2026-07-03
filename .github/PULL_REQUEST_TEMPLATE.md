## Summary

<!-- What does this change do, and why? -->

## Checklist

- [ ] `cargo fmt --check` passes
- [ ] `cargo clippy --all-targets -- -D warnings` passes
- [ ] `cargo test` passes
- [ ] `cargo check --no-default-features --target thumbv7em-none-eabihf` passes (no_std)
- [ ] Tests added/updated for any behavior change

## Evidence (only for new/changed frame types or payload layouts)

<!--
If this PR adds or changes wire-format parsing (new frame type, new field,
changed byte layout), include one of:
  - A raw byte capture from real hardware showing the frame.
  - A reference to the source cross-checked against (e.g. the ESPHome
    C++ component, or mr60bha2-proto), with relevant lines/commit linked.

Delete this section if not applicable.
-->

## Related issue

<!-- Closes #... -->
