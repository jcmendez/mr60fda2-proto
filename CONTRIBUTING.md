# Contributing

Thanks for considering a contribution to `mr60fda2-proto`. By
participating, you're expected to uphold our
[Code of Conduct](CODE_OF_CONDUCT.md).

## Before you start

For anything beyond a small fix (typo, doc tweak, obvious bug), please
open an issue first to discuss the change. This is especially true for
new frame types or payload layouts — since Seeed doesn't publish a spec,
those need to be justified with evidence (see below).

## Development setup

You'll need a stable Rust toolchain (`rust-version` in `Cargo.toml` is the
floor) and the `thumbv7em-none-eabihf` target for `no_std` checks:

```sh
rustup component add rustfmt clippy
rustup target add thumbv7em-none-eabihf
```

## Before opening a PR

Run the same checks CI runs:

```sh
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test
cargo check --no-default-features --target thumbv7em-none-eabihf
```

All four must pass. `cargo fmt` (without `--check`) will fix formatting
for you.

## Scope

This crate covers the MR60FDA2 UART wire protocol only: frame parsing and
command builders. Please keep contributions focused on that — the UART
peripheral driver itself is intentionally out of scope (see `examples/`
for how to integrate with a platform-specific driver).

## Adding or changing frame types / payload layouts

Since there's no official spec, changes to the wire format (new frame
types, new fields, changed byte layouts) need evidence, not just
plausibility. In your PR description, include one of:

- A byte-level capture (raw hex) from real hardware showing the frame,
  ideally with enough context to show how you determined the field
  boundaries.
- A reference to the source you cross-checked against (e.g. the ESPHome
  C++ component, or `mr60bha2-proto`), with the relevant lines/commit
  quoted or linked.

Unverified additions are welcome as draft PRs for discussion, but won't be
merged without one of the above.

## Commit / PR style

- Keep PRs focused on one change.
- Write commit messages that explain *why*, not just *what*.
- Add or update tests for any behavior change.

## Reporting bugs

See the [bug report template](.github/ISSUE_TEMPLATE/bug_report.md) —
opening an issue will offer it automatically.
