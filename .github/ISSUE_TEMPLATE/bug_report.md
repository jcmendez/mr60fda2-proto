---
name: Bug report
about: Report a parsing/encoding bug or incorrect behavior
title: ""
labels: bug
assignees: ""
---

## Description

A clear description of what's wrong.

## Environment

- Crate version:
- Rust toolchain (`rustc --version`):
- Target (`no_std` target, or host with `std` feature):
- Hardware (if applicable, e.g. XIAO ESP32-C6 + MR60FDA2):

## Steps to reproduce

Minimal code to reproduce, e.g.:

```rust
let mut parser = FrameParser::new();
for byte in [/* raw bytes */] {
    let event = parser.feed(byte);
    // ...
}
```

## Raw bytes (if this is a parsing bug)

If the bug involves parsing a frame from real hardware, please include the
raw byte capture in hex, e.g.:

```
53 00 01 0F 09 ...
```

This is the most useful thing you can give us — without an official spec,
we verify parsing correctness against real captures.

## Expected behavior

What you expected to happen.

## Actual behavior

What actually happened (including any panic message, incorrect
`ParseEvent`, checksum mismatch, etc.).
