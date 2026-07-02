# mr60fda2-proto

A `no_std`, zero-allocation Rust parser and command builder for the
[Seeed Studio MR60FDA2](https://wiki.seeedstudio.com/led_60ghz_mmwave_fall_detection_radar/)
60 GHz mmWave fall-detection radar's UART protocol.

Verified against real hardware: see [`Verified on hardware`](#verified-on-hardware) below.

## Why this exists

Seeed doesn't publish a Rust driver for the MR60FDA2 (or any of their 60 GHz
mmWave radar family). The reference implementation is a C++
[ESPHome](https://esphome.io/) external component. This crate is an
independent Rust implementation of the same wire protocol, written for use
in bare-metal `no_std` firmware (originally: an `esp-hal`/embassy-based
ESP32-C6 firmware).

Seeed's whole MR60 60 GHz radar family — this sensor (fall detection) and
the sibling [MR60BHA2](https://wiki.seeedstudio.com/mmwave_radar_kit/)
(vitals/presence) — shares one wire format: same SOF byte, same big-endian
`LEN`/`TYPE` header, same `~XOR` checksum algorithm. That's confirmed by
comparing this crate against
[`mr60bha2-proto`](https://crates.io/crates/mr60bha2-proto), an independent
parser for the BHA2: the frame state machine here is modeled closely on
that crate's, and both decode frame type `0x0F09` identically as a
human-presence boolean. Only the FDA2-specific type codes and payload
layouts (fall detection, install height, height threshold, sensitivity,
live target height) are new.

## What's included

- `FrameParser` — a byte-at-a-time streaming state machine that validates
  both header and data checksums and emits a `ParseEvent` per complete
  frame. No heap allocation, fixed-size internal buffers.
- Command builders for every documented outgoing frame: `get_parameters`,
  `factory_reset`, `enable_height_upload`, `set_install_height`,
  `set_height_threshold`, `set_sensitivity`.

Not included: the UART peripheral driver itself (that's necessarily
platform-specific — see `examples/` for the shape of an `embedded-io-async`
integration).

## Frame format

```text
[SOF:1] [ID:2] [LEN:2] [TYPE:2] [HEAD_CKSUM:1] [DATA:LEN] [DATA_CKSUM:1]
```

`LEN` and `TYPE` are big-endian. Data-section fields (floats, u32s) are
little-endian. Checksum: `~(XOR of covered bytes) & 0xFF`.

## Usage

```rust
use mr60fda2_proto::{FrameParser, ParseEvent, get_parameters};

let mut parser = FrameParser::new();

// Send `get_parameters()` over your UART, then feed received bytes in:
for byte in received_bytes {
    if let Some(event) = parser.feed(byte) {
        match event {
            ParseEvent::Fall(detected) => { /* ... */ }
            ParseEvent::PeopleExist(present) => { /* ... */ }
            ParseEvent::TargetHeight(metres) => { /* ... */ }
            ParseEvent::Parameters { install_height_m, height_threshold_m, sensitivity } => { /* ... */ }
            _ => {}
        }
    }
}
```

`no_std` by default; enable the `std` feature (on by default when used as a
dev-dependency) if you want it for host-side tooling.

## Verified on hardware

Round-tripped against a real MR60FDA2 (Seeed fall-detection kit, XIAO
ESP32-C6) over UART: `get_parameters()` sent, sensor replied with a real
`ParseEvent::Parameters` frame; continuous `Fall`/`PeopleExist` frames
received and parsed with zero unrecognized frame types.

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
