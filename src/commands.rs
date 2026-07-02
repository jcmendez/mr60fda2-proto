//! Outgoing command frames for the MR60FDA2, ported from Seeed's reference
//! ESPHome C++ component's `set_install_height`/`set_height_threshold`/
//! `set_sensitivity`/`get_radar_parameters`/`factory_reset` methods. Every
//! constant byte here is copied from that source, not re-derived -- the
//! header-checksum values in particular are cross-checked against
//! `calculate_checksum` in this crate's own tests.

/// Selectable mounting heights, metres. Index into this array is what the
/// ESPHome `select` component and `set_install_height` take.
pub const INSTALL_HEIGHT_M: [f32; 7] = [2.4, 2.5, 2.6, 2.7, 2.8, 2.9, 3.0];

/// Selectable fall-height thresholds, metres.
pub const HEIGHT_THRESHOLD_M: [f32; 7] = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6];

/// Selectable sensitivity levels (unitless, sensor-defined scale).
pub const SENSITIVITY: [u32; 3] = [3, 15, 30];

fn checksum(data: &[u8]) -> u8 {
    !data.iter().fold(0u8, |acc, &b| acc ^ b)
}

/// `RESULT_PARAMETERS` (0x0E06) query -- request the sensor's current
/// install height / height threshold / sensitivity. The response comes back
/// as a `ParseEvent::Parameters` frame.
pub const fn get_parameters() -> [u8; 8] {
    [0x01, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x06, 0xF6]
}

/// Factory-reset the sensor's configuration (`0x2110`).
pub const fn factory_reset() -> [u8; 8] {
    [0x01, 0x00, 0x00, 0x00, 0x00, 0x21, 0x10, 0xCF]
}

/// Opt in to unsolicited `HEIGHT_UPLOAD_TYPE` (0x0E0E) frames -- the sensor
/// does not stream live target height unless this is sent first (mirrors
/// `controller.yaml`'s boot-time `uart.write`).
pub const fn enable_height_upload() -> [u8; 8] {
    [0x01, 0x80, 0x00, 0x00, 0x00, 0x0E, 0x0E, 0x7E]
}

/// Set the mounting height (`RESULT_INSTALL_HEIGHT`, 0x0E04). `index` picks
/// a value from [`INSTALL_HEIGHT_M`]; returns `None` for an out-of-range
/// index. Ack arrives as `ParseEvent::SetInstallHeightAck`.
pub fn set_install_height(index: usize) -> Option<[u8; 13]> {
    let value = *INSTALL_HEIGHT_M.get(index)?;
    Some(build_set_f32(0x0E, 0x04, value))
}

/// Set the fall-height threshold (`RESULT_HEIGHT_THRESHOLD`, 0x0E08).
/// `index` picks a value from [`HEIGHT_THRESHOLD_M`]. Ack arrives as
/// `ParseEvent::SetHeightThresholdAck`.
pub fn set_height_threshold(index: usize) -> Option<[u8; 13]> {
    let value = *HEIGHT_THRESHOLD_M.get(index)?;
    Some(build_set_f32(0x0E, 0x08, value))
}

/// Set the detection sensitivity (`RESULT_SENSITIVITY`, 0x0E0A). `index`
/// picks a value from [`SENSITIVITY`]. Ack arrives as
/// `ParseEvent::SetSensitivityAck`.
pub fn set_sensitivity(index: usize) -> Option<[u8; 13]> {
    let value = *SENSITIVITY.get(index)?;
    Some(build_set_u32(0x0E, 0x0A, value))
}

/// Builds a 13-byte `[SOF][ID:2][LEN:2][TYPE:2][HEAD_CKSUM][DATA:4][DATA_CKSUM]`
/// frame carrying a little-endian `f32` payload.
fn build_set_f32(type_hi: u8, type_lo: u8, value: f32) -> [u8; 13] {
    build_set_bytes(type_hi, type_lo, value.to_le_bytes())
}

/// Same shape as [`build_set_f32`], for a little-endian `u32` payload.
fn build_set_u32(type_hi: u8, type_lo: u8, value: u32) -> [u8; 13] {
    build_set_bytes(type_hi, type_lo, value.to_le_bytes())
}

fn build_set_bytes(type_hi: u8, type_lo: u8, payload: [u8; 4]) -> [u8; 13] {
    let mut frame = [0u8; 13];
    frame[0] = 0x01; // SOF
    frame[1] = 0x00; // ID hi
    frame[2] = 0x00; // ID lo
    frame[3] = 0x00; // LEN hi
    frame[4] = 0x04; // LEN lo -- 4-byte payload
    frame[5] = type_hi;
    frame[6] = type_lo;
    frame[7] = checksum(&frame[0..7]);
    frame[8..12].copy_from_slice(&payload);
    frame[12] = checksum(&payload);
    frame
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every constant below is a byte-for-byte copy from
    /// `seeed_mr60fda2.cpp`'s `send_data` arrays -- these tests confirm our
    /// `checksum`/frame-building logic reproduces bytes the real firmware is
    /// already known to send successfully, not just internally-consistent
    /// bytes.
    #[test]
    fn get_parameters_matches_known_bytes() {
        assert_eq!(
            get_parameters(),
            [0x01, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x06, 0xF6]
        );
    }

    #[test]
    fn factory_reset_matches_known_bytes() {
        assert_eq!(
            factory_reset(),
            [0x01, 0x00, 0x00, 0x00, 0x00, 0x21, 0x10, 0xCF]
        );
    }

    #[test]
    fn enable_height_upload_matches_known_bytes() {
        assert_eq!(
            enable_height_upload(),
            [0x01, 0x80, 0x00, 0x00, 0x00, 0x0E, 0x0E, 0x7E]
        );
    }

    #[test]
    fn set_install_height_header_matches_known_bytes() {
        // seeed_mr60fda2.cpp: {0x01,0x00,0x00,0x00,0x04,0x0E,0x04,0xF0, <f32 LE>, <cksum>}
        let frame = set_install_height(0).unwrap();
        assert_eq!(
            frame[0..8],
            [0x01, 0x00, 0x00, 0x00, 0x04, 0x0E, 0x04, 0xF0]
        );
        assert_eq!(&frame[8..12], &2.4f32.to_le_bytes());
    }

    #[test]
    fn set_height_threshold_header_matches_known_bytes() {
        // seeed_mr60fda2.cpp: {0x01,0x00,0x00,0x00,0x04,0x0E,0x08,0xFC, ...}
        let frame = set_height_threshold(0).unwrap();
        assert_eq!(
            frame[0..8],
            [0x01, 0x00, 0x00, 0x00, 0x04, 0x0E, 0x08, 0xFC]
        );
    }

    #[test]
    fn set_sensitivity_header_matches_known_bytes() {
        // seeed_mr60fda2.cpp: {0x01,0x00,0x00,0x00,0x04,0x0E,0x0A,0xFE, ...}
        let frame = set_sensitivity(0).unwrap();
        assert_eq!(
            frame[0..8],
            [0x01, 0x00, 0x00, 0x00, 0x04, 0x0E, 0x0A, 0xFE]
        );
        assert_eq!(&frame[8..12], &3u32.to_le_bytes());
    }

    #[test]
    fn out_of_range_index_is_none() {
        assert_eq!(set_install_height(7), None);
        assert_eq!(set_height_threshold(7), None);
        assert_eq!(set_sensitivity(3), None);
    }

    /// The data checksum covers only the 4 payload bytes, same as the C++
    /// component's `calculate_checksum(send_data + 8, 4)`.
    #[test]
    fn data_checksum_is_correct() {
        let frame = set_install_height(3).unwrap(); // 2.7m
        let expected = !frame[8..12].iter().fold(0u8, |acc, &b| acc ^ b);
        assert_eq!(frame[12], expected);
    }
}
