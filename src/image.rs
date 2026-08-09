//! The provenance-hashed deployable fill-map — the artifact a dispenser
//! actually executes, reusing the flash-image discipline proven on silicon
//! in the fast-charge and hearing-fitting programs.
//!
//! Layout (2436 bytes, little-endian):
//!
//! | offset | bytes | field |
//! |---|---|---|
//! | 0 | 4 | magic `"QCH2"` = `0x3248_4351` |
//! | 4 | 4 | format version |
//! | 8 | 8 | dispenser serial |
//! | 16 | 8 | tank-parameters + rulebook hash |
//! | 24 | 8 | fill-map fingerprint |
//! | 32 | 2400 | the map: one action byte per state |
//! | 2432 | 4 | CRC32 over bytes 0..2432 |
//!
//! Validation is **fail-closed** and ordered: magic → version → CRC →
//! fingerprint, then (device side) the provisioned tank-hash comparison. A
//! map solved for a different tank, or under a revised rulebook, is refused
//! before a single kilogram is dispensed — the between-session re-solve
//! cadence made mechanical, which is also what keeps the patent posture
//! honest (no in-fill adaptation; the *image* is the unit of change).

use crate::fill_env::{
    ACTIONS, ALPHA, BETA, COP_PRECOOL, FLOW_TIERS, N_STATES, PRECOOL_C, TARGET_BAND,
};
use crate::thermo::{
    TankParams, B_COVOL, LINER_RAMP_CAP, P_CEILING, T_GAS_CEILING_C, T_LINER_CEILING_C,
};

pub const MAGIC: u32 = 0x3248_4351; // "QCH2" little-endian
pub const VERSION: u32 = 1;
pub const TABLE_LEN: usize = N_STATES; // 2400 action bytes
pub const HEADER_LEN: usize = 32;
pub const IMAGE_LEN: usize = HEADER_LEN + TABLE_LEN + 4; // 2436

pub fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

fn mix(h: u64, v: u64) -> u64 {
    (h.rotate_left(7) ^ v).wrapping_mul(0xBF58_476D_1CE4_E5B9)
}

/// Hash the tank parameters AND the full rulebook: the provenance binding
/// that makes a stale map (re-identified tank OR revised limits OR
/// re-declared objective weights) detectable before it is trusted.
pub fn tank_hash(p: &TankParams) -> u64 {
    let mut h: u64 = 0x9E37_79B9_7F4A_7C15;
    for v in [
        p.volume_m3,
        p.ua_gas_liner,
        p.ua_liner_amb,
        p.c_liner,
        p.t_amb_c,
        B_COVOL,
        T_GAS_CEILING_C,
        T_LINER_CEILING_C,
        P_CEILING,
        LINER_RAMP_CAP,
        COP_PRECOOL,
        ALPHA,
        BETA,
    ] {
        h = mix(h, v.to_bits());
    }
    for v in FLOW_TIERS {
        h = mix(h, v.to_bits());
    }
    for v in PRECOOL_C {
        h = mix(h, v.to_bits());
    }
    mix(h, TARGET_BAND as u64)
}

pub fn fingerprint(table: &[u8]) -> u64 {
    let mut h: u64 = 0x9E37_79B9_7F4A_7C15;
    for &b in table {
        h = mix(h, b as u64);
    }
    h
}

/// Pack a trained policy into the deployable map. States with no gate-clean
/// action are written as the declared SAFE FALLBACK (`action 0` = lowest
/// mass-flow tier at the coldest pre-cool setpoint — minimum enthalpy
/// injection), so the dispenser has a defined command for every state it can
/// possibly index.
pub fn table_from_policy(actions: &dyn Fn(usize) -> usize) -> Vec<u8> {
    (0..N_STATES)
        .map(|s| {
            let a = actions(s);
            if a < ACTIONS {
                a as u8
            } else {
                0
            }
        })
        .collect()
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageError {
    BadMagic,
    BadVersion,
    BadCrc,
    FingerprintMismatch,
    /// The map's provenance hash is not what this dispenser was provisioned
    /// to expect — wrong tank model, or a stale rulebook.
    StaleProvenance,
}

pub fn build(serial: u64, thash: u64, table: &[u8]) -> Vec<u8> {
    assert_eq!(table.len(), TABLE_LEN);
    let mut img = vec![0u8; IMAGE_LEN];
    img[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    img[4..8].copy_from_slice(&VERSION.to_le_bytes());
    img[8..16].copy_from_slice(&serial.to_le_bytes());
    img[16..24].copy_from_slice(&thash.to_le_bytes());
    img[24..32].copy_from_slice(&fingerprint(table).to_le_bytes());
    img[HEADER_LEN..HEADER_LEN + TABLE_LEN].copy_from_slice(table);
    let crc = crc32(&img[..IMAGE_LEN - 4]);
    img[IMAGE_LEN - 4..].copy_from_slice(&crc.to_le_bytes());
    img
}

#[derive(Debug)]
pub struct ValidImage {
    pub serial: u64,
    pub tank_hash: u64,
    pub table: Vec<u8>,
}

pub fn validate(img: &[u8]) -> Result<ValidImage, ImageError> {
    if img.len() < IMAGE_LEN {
        return Err(ImageError::BadMagic);
    }
    let u32_at = |i: usize| u32::from_le_bytes(img[i..i + 4].try_into().unwrap());
    let u64_at = |i: usize| u64::from_le_bytes(img[i..i + 8].try_into().unwrap());
    if u32_at(0) != MAGIC {
        return Err(ImageError::BadMagic);
    }
    if u32_at(4) != VERSION {
        return Err(ImageError::BadVersion);
    }
    if crc32(&img[..IMAGE_LEN - 4]) != u32_at(IMAGE_LEN - 4) {
        return Err(ImageError::BadCrc);
    }
    let table = img[HEADER_LEN..HEADER_LEN + TABLE_LEN].to_vec();
    if fingerprint(&table) != u64_at(24) {
        return Err(ImageError::FingerprintMismatch);
    }
    Ok(ValidImage { serial: u64_at(8), tank_hash: u64_at(16), table })
}

/// Full dispenser-side acceptance: structural validation PLUS the
/// provisioned provenance expectation.
pub fn accept(img: &[u8], expected_tank_hash: u64) -> Result<ValidImage, ImageError> {
    let v = validate(img)?;
    if v.tank_hash != expected_tank_hash {
        return Err(ImageError::StaleProvenance);
    }
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain_engine;
    use crate::fill_env::FillEnv;
    use crate::thermo::RESIDUAL_SOC;

    #[test]
    fn crc32_known_vector() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn magic_is_the_declared_ascii() {
        assert_eq!(&MAGIC.to_le_bytes(), b"QCH2");
    }

    #[test]
    fn image_roundtrip_and_fail_closed() {
        let p = TankParams::nominal(25.0);
        let env = FillEnv::new(&p, RESIDUAL_SOC);
        let (policy, _) = domain_engine().train(&env);
        let table = table_from_policy(&|s| policy.action(s));
        let thash = tank_hash(&p);
        let img = build(0xD152_0042, thash, &table);
        let v = validate(&img).expect("valid");
        assert_eq!(v.serial, 0xD152_0042);
        assert_eq!(v.tank_hash, thash);
        assert_eq!(v.table, table);

        // Flip a map byte → CRC refuses; forge the CRC → fingerprint refuses.
        let mut bad = img.clone();
        bad[HEADER_LEN + 137] ^= 1;
        assert_eq!(validate(&bad).unwrap_err(), ImageError::BadCrc);
        let crc = crc32(&bad[..IMAGE_LEN - 4]);
        bad[IMAGE_LEN - 4..].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(validate(&bad).unwrap_err(), ImageError::FingerprintMismatch);

        // Corrupt magic / version.
        let mut m = img.clone();
        m[0] ^= 0xFF;
        assert_eq!(validate(&m).unwrap_err(), ImageError::BadMagic);
        let mut vsn = img.clone();
        vsn[4] = 9;
        assert_eq!(validate(&vsn).unwrap_err(), ImageError::BadVersion);
        assert_eq!(validate(&img[..IMAGE_LEN - 1]).unwrap_err(), ImageError::BadMagic);

        // A different tank model changes the provenance hash: the
        // stale-map detection that anchors the re-solve cadence.
        let p2 = TankParams::nominal(35.0);
        assert_ne!(tank_hash(&p2), thash);
        assert_eq!(accept(&img, tank_hash(&p2)).unwrap_err(), ImageError::StaleProvenance);
        assert!(accept(&img, thash).is_ok());
    }
}
