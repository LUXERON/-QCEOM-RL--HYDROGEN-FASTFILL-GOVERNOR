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
    ACTIONS, ALPHA, BETA, COP_PRECOOL, FLOW_TIERS, GAS_BANDS, GAS_BAND_C, GAS_BASE_C, LIN_BANDS,
    LIN_BAND_C, LIN_BASE_C, N_STATES, PRECOOL_C, SOC_BANDS, TARGET_BAND,
};
use crate::thermo::{
    TankParams, B_COVOL, CP0, CV0, DT_S, LINER_RAMP_CAP, M_H2, P_CEILING, P_NWP, RESIDUAL_SOC, R_U,
    T_GAS_CEILING_C, T_LINER_CEILING_C, T_REF_SOC_K,
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

/// Hash the tank parameters, the full rulebook AND the codec: the
/// provenance binding that makes a stale map (re-identified tank OR
/// revised limits OR re-declared objective weights OR a re-based band
/// grid) detectable before it is trusted.
///
/// This harness already hashed its **action** tiers, which the
/// 2026-08-09 estate-wide audit called the best-covered case in the
/// estate. It did not hash the **state** band lattice. The deployed
/// image is a bare action-index byte per state, so the state decode is
/// as load-bearing as the action decode: re-basing `GAS_BASE_C` or
/// widening `LIN_BAND_C` while keeping the band counts yields a
/// same-length image with an unchanged hash that misindexes every
/// lookup, and magic → version → CRC → fingerprint all pass it. The
/// band bases and widths are now hashed alongside the counts.
pub fn tank_hash(p: &TankParams) -> u64 {
    let mut h: u64 = 0x9E37_79B9_7F4A_7C15;
    for v in [
        p.volume_m3,
        p.ua_gas_liner,
        p.ua_liner_amb,
        p.c_liner,
        p.t_amb_c,
        B_COVOL,
        // EQUATION OF STATE + CALORIC MODEL (added 2026-08-09 by the L30
        // omission guard). These were outside the hash and every one of
        // them re-solves the map:
        //
        //   R_U, T_REF_SOC_K, P_NWP  define `n_full()` — i.e. what "100 %
        //     SoC" MEANS. Revising any of them moves the target itself,
        //     so the map would be aiming at a different fill.
        //   M_H2                     mass and inlet enthalpy.
        //   CV0                      the gas energy balance AND the
        //     τ_gas the L12 step was chosen against.
        //   CP0                      inlet enthalpy and the pre-cool
        //     energy that the objective prices.
        //   RESIDUAL_SOC             the START state the map is solved
        //     from (`FillEnv::new(&p, RESIDUAL_SOC)`).
        //   DT_S                     the integration step every cell is
        //     characterized at.
        //
        // None is subsumed by a hashed value, so none is exempt. That a
        // constant is a constant of NATURE (R_U, M_H2) is not a reason to
        // leave it out: the hash binds the map to the model it was solved
        // under, not to that model's plausibility.
        R_U,
        M_H2,
        CV0,
        CP0,
        P_NWP,
        T_REF_SOC_K,
        RESIDUAL_SOC,
        DT_S,
        T_GAS_CEILING_C,
        T_LINER_CEILING_C,
        P_CEILING,
        LINER_RAMP_CAP,
        COP_PRECOOL,
        ALPHA,
        BETA,
        // CODEC: the state band lattice — base and width per dimension,
        // not just the counts below. SoC bands are uniform fractions of
        // capacity, so `SOC_BANDS` alone defines that axis.
        GAS_BASE_C,
        GAS_BAND_C,
        LIN_BASE_C,
        LIN_BAND_C,
    ] {
        h = mix(h, v.to_bits());
    }
    // CODEC: the action tier tables
    for v in FLOW_TIERS {
        h = mix(h, v.to_bits());
    }
    for v in PRECOOL_C {
        h = mix(h, v.to_bits());
    }
    // CODEC: the lattice shape
    for n in [SOC_BANDS, GAS_BANDS, LIN_BANDS, N_STATES, ACTIONS, TARGET_BAND] {
        h = mix(h, n as u64);
    }
    h
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

    /// **Codec-coverage regression (estate-wide finding, 2026-08-09).**
    /// The image is a bare action-index byte per state, so the STATE
    /// decode is as load-bearing as the action decode. Re-basing a band
    /// grid at constant band count would produce a same-length image with
    /// an unchanged hash that misindexes every lookup.
    ///
    /// The lattice constants cannot be perturbed at runtime, so the hash
    /// is pinned: any edit to `GAS_BASE_C`, `GAS_BAND_C`, `LIN_BASE_C`,
    /// `LIN_BAND_C`, the band counts, `TARGET_BAND`, or either action
    /// tier table fails here and forces a deliberate re-emission of the
    /// golden vector rather than a silent same-hash image.
    /// **2026-08-09, second pass.** This pin was eight constants short and
    /// shipped that way. The equation-of-state and caloric group — `R_U`,
    /// `M_H2`, `CV0`, `CP0`, `P_NWP`, `T_REF_SOC_K`, `RESIDUAL_SOC`,
    /// `DT_S` — sat outside the hash, and three of them (`R_U`, `P_NWP`,
    /// `T_REF_SOC_K`) jointly define `n_full()`, i.e. what "100 % SoC"
    /// MEANS. Revising them moved the fill target itself under a
    /// byte-identical hash. Found by
    /// [`every_declared_model_constant_is_hashed`], not by this pin —
    /// which is the point of having both.
    /// Pin moved `0x0723da1ccdc8bb94` → `0x6f9f25ac945c4600`.
    #[test]
    fn tank_hash_is_pinned_to_the_rulebook_and_the_codec() {
        assert_eq!(tank_hash(&TankParams::nominal(25.0)), 0x6f9f_25ac_945c_4600);
        // The pre-fix value must be unreachable.
        assert_ne!(tank_hash(&TankParams::nominal(25.0)), 0x0723_da1c_cdc8_bb94);
    }

    /// **The omission guard (L30, ported from R6 Czochralski 2026-08-09).**
    ///
    /// A pinned hash literal and this test fail on different things, and
    /// that distinction is the whole point. The pin above catches a
    /// *change* to a constant the hash ALREADY covers. It is structurally
    /// blind to a constant the hash NEVER covered — a pin cannot notice an
    /// absence. R6 shipped a hash that omitted its entire heat-transport
    /// group with a green pinned test; revising one thermal conductivity
    /// re-solved 61.8 % of the deployed map under a byte-identical hash,
    /// and every fielded image would have been served.
    ///
    /// So this checks COVERAGE, against the source rather than against a
    /// hand-maintained list — because a hand-maintained list is exactly
    /// what was one group short. Both files are pulled in with
    /// `include_str!`, so it is a compile-time string comparison: no I/O,
    /// no ordering assumptions, nothing to keep in sync. The hash body is
    /// extracted by brace matching and its comments are stripped, so
    /// naming a constant in prose does not satisfy the check — it has to
    /// be mixed in.
    ///
    /// Adding a `pub const` to `thermo.rs` and forgetting the hash now
    /// fails here, naming the constant. Exempting one requires editing
    /// `SUBSUMED` and writing down why.
    #[test]
    fn every_declared_model_constant_is_hashed() {
        const MODEL_SRC: &str = include_str!("thermo.rs");
        const HASH_SRC: &str = include_str!("image.rs");

        /// Constants ALGEBRAICALLY SUBSUMED by something the hash already
        /// mixes, and therefore not independently required.
        ///
        /// Empty for this harness: every declared constant in `thermo.rs`
        /// reaches the model in its own right, so every one is hashed.
        /// The bar for this list is narrow and is not "unlikely to be
        /// revised" — it is *cannot move the map without moving a hashed
        /// value*. A constant of nature that the EOS divides by still
        /// belongs in the hash, because the hash binds the map to the
        /// model it was solved under, not to the model's plausibility.
        const SUBSUMED: [&str; 0] = [];

        let start = HASH_SRC
            .find("pub fn tank_hash(")
            .expect("tank_hash must be declared in this file");
        let rest = &HASH_SRC[start..];
        let open = rest.find('{').expect("tank_hash must have a body");
        let b = rest.as_bytes();
        let mut depth = 0usize;
        let mut end = open;
        for (i, &ch) in b.iter().enumerate().skip(open) {
            match ch {
                b'{' => depth += 1,
                b'}' => {
                    depth -= 1;
                    if depth == 0 {
                        end = i;
                        break;
                    }
                }
                _ => {}
            }
        }
        assert!(end > open, "tank_hash body must be brace-balanced");
        // Strip line comments: a constant NAMED in a comment must not
        // count as a constant HASHED.
        let body: String = rest[open..=end]
            .lines()
            .map(|l| match l.find("//") {
                Some(i) => &l[..i],
                None => l,
            })
            .collect::<Vec<_>>()
            .join("\n");

        // Whole-identifier match, not a substring match.
        let mentions = |hay: &str, name: &str| {
            let n = name.as_bytes();
            hay.as_bytes().windows(n.len()).enumerate().any(|(i, w)| {
                if w != n {
                    return false;
                }
                let before = if i == 0 { b' ' } else { hay.as_bytes()[i - 1] };
                let after = *hay.as_bytes().get(i + n.len()).unwrap_or(&b' ');
                let ident = |c: u8| c.is_ascii_alphanumeric() || c == b'_';
                !ident(before) && !ident(after)
            })
        };

        let mut checked = 0usize;
        let mut missing: Vec<&str> = Vec::new();
        for line in MODEL_SRC.lines() {
            let l = line.trim_start();
            let Some(r) = l.strip_prefix("pub const ") else {
                continue;
            };
            let name = r
                .split(|c: char| c == ':' || c.is_whitespace())
                .next()
                .expect("a const declaration has a name");
            // `pub const fn ...` is a function, not a constant.
            if name.is_empty() || name == "fn" || SUBSUMED.contains(&name) {
                continue;
            }
            checked += 1;
            if !mentions(&body, name) {
                missing.push(name);
            }
        }

        assert!(
            checked >= 10,
            "the scanner found only {checked} constants in thermo.rs — it has \
             stopped parsing the file and is no longer guarding anything"
        );
        assert!(
            missing.is_empty(),
            "these constants are declared in thermo.rs but are NOT mixed into \
             tank_hash, so revising any of them re-solves the map while every \
             fielded image keeps validating — the R6 silent mis-serve, \
             exactly: {missing:?}"
        );
    }
}
