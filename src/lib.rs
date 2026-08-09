//! [QCEOM RL] Hydrogen fast-fill governor (R3).
//!
//! Governed exact-DP fill policies for 700 bar heavy-duty hydrogen fuelling,
//! solved offline against a declared real-gas + lumped-wall tank model and
//! benchmarked against the SAE J2601-style lookup-table (fixed APRR + T40)
//! incumbent and a greedy max-flow-until-hot station controller.
//!
//! **Patent posture (PATENT-LANDSCAPE.md).** The deployed artifact is a
//! *static governed fill-map*: a precomputed
//! (SoC band × gas-temp band × liner-temp band) → (mass-flow tier, pre-cool
//! setpoint) lookup table, read at band entry and held for the crossing.
//! There is no in-fill adaptive feedback loop, no online thermal observer,
//! and no hardware claim. Adaptation happens *between* sessions:
//! re-characterize → re-solve → re-fingerprint → re-burn.
//!
//! **Standards posture.** J2601 compliance is a certification question, not
//! a patent question, and is never self-awarded. This harness produces a
//! station-side map under a declared model and a declared rulebook.

pub mod fill_env;
pub mod image;
pub mod incumbent;
pub mod thermo;

use qceom_core::{EngineConfig, MathematicalRLEngine};

/// Kernel defaults with two DECLARED domain deviations, both mandated by the
/// factory's lessons registry and both verified by the plain-DP probe:
///
/// - **γ = 0.9999 (L10).** The fill is a SoC-layered DAG of 15 decisions, so
///   the correct horizon is undiscounted. γ = 1.0 collapses the kernel (its
///   solve machinery assumes contraction), and γ = 0.95 measurably shades
///   the late, pressure-critical bands. 0.9999¹⁵ = 0.9985 is
///   indistinguishable from the undiscounted reference at this horizon.
/// - **shaping_weight = 0.0 (L9).** The PME shaping term is policy-invariant
///   in exact arithmetic but not through finite-tolerance value iteration,
///   and this domain's per-decision rewards are small (fractions of a
///   minute-equivalent). With shaping off, the kernel reproduces the
///   plain-DP optimum EXACTLY — which `fill_env::kernel_matches_the_plain_dp_reference`
///   enforces as a standing test rather than a one-off probe.
pub fn domain_engine() -> MathematicalRLEngine {
    MathematicalRLEngine::new(EngineConfig {
        gamma: 0.9999,
        shaping_weight: 0.0,
        ..EngineConfig::default()
    })
}
