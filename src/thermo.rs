//! Hydrogen fast-fill thermodynamics: a DECLARED lumped model of a 700 bar
//! Type-IV composite tank being filled from a pre-cooled dispenser.
//!
//! # What is declared (and what is NOT claimed)
//!
//! **Real gas.** Hydrogen's compressibility deviates *upward* from ideal at
//! fuelling pressures (Z ≈ 1.5 at 700 bar / 300 K). We declare the simplest
//! form that captures it,
//!
//! ```text
//!     Z(P, T) = 1 + b·P / (R·T),      b = 1.9e-5 m³/mol
//! ```
//!
//! which rearranges **exactly** into a covolume equation of state:
//!
//! ```text
//!     P = n·R·T / (V − n·b)
//! ```
//!
//! This is a DECLARED SIMPLIFICATION of NIST REFPROP, **not a claim of
//! REFPROP fidelity**. It is chosen because (a) one constant reproduces the
//! headline deviation to within a few percent over the fuelling envelope
//! (at 700 bar / 288 K it gives 37.9 kg/m³ against REFPROP's ≈ 39 kg/m³),
//! and (b) it has an exact and unusually convenient caloric consequence:
//!
//! ```text
//!     (∂u/∂V)_T = T·(∂P/∂T)_V − P = T·nR/(V−nb) − P = 0
//! ```
//!
//! so the internal energy of a covolume gas depends **only** on temperature,
//! `u = c_v0·T`, with no departure function. Every departure that matters is
//! therefore pushed into the *inlet enthalpy*, where it belongs physically
//! (hydrogen's negative Joule–Thomson coefficient above ~200 K is why fills
//! get hot):
//!
//! ```text
//!     h_in = c_p0·T_in + b·P_tank / M
//! ```
//!
//! derived by evaluating `h = u + P·v` for the inlet stream throttled to the
//! current tank pressure (`V_m,in = R·T_in/P + b`, so `R·T·V_m/(V_m−b)`
//! collapses to `R·T_in + b·P`). At 700 bar that departure term is ~660 kJ/kg
//! — 20 % of the inlet enthalpy — so dropping it would have understated the
//! fill's heating by a factor of two. Declared, derived, and kept.
//!
//! **Lumped energy balance.** Gas control volume with mass inflow, plus a
//! single wall node ("liner", meaning liner + the thermally participating
//! fraction of the composite overwrap):
//!
//! ```text
//!     m·c_v·dT_gas/dt   = ṁ·(h_in − u_gas) − UA·(T_gas − T_liner)
//!     C_liner·dT_liner/dt = UA·(T_gas − T_liner) − UA_amb·(T_liner − T_amb)
//! ```
//!
//! **The Type-IV composite has HIGH thermal resistance to ambient, and that
//! is the entire point of the harness.** `UA_amb` is ~40× smaller than
//! `UA`: heat pushed into the wall during a fill does not leave on the
//! timescale of a fill (τ_wall→amb ≈ 3 h). The wall temperature is therefore
//! an accumulating, shared, history-dependent resource — which is exactly
//! what a reactive controller cannot price and dynamic programming can.
//!
//! Parameters here are representative of a 350 L heavy-duty module; per-tank
//! identification replaces them through the same struct.

/// Universal gas constant, J/(mol·K).
pub const R_U: f64 = 8.314_462_618;
/// Molar mass of H₂, kg/mol.
pub const M_H2: f64 = 2.015_88e-3;
/// DECLARED hydrogen covolume, m³/mol (the single real-gas parameter).
pub const B_COVOL: f64 = 1.9e-5;
/// Ideal-gas specific heats of H₂, J/(kg·K) (c_p − c_v = R/M ≈ 4124).
pub const CV0: f64 = 10_180.0;
pub const CP0: f64 = 14_300.0;

/// Nominal working pressure, Pa.
pub const P_NWP: f64 = 700e5;
/// SAE reference for 100 % SoC: NWP density at 15 °C.
pub const T_REF_SOC_K: f64 = 288.15;

/// Gas transient ceiling (SAE J2601 receptacle/tank limit), °C.
pub const T_GAS_CEILING_C: f64 = 85.0;
/// Pressure ceiling: 125 % of NWP, Pa.
pub const P_CEILING: f64 = 875e5;
/// DECLARED sustained liner/resin service ceiling, °C. Lower than the gas
/// transient ceiling because the wall's exposure is sustained, not transient.
pub const T_LINER_CEILING_C: f64 = 75.0;
/// DECLARED liner thermal-shock cap on dT_liner/dt, K/s.
pub const LINER_RAMP_CAP: f64 = 0.45;

/// Declared minimum residual state of charge (≈ 20 bar back-pressure).
pub const RESIDUAL_SOC: f64 = 0.05;

/// Integration step, s. See [`l12_time_constant_check`] — this is the
/// L12-mandated choice, not a convenience.
pub const DT_S: f64 = 0.25;

#[derive(Debug, Clone, Copy)]
pub struct TankParams {
    /// Internal volume, m³.
    pub volume_m3: f64,
    /// Gas ↔ liner conductance, W/K.
    pub ua_gas_liner: f64,
    /// Liner ↔ ambient conductance, W/K (small: composite is an insulator).
    pub ua_liner_amb: f64,
    /// Wall lumped heat capacity, J/K.
    pub c_liner: f64,
    /// Ambient temperature, °C.
    pub t_amb_c: f64,
}

impl TankParams {
    /// A 350 L heavy-duty Type-IV module at the declared nominal ambient.
    ///
    /// `c_liner = 1.3e5 J/K` ≈ 15 kg HDPE liner (c ≈ 1900 J/kg·K) plus the
    /// ~100 kg of carbon/epoxy overwrap (c ≈ 1000 J/kg·K) that is thermally
    /// penetrated on a fill timescale. `ua_gas_liner = 500 W/K` ≈ 3.3 m²
    /// internal area at h ≈ 150 W/(m²·K), the mid-range of published
    /// in-tank filling correlations. `ua_liner_amb = 12 W/K` is the
    /// composite's outward resistance — τ ≈ 3 h.
    pub fn nominal(t_amb_c: f64) -> Self {
        Self {
            volume_m3: 0.35,
            ua_gas_liner: 500.0,
            ua_liner_amb: 12.0,
            c_liner: 1.3e5,
            t_amb_c,
        }
    }

    /// Moles at 100 % SoC: NWP density at the SAE 15 °C reference.
    pub fn n_full(&self) -> f64 {
        P_NWP * self.volume_m3 / (R_U * T_REF_SOC_K + P_NWP * B_COVOL)
    }

    /// Deliverable mass at 100 % SoC, kg.
    pub fn mass_full_kg(&self) -> f64 {
        self.n_full() * M_H2
    }
}

/// Tank pressure from the declared covolume EOS, Pa.
pub fn pressure(p: &TankParams, n_mol: f64, t_gas_k: f64) -> f64 {
    n_mol * R_U * t_gas_k / (p.volume_m3 - n_mol * B_COVOL)
}

/// Compressibility factor implied by the declared EOS (diagnostic).
pub fn z_factor(p_pa: f64, t_k: f64) -> f64 {
    1.0 + B_COVOL * p_pa / (R_U * t_k)
}

/// Inlet specific enthalpy, J/kg — see the module docs for the derivation.
pub fn h_inlet(t_in_k: f64, p_tank_pa: f64) -> f64 {
    CP0 * t_in_k + B_COVOL * p_tank_pa / M_H2
}

#[derive(Debug, Clone, Copy)]
pub struct TankState {
    pub n_mol: f64,
    pub t_gas_k: f64,
    pub t_liner_k: f64,
    /// Cumulative delivered enthalpy ∫ṁ·h_in dt, J. Carried in the state so
    /// the energy-balance test closes on the *integrator's* own quadrature.
    pub enthalpy_in_j: f64,
}

impl TankState {
    /// A tank resting at `soc` with gas and wall equilibrated at `t_c`.
    pub fn resting(p: &TankParams, soc: f64, t_c: f64) -> Self {
        Self {
            n_mol: soc * p.n_full(),
            t_gas_k: t_c + 273.15,
            t_liner_k: t_c + 273.15,
            enthalpy_in_j: 0.0,
        }
    }

    pub fn soc(&self, p: &TankParams) -> f64 {
        self.n_mol / p.n_full()
    }

    pub fn mass_kg(&self) -> f64 {
        self.n_mol * M_H2
    }

    pub fn pressure(&self, p: &TankParams) -> f64 {
        pressure(p, self.n_mol, self.t_gas_k)
    }

    /// dT_liner/dt at this instant, K/s (the gated structural rate).
    pub fn liner_ramp(&self, p: &TankParams) -> f64 {
        (p.ua_gas_liner * (self.t_gas_k - self.t_liner_k)
            - p.ua_liner_amb * (self.t_liner_k - (p.t_amb_c + 273.15)))
            / p.c_liner
    }

    /// Gas-temperature time constant, s: τ = m·c_v / (ṁ·c_v + UA).
    ///
    /// From linearizing the gas balance about T_gas:
    /// ∂(dT/dt)/∂T = −(ṁ·c_v + UA)/(m·c_v).
    pub fn tau_gas_s(&self, p: &TankParams, mdot: f64) -> f64 {
        self.mass_kg() * CV0 / (mdot * CV0 + p.ua_gas_liner)
    }
}

/// One RK4 step of `dt` seconds at commanded mass flow `mdot` (kg/s) with
/// inlet temperature `t_in_k`.
pub fn step(
    p: &TankParams,
    s: &TankState,
    mdot: f64,
    t_in_k: f64,
    dt: f64,
) -> TankState {
    let t_amb_k = p.t_amb_c + 273.15;
    let f = |st: &TankState| {
        let m = st.n_mol * M_H2;
        let pres = pressure(p, st.n_mol, st.t_gas_k);
        let h_in = h_inlet(t_in_k, pres);
        let q_wall = p.ua_gas_liner * (st.t_gas_k - st.t_liner_k);
        (
            mdot / M_H2,
            (mdot * (h_in - CV0 * st.t_gas_k) - q_wall) / (m * CV0),
            (q_wall - p.ua_liner_amb * (st.t_liner_k - t_amb_k)) / p.c_liner,
            mdot * h_in,
        )
    };
    let add = |st: &TankState, k: (f64, f64, f64, f64), h: f64| TankState {
        n_mol: st.n_mol + h * k.0,
        t_gas_k: st.t_gas_k + h * k.1,
        t_liner_k: st.t_liner_k + h * k.2,
        enthalpy_in_j: st.enthalpy_in_j + h * k.3,
    };
    let k1 = f(s);
    let k2 = f(&add(s, k1, 0.5 * dt));
    let k3 = f(&add(s, k2, 0.5 * dt));
    let k4 = f(&add(s, k3, dt));
    TankState {
        n_mol: s.n_mol + dt / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0),
        t_gas_k: s.t_gas_k + dt / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1),
        t_liner_k: s.t_liner_k + dt / 6.0 * (k1.2 + 2.0 * k2.2 + 2.0 * k3.2 + k4.2),
        enthalpy_in_j: s.enthalpy_in_j
            + dt / 6.0 * (k1.3 + 2.0 * k2.3 + 2.0 * k3.3 + k4.3),
    }
}

/// **L12 — MANDATORY integration-step check.**
///
/// The stiffest gated dynamic is the gas temperature. Its time constant is
/// `τ = m·c_v / (ṁ·c_v + UA)`, which is SMALLEST when the gas mass is
/// smallest (start of fill, at the declared residual SoC) and the mass flow
/// is largest (top tier). This function returns that worst-case τ, in
/// seconds, for the given tank and the fastest flow tier; the doctrine
/// requires `DT_S ≤ τ_min / 5` and the test below enforces it.
///
/// Measured for the nominal 350 L module at 120 g/s from 5 % SoC:
/// m = 0.663 kg → m·c_v = 6749 J/K; ṁ·c_v + UA = 1222 + 500 = 1722 W/K;
/// **τ_min = 3.92 s**, so DT_S = 0.25 s resolves it 15.7×, comfortably
/// inside the τ/5 rule. The liner node (τ = C/(UA+UA_amb) ≈ 254 s) and the
/// mass state (a pure ramp) are both far slower and are not binding.
pub fn l12_time_constant_check(p: &TankParams, mdot_max: f64) -> f64 {
    let worst = TankState::resting(p, RESIDUAL_SOC, p.t_amb_c);
    worst.tau_gas_s(p, mdot_max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal() -> TankParams {
        TankParams::nominal(25.0)
    }

    #[test]
    fn l12_dt_resolves_the_stiffest_gated_dynamic() {
        let p = nominal();
        let tau = l12_time_constant_check(&p, 0.120);
        assert!(
            (3.5..4.5).contains(&tau),
            "worst-case gas tau moved: {tau:.3} s — re-derive DT_S"
        );
        assert!(
            DT_S <= tau / 5.0,
            "DT_S {DT_S} violates the tau/5 rule (tau_min {tau:.3} s)"
        );
        // The liner node is much slower; it is not the binding dynamic.
        let tau_liner = p.c_liner / (p.ua_gas_liner + p.ua_liner_amb);
        assert!(tau_liner > 20.0 * tau, "liner tau {tau_liner:.0} s");
    }

    #[test]
    fn real_gas_form_is_the_declared_one() {
        let p = nominal();
        // Z ≈ 1.5 at 700 bar / 300 K: hydrogen's positive deviation.
        let z = z_factor(700e5, 300.0);
        assert!((1.45..1.60).contains(&z), "Z = {z}");
        // The covolume EOS reproduces the SoC reference by construction.
        let n = p.n_full();
        let pr = pressure(&p, n, T_REF_SOC_K);
        assert!((pr - P_NWP).abs() < 1.0, "{pr} vs {P_NWP}");
        // 350 L holds ~13 kg at 700 bar / 15 °C.
        let m = p.mass_full_kg();
        assert!((12.5..14.0).contains(&m), "mass_full = {m} kg");
        // The inlet enthalpy departure is NOT negligible at fuelling
        // pressure — dropping it would halve the modelled heating.
        let dep = B_COVOL * P_NWP / M_H2;
        assert!(dep > 0.15 * CP0 * 233.15, "departure {dep} J/kg");
    }

    #[test]
    fn energy_balance_closes_on_an_adiabatic_fill() {
        // With both conductances zeroed, the gas control volume must satisfy
        // m_f·u_f − m_0·u_0 = ∫ṁ·h_in dt exactly (u = c_v·T for a covolume
        // gas — the whole reason this EOS was chosen).
        let mut p = nominal();
        p.ua_gas_liner = 0.0;
        p.ua_liner_amb = 0.0;
        let s0 = TankState::resting(&p, RESIDUAL_SOC, 25.0);
        let mut s = s0;
        let target = 0.95 * p.n_full();
        while s.n_mol < target {
            s = step(&p, &s, 0.090, 233.15, DT_S);
        }
        let du = s.mass_kg() * CV0 * s.t_gas_k - s0.mass_kg() * CV0 * s0.t_gas_k;
        let rel = (du - s.enthalpy_in_j).abs() / s.enthalpy_in_j.abs();
        assert!(rel < 1e-6, "energy balance off by {rel:e} (du {du}, in {})",
                s.enthalpy_in_j);
    }

    #[test]
    fn adiabatic_compression_heating_is_real() {
        // A fast fill WITHOUT pre-cooling must blow through the 85 °C
        // ceiling — otherwise the governance problem is vacuous.
        let p = nominal();
        let mut s = TankState::resting(&p, RESIDUAL_SOC, 25.0);
        let target = 0.9375 * p.n_full();
        let mut peak = s.t_gas_k;
        while s.n_mol < target {
            s = step(&p, &s, 0.120, 298.15, DT_S);
            peak = peak.max(s.t_gas_k);
        }
        assert!(
            peak - 273.15 > T_GAS_CEILING_C,
            "un-pre-cooled fast fill must exceed {T_GAS_CEILING_C} °C (got {:.1})",
            peak - 273.15
        );
        // And the pressure ceiling is a real constraint at that temperature.
        assert!(s.pressure(&p) > 0.9 * P_CEILING);
    }

    #[test]
    fn precooling_genuinely_helps_and_is_monotone() {
        let p = nominal();
        let target = 0.9375 * p.n_full();
        let peak_at = |t_in_c: f64| {
            let mut s = TankState::resting(&p, RESIDUAL_SOC, 25.0);
            let mut peak = s.t_gas_k;
            while s.n_mol < target {
                s = step(&p, &s, 0.120, t_in_c + 273.15, DT_S);
                peak = peak.max(s.t_gas_k);
            }
            peak - 273.15
        };
        let (p40, p20, p00, p25) = (peak_at(-40.0), peak_at(-20.0), peak_at(0.0), peak_at(25.0));
        assert!(p40 < p20 && p20 < p00 && p00 < p25, "{p40} {p20} {p00} {p25}");
        // -40 °C is the T40 category for a reason: it is the tier that gets
        // a *fast* fill near the ceiling rather than far past it.
        assert!(p40 < T_GAS_CEILING_C + 10.0, "T40 fast fill peak {p40:.1} °C");
        assert!(p20 > T_GAS_CEILING_C, "T20 fast fill must breach ({p20:.1} °C)");
    }

    #[test]
    fn slow_fills_dump_the_heat_into_a_wall_that_cannot_shed_it() {
        // The harness's whole premise: the wall absorbs, and does not
        // release. A slow fill ends cooler in the gas but leaves the wall
        // hot, and the wall stays hot on a fill timescale.
        let p = nominal();
        let target = 0.9375 * p.n_full();
        let run = |mdot: f64| {
            let mut s = TankState::resting(&p, RESIDUAL_SOC, 25.0);
            while s.n_mol < target {
                s = step(&p, &s, mdot, 233.15, DT_S);
            }
            (s.t_gas_k - 273.15, s.t_liner_k - 273.15)
        };
        let (g_fast, l_fast) = run(0.120);
        let (g_slow, l_slow) = run(0.015);
        assert!(g_slow < g_fast, "slow fill must end cooler in the gas");
        assert!(l_slow > l_fast + 10.0, "slow fill must load the wall");
        // Wall → ambient time constant is hours, not minutes.
        let tau_out = p.c_liner / p.ua_liner_amb;
        assert!(tau_out > 3600.0, "wall must not shed heat during a fill ({tau_out:.0} s)");
    }

    #[test]
    fn integration_is_bit_deterministic() {
        let p = nominal();
        let run = || {
            let mut s = TankState::resting(&p, RESIDUAL_SOC, 25.0);
            for _ in 0..4000 {
                s = step(&p, &s, 0.070, 243.15, DT_S);
            }
            (s.n_mol.to_bits(), s.t_gas_k.to_bits(), s.t_liner_k.to_bits())
        };
        assert_eq!(run(), run());
    }
}
