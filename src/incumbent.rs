//! The incumbents, unmodified, plus the shared closed-loop scorer.
//!
//! **(a) J2601-style lookup-table method.** A fixed *Average Pressure Ramp
//! Rate* selected once for the fill, plus T40 (−40 °C) pre-cooling for the
//! whole fill — the currently deployed standard. APRR does not change during
//! the fill, so the fill duration is a constant once the table row is
//! chosen. A station realizes an APRR with a pressure-tracking flow
//! controller, which is what `run_table_method` does.
//!
//! **(b) Greedy max-flow-until-hot.** The obvious station algorithm: command
//! the top flow tier and the *cheapest* (warmest) pre-cool setpoint, and back
//! off reactively when the gas gets hot. Given every advantage a real
//! dispenser does not have (L1 baseline discipline): it reads the TRUE
//! in-tank gas temperature with zero sensor lag — real dispensers cannot,
//! which is precisely why J2601 and the MC formula exist — and its hot
//! threshold is oracle-tuned per scenario by `best_greedy`.
//!
//! Both are instrumented for the same gates the governed map is held to, so
//! the comparison is symmetric. Neither *knows* about the liner ceiling —
//! that is the incumbent blind spot the harness exists to close.

use crate::fill_env::{precool_kwh, ALPHA, BETA, FLOW_TIERS, PRECOOL_C, TARGET_BAND, SOC_BANDS};
use crate::thermo::{
    pressure, step, TankParams, TankState, DT_S, LINER_RAMP_CAP, P_CEILING,
    T_GAS_CEILING_C, T_LINER_CEILING_C,
};

/// J2601-style fixed average pressure ramp rate, Pa/s. 16 MPa/min is the
/// aggressive end of the plausible envelope for a tank this large — chosen
/// deliberately to give the incumbent its best shot (L1).
pub const APRR: f64 = 16e6 / 60.0;
/// Proportional gain of the station's ramp-tracking flow controller,
/// kg/s per Pa of pressure error.
pub const APRR_GAIN: f64 = 5e-8;
/// Dispenser mass-flow ceiling, kg/s.
pub const STATION_MAX_FLOW: f64 = 0.150;
/// Hot thresholds offered to the oracle-tuned greedy, °C.
pub const GREEDY_THRESHOLDS: [f64; 4] = [60.0, 65.0, 70.0, 75.0];
/// Terminal SoC of a fill, matching the semi-MDP's terminal band.
pub const TARGET_SOC: f64 = TARGET_BAND as f64 / SOC_BANDS as f64;
/// Hard wall-clock cap on a fill, s.
pub const MAX_FILL_S: f64 = 3600.0;

#[derive(Debug, Clone, Copy)]
pub struct FillRun {
    pub minutes: f64,
    pub precool_kwh: f64,
    pub reached_target: bool,
    pub gas_violation_s: f64,
    pub pressure_violation_s: f64,
    pub liner_violation_s: f64,
    pub ramp_violation_s: f64,
    pub peak_gas_c: f64,
    pub peak_liner_c: f64,
    pub peak_bar: f64,
    /// How often the map had no gate-clean entry and the declared safe
    /// fallback was commanded (governed runs only).
    pub fallbacks: usize,
}

impl FillRun {
    fn new(s: &TankState, p: &TankParams) -> Self {
        Self {
            minutes: 0.0,
            precool_kwh: 0.0,
            reached_target: false,
            gas_violation_s: 0.0,
            pressure_violation_s: 0.0,
            liner_violation_s: 0.0,
            ramp_violation_s: 0.0,
            peak_gas_c: s.t_gas_k - 273.15,
            peak_liner_c: s.t_liner_k - 273.15,
            peak_bar: pressure(p, s.n_mol, s.t_gas_k) / 1e5,
            fallbacks: 0,
        }
    }

    pub fn clean(&self) -> bool {
        self.reached_target
            && self.gas_violation_s == 0.0
            && self.pressure_violation_s == 0.0
            && self.liner_violation_s == 0.0
            && self.ramp_violation_s == 0.0
    }

    /// The declared objective, in dispenser-minutes-equivalent.
    pub fn objective(&self) -> f64 {
        ALPHA * self.minutes + BETA * self.precool_kwh
    }
}

/// Run any controller closed-loop on the continuous model.
///
/// `control(state, seconds, held) -> (mdot, precool_c)`. `held` carries the
/// command chosen at the current SoC band's entry so a controller can honour
/// the semi-MDP hold contract (L13); reactive controllers ignore it.
pub fn run_closed_loop<F>(p: &TankParams, start: TankState, mut control: F) -> FillRun
where
    F: FnMut(&TankState, f64, Option<(f64, f64)>) -> Option<(f64, f64)>,
{
    let n_target = TARGET_SOC * p.n_full();
    let mut s = start;
    let mut t = 0.0;
    let mut out = FillRun::new(&s, p);
    let mut held: Option<(f64, f64)> = None;
    let mut held_band = usize::MAX;
    while t < MAX_FILL_S {
        let band = crate::fill_env::soc_band(s.soc(p));
        if band != held_band {
            held_band = band;
            held = None;
        }
        let cmd = match control(&s, t, held) {
            Some(c) => c,
            None => break,
        };
        held = Some(cmd);
        let (mdot, set_c) = cmd;
        let ramp = s.liner_ramp(p);
        s = step(p, &s, mdot, set_c + 273.15, DT_S);
        t += DT_S;
        out.precool_kwh += precool_kwh(mdot * DT_S, set_c, p.t_amb_c);
        let gas_c = s.t_gas_k - 273.15;
        let liner_c = s.t_liner_k - 273.15;
        let bar = pressure(p, s.n_mol, s.t_gas_k) / 1e5;
        if gas_c > T_GAS_CEILING_C {
            out.gas_violation_s += DT_S;
        }
        if bar * 1e5 > P_CEILING {
            out.pressure_violation_s += DT_S;
        }
        if liner_c > T_LINER_CEILING_C {
            out.liner_violation_s += DT_S;
        }
        if ramp > LINER_RAMP_CAP {
            out.ramp_violation_s += DT_S;
        }
        out.peak_gas_c = out.peak_gas_c.max(gas_c);
        out.peak_liner_c = out.peak_liner_c.max(liner_c);
        out.peak_bar = out.peak_bar.max(bar);
        if s.n_mol >= n_target {
            out.reached_target = true;
            break;
        }
    }
    out.minutes = t / 60.0;
    out
}

/// Incumbent (a): fixed APRR + T40 pre-cooling, tracked by a proportional
/// mass-flow controller.
pub fn run_table_method(p: &TankParams, start: TankState) -> FillRun {
    let p0 = pressure(p, start.n_mol, start.t_gas_k);
    run_closed_loop(p, start, |s, t, _| {
        let err = (p0 + APRR * t) - pressure(p, s.n_mol, s.t_gas_k);
        let mdot = (APRR_GAIN * err).clamp(1e-4, STATION_MAX_FLOW);
        Some((mdot, PRECOOL_C[0]))
    })
}

/// Incumbent (b): greedy max-flow-until-hot at a given hot threshold.
pub fn run_greedy(p: &TankParams, start: TankState, hot_c: f64) -> FillRun {
    let mut fi = FLOW_TIERS.len() - 1;
    let mut ci = PRECOOL_C.len() - 1;
    run_closed_loop(p, start, move |s, _, _| {
        let gas_c = s.t_gas_k - 273.15;
        if gas_c > hot_c && ci > 0 {
            ci -= 1;
        }
        if gas_c > hot_c + 7.0 && fi > 0 {
            fi -= 1;
        }
        Some((FLOW_TIERS[fi], PRECOOL_C[ci]))
    })
}

/// The naive "just push harder" controller: a fixed flow tier at a fixed
/// pre-cool setpoint, with no reaction of any kind. The incumbent blind spot
/// in its purest form.
pub fn run_fixed(p: &TankParams, start: TankState, mdot: f64, set_c: f64) -> FillRun {
    run_closed_loop(p, start, move |_, _, _| Some((mdot, set_c)))
}

/// Oracle-tuned greedy: the best FEASIBLE hot threshold for this tank.
/// Deliberately stronger than any deployable reactive controller.
pub fn best_greedy(p: &TankParams, start: TankState) -> (FillRun, f64) {
    let mut best: Option<(FillRun, f64)> = None;
    for &thr in GREEDY_THRESHOLDS.iter() {
        let r = run_greedy(p, start, thr);
        let better = match best {
            None => true,
            Some((b, _)) => {
                (r.clean(), -r.objective()) > (b.clean(), -b.objective())
            }
        };
        if better {
            best = Some((r, thr));
        }
    }
    best.expect("at least one threshold")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::thermo::{P_NWP, RESIDUAL_SOC};

    #[test]
    fn table_method_completes_a_clean_fill() {
        let p = TankParams::nominal(25.0);
        let s = TankState::resting(&p, RESIDUAL_SOC, 25.0);
        let r = run_table_method(&p, s);
        assert!(r.reached_target, "the J2601-style incumbent must finish");
        assert!(r.minutes > 1.0 && r.minutes < 20.0, "{} min", r.minutes);
        assert!(r.peak_bar < P_CEILING / 1e5, "peak {:.0} bar", r.peak_bar);
        // It fills to ~NWP, as a 700 bar protocol must.
        assert!(r.peak_bar > 0.85 * P_NWP / 1e5, "peak {:.0} bar", r.peak_bar);
    }

    #[test]
    fn the_incumbents_have_a_blind_spot() {
        let p = TankParams::nominal(35.0);
        let s = TankState::resting(&p, RESIDUAL_SOC, 35.0);
        // (1) "Just push harder" -- top flow at the cheapest pre-cool -- must
        // blow through the gas ceiling.
        let naive = run_fixed(&p, s, FLOW_TIERS[5], PRECOOL_C[3]);
        assert!(naive.gas_violation_s > 0.0, "naive max push must overheat");
        assert!(!naive.clean());
        // (2) The ACCUMULATING constraint a gas-watching controller cannot
        // see. A slow fill at cheap pre-cooling dumps its enthalpy into the
        // wall instead of the gas: on the nominal tank the GAS stays legal
        // all the way and the wall still ends within a few K of its ceiling,
        // and on a lighter-walled tank (a legitimate tank-to-tank variation)
        // the LINER ceiling goes while the gas never comes close.
        let slow = run_fixed(&p, s, FLOW_TIERS[0], PRECOOL_C[3]);
        assert!(
            slow.peak_liner_c > T_LINER_CEILING_C - 6.0,
            "slow cheap fill must load the wall (peak liner {:.1} C)",
            slow.peak_liner_c
        );
        let mut light = TankParams::nominal(35.0);
        light.c_liner = 0.9e5;
        let s_light = TankState::resting(&light, RESIDUAL_SOC, 35.0);
        let slow_light = run_fixed(&light, s_light, FLOW_TIERS[0], PRECOOL_C[3]);
        assert!(
            slow_light.liner_violation_s > 0.0,
            "light-walled slow cheap fill must breach the liner ceiling (peak {:.1} C)",
            slow_light.peak_liner_c
        );
        assert!(
            slow.gas_violation_s == 0.0,
            "on the nominal tank the wall, not the gas, is what runs out of              room on a slow cheap fill (peak gas {:.1} C, peak liner {:.1} C)",
            slow.peak_gas_c, slow.peak_liner_c
        );
        // (3) T40 at the top tier is the incumbent's answer and it completes
        // -- the rulebook is binding, not impossible.
        assert!(run_fixed(&p, s, FLOW_TIERS[5], PRECOOL_C[0]).reached_target);
    }

    #[test]
    fn incumbents_are_deterministic() {
        let p = TankParams::nominal(25.0);
        let s = TankState::resting(&p, RESIDUAL_SOC, 25.0);
        let a = run_table_method(&p, s);
        let b = run_table_method(&p, s);
        assert_eq!(a.minutes.to_bits(), b.minutes.to_bits());
        assert_eq!(a.precool_kwh.to_bits(), b.precool_kwh.to_bits());
    }
}
