//! The fill semi-MDP: one decision carries the tank across ONE SoC band.
//!
//! State = (SoC band × gas-temperature band × liner-temperature band).
//! Action = (mass-flow tier × pre-cool setpoint). The decision is taken at
//! band ENTRY and HELD for the whole crossing (L13 — the deployed semantics
//! of a band-indexed table, and the patent posture: no in-fill adaptation).
//!
//! # Characterization: gates from worst-case edges, transitions from centres
//!
//! Every (state, action) is characterized by integrating the true continuous
//! model across the band crossing. **Three** runs are needed, because the
//! constraints do not share a worst-case sense (L8) and because transitions
//! must not compound:
//!
//! | run | start edges | what it decides |
//! |---|---|---|
//! | HOT/HOT | gas band hot edge, liner band hot edge | gas-ceiling gate, pressure gate, liner-ceiling gate |
//! | HOT/COLD | gas band hot edge, liner band **cold** edge | liner thermal-ramp gate (max gradient) |
//! | CENTRE | both band centres | next state and duration |
//!
//! The structural gate `[2]` is the union of two sub-checks whose worst-case
//! liner edges are **opposite** — the liner *ceiling* is worst from the
//! hottest wall, the liner *ramp* is worst from the coldest wall — so it is
//! characterized from both edges. That is L8 applied *within* one gate.
//!
//! **Why transitions come from band centres.** The R3 falsifier measured
//! this directly: characterizing the transition from the worst-case edge
//! COMPOUNDS. Each of the 15 crossings re-applies the hot edge to a state
//! the previous crossing already pushed up, ratcheting ~half a band per
//! step until every path is gated and the DP is vacuously infeasible
//! (measured 0/24 feasible). Splitting the model preserves the safety
//! guarantee *exactly*: the hard gates are evaluated at whatever band the
//! closed loop actually enters, from that band's own worst edge, so safety
//! never depended on the transition model being conservative. Only
//! optimality does — and the closed-loop benchmark on the continuous model
//! is the check on that.
//!
//! **Band-grid alignment (also falsifier-measured).** The top band edge of
//! each gated temperature dimension is aligned with its ceiling
//! (gas → 85 °C, liner → 75 °C). A band whose hot edge sits *above* its
//! ceiling is unconditionally dead, and it is reachable — any legal crossing
//! may end inside it — which strands the closed loop mid-fill.

use crate::thermo::{
    pressure, step, TankParams, TankState, CP0, DT_S, LINER_RAMP_CAP, M_H2,
    P_CEILING, RESIDUAL_SOC, T_GAS_CEILING_C, T_LINER_CEILING_C,
};
use qceom_core::{Action, Charge, ConstraintSpec, Environment, PmeConfig, SparseGraph};

pub const SOC_BANDS: usize = 16;
/// Gas-temperature bands: 5 K each from 10 °C — the top edge is 85 °C, the
/// gas ceiling exactly.
pub const GAS_BANDS: usize = 15;
pub const GAS_BASE_C: f64 = 10.0;
pub const GAS_BAND_C: f64 = 5.0;
/// Liner bands: 6 K each from 15 °C — the top edge is 75 °C, the liner
/// ceiling exactly.
pub const LIN_BANDS: usize = 10;
pub const LIN_BASE_C: f64 = 15.0;
pub const LIN_BAND_C: f64 = 6.0;

pub const N_STATES: usize = SOC_BANDS * GAS_BANDS * LIN_BANDS; // 2400

/// Commanded mass-flow tiers, kg/s (per 350 L module of a multi-tank HDV).
pub const FLOW_TIERS: [f64; 6] = [0.015, 0.030, 0.050, 0.070, 0.095, 0.120];
/// Pre-cool setpoints, °C (SAE station categories T40/T30/T20/T10).
pub const PRECOOL_C: [f64; 4] = [-40.0, -30.0, -20.0, -10.0];
pub const ACTIONS: usize = FLOW_TIERS.len() * PRECOOL_C.len(); // 24

/// Terminal SoC band: the fill is complete at SoC ≥ 15/16 = 93.75 %.
pub const TARGET_BAND: usize = 15;

/// Declared pre-cooler coefficient of performance (electrical → thermal).
pub const COP_PRECOOL: f64 = 0.8;

/// Objective weight on dispenser occupancy, per minute.
pub const ALPHA: f64 = 1.0;
/// Objective weight on pre-cooling, in dispenser-minutes per kWh.
///
/// DERIVED, not tuned. The station's pre-cooler is the throughput bottleneck
/// for back-to-back heavy-duty fills, so a kWh of pre-cooling duty is priced
/// in the dispenser-minutes the chiller needs to recover it:
/// 1 kWh_electrical × COP 0.8 = 0.8 kWh_thermal, and a 16 kW pre-cooler
/// recovers that in 0.8/16 h = **3.0 min**. A station with a different
/// chiller re-declares β and re-solves; that is the whole point of the
/// between-session re-solve cadence.
pub const BETA: f64 = 3.0;

/// Hard cap on a single band crossing, s (a longer crossing is "stalled").
pub const MAX_CROSSING_S: f64 = 900.0;

pub fn soc_band(soc: f64) -> usize {
    ((soc * SOC_BANDS as f64).floor().max(0.0) as usize).min(SOC_BANDS - 1)
}

pub fn gas_band(t_gas_c: f64) -> usize {
    (((t_gas_c - GAS_BASE_C) / GAS_BAND_C).floor().max(0.0) as usize).min(GAS_BANDS - 1)
}

pub fn liner_band(t_liner_c: f64) -> usize {
    (((t_liner_c - LIN_BASE_C) / LIN_BAND_C).floor().max(0.0) as usize).min(LIN_BANDS - 1)
}

pub fn state_id(sb: usize, gb: usize, lb: usize) -> usize {
    (sb * GAS_BANDS + gb) * LIN_BANDS + lb
}

pub fn bands(s: usize) -> (usize, usize, usize) {
    (s / (GAS_BANDS * LIN_BANDS), (s / LIN_BANDS) % GAS_BANDS, s % LIN_BANDS)
}

/// Action decoding: `a = flow_tier * 4 + precool_tier`.
pub fn flow_of(a: Action) -> f64 {
    FLOW_TIERS[a / PRECOOL_C.len()]
}

pub fn precool_of(a: Action) -> f64 {
    PRECOOL_C[a % PRECOOL_C.len()]
}

/// SoC interval spanned by a band, floored at the declared residual.
pub fn band_soc_range(sb: usize) -> (f64, f64) {
    let lo = (sb as f64 / SOC_BANDS as f64).max(RESIDUAL_SOC);
    let hi = (sb + 1) as f64 / SOC_BANDS as f64;
    (lo, hi.max(lo + 1e-9))
}

/// Pre-cooling electricity for delivering `dmass` kg at `set_c`, kWh.
pub fn precool_kwh(dmass_kg: f64, set_c: f64, t_amb_c: f64) -> f64 {
    dmass_kg * CP0 * (t_amb_c - set_c) / COP_PRECOOL / 3.6e6
}

#[derive(Debug, Clone, Copy)]
struct Crossing {
    secs: f64,
    t_gas_end_c: f64,
    t_liner_end_c: f64,
    max_gas_c: f64,
    max_p: f64,
    max_liner_c: f64,
    max_ramp: f64,
    stalled: bool,
}

fn cross(
    p: &TankParams,
    n0: f64,
    n1: f64,
    t_gas_c: f64,
    t_liner_c: f64,
    mdot: f64,
    set_c: f64,
) -> Crossing {
    let mut s = TankState {
        n_mol: n0,
        t_gas_k: t_gas_c + 273.15,
        t_liner_k: t_liner_c + 273.15,
        enthalpy_in_j: 0.0,
    };
    let t_in_k = set_c + 273.15;
    let mut secs = 0.0;
    let mut max_gas = s.t_gas_k;
    let mut max_p = pressure(p, s.n_mol, s.t_gas_k);
    let mut max_liner = s.t_liner_k;
    let mut max_ramp = s.liner_ramp(p);
    while s.n_mol < n1 && secs < MAX_CROSSING_S {
        s = step(p, &s, mdot, t_in_k, DT_S);
        secs += DT_S;
        if s.t_gas_k > max_gas {
            max_gas = s.t_gas_k;
        }
        let pr = pressure(p, s.n_mol, s.t_gas_k);
        if pr > max_p {
            max_p = pr;
        }
        if s.t_liner_k > max_liner {
            max_liner = s.t_liner_k;
        }
        let r = s.liner_ramp(p);
        if r > max_ramp {
            max_ramp = r;
        }
    }
    Crossing {
        secs,
        t_gas_end_c: s.t_gas_k - 273.15,
        t_liner_end_c: s.t_liner_k - 273.15,
        max_gas_c: max_gas - 273.15,
        max_p,
        max_liner_c: max_liner - 273.15,
        max_ramp,
        stalled: s.n_mol < n1,
    }
}

/// The characterized band-crossing table — the whole declared model the
/// kernel and the reference DP both see.
#[derive(Debug, Clone)]
pub struct TransitionTable {
    pub next: Vec<[u16; ACTIONS]>,
    pub minutes: Vec<[f64; ACTIONS]>,
    pub kwh: Vec<[f64; ACTIONS]>,
    /// Worst-case gate levels per (state, action): [thermal, pressure,
    /// structural]; 1.0 = the crossing breaches somewhere.
    pub viol: Vec<[[f64; 3]; ACTIONS]>,
}

impl TransitionTable {
    pub fn characterize(p: &TankParams) -> Self {
        let mut next = vec![[0u16; ACTIONS]; N_STATES];
        let mut minutes = vec![[0.0f64; ACTIONS]; N_STATES];
        let mut kwh = vec![[0.0f64; ACTIONS]; N_STATES];
        let mut viol = vec![[[0.0f64; 3]; ACTIONS]; N_STATES];
        let n_full = p.n_full();
        for sb in 0..SOC_BANDS {
            let (soc0, soc1) = band_soc_range(sb);
            let (n0, n1) = (soc0 * n_full, soc1 * n_full);
            let dmass = (n1 - n0) * M_H2;
            for gb in 0..GAS_BANDS {
                let g_hot = GAS_BASE_C + (gb + 1) as f64 * GAS_BAND_C;
                let g_mid = GAS_BASE_C + (gb as f64 + 0.5) * GAS_BAND_C;
                for lb in 0..LIN_BANDS {
                    let l_hot = LIN_BASE_C + (lb + 1) as f64 * LIN_BAND_C;
                    let l_mid = LIN_BASE_C + (lb as f64 + 0.5) * LIN_BAND_C;
                    let l_cold = LIN_BASE_C + lb as f64 * LIN_BAND_C;
                    let sid = state_id(sb, gb, lb);
                    for a in 0..ACTIONS {
                        let (mdot, set_c) = (flow_of(a), precool_of(a));
                        // HOT/HOT: gas ceiling, pressure, liner ceiling.
                        let hh = cross(p, n0, n1, g_hot, l_hot, mdot, set_c);
                        // HOT/COLD: liner thermal-ramp (steepest gradient).
                        let hc = cross(p, n0, n1, g_hot, l_cold, mdot, set_c);
                        // CENTRE: transition + duration.
                        let cc = cross(p, n0, n1, g_mid, l_mid, mdot, set_c);
                        let mut v = [0.0f64; 3];
                        if hh.max_gas_c > T_GAS_CEILING_C {
                            v[0] = 1.0;
                        }
                        if hh.max_p > P_CEILING {
                            v[1] = 1.0;
                        }
                        if hh.max_liner_c > T_LINER_CEILING_C
                            || hc.max_ramp > LINER_RAMP_CAP
                            || hh.stalled
                            || cc.stalled
                        {
                            v[2] = 1.0;
                        }
                        viol[sid][a] = v;
                        minutes[sid][a] = cc.secs / 60.0;
                        kwh[sid][a] = precool_kwh(dmass, set_c, p.t_amb_c);
                        next[sid][a] = state_id(
                            (sb + 1).min(SOC_BANDS - 1),
                            gas_band(cc.t_gas_end_c),
                            liner_band(cc.t_liner_end_c),
                        ) as u16;
                    }
                }
            }
        }
        Self { next, minutes, kwh, viol }
    }

    /// The declared objective for one crossing (minutes-equivalent).
    pub fn cost(&self, s: usize, a: Action) -> f64 {
        ALPHA * self.minutes[s][a] + BETA * self.kwh[s][a]
    }
}

#[derive(Debug, Clone)]
pub struct FillEnv {
    pub table: TransitionTable,
    pub params: TankParams,
    pub start: usize,
    honor_gates: bool,
}

impl FillEnv {
    pub fn new(p: &TankParams, start_soc: f64) -> Self {
        Self {
            table: TransitionTable::characterize(p),
            params: *p,
            start: state_id(
                soc_band(start_soc.max(RESIDUAL_SOC)),
                gas_band(p.t_amb_c),
                liner_band(p.t_amb_c),
            ),
            honor_gates: true,
        }
    }

    /// The proof-pair twin: identical rewards, gates removed.
    pub fn with_gates_ignored(mut self) -> Self {
        self.honor_gates = false;
        self
    }
}

impl Environment for FillEnv {
    fn num_states(&self) -> usize {
        N_STATES
    }

    fn num_actions(&self) -> usize {
        ACTIONS
    }

    fn start_state(&self) -> usize {
        self.start
    }

    fn is_terminal(&self, s: usize) -> bool {
        bands(s).0 >= TARGET_BAND
    }

    fn step(&self, s: usize, a: Action) -> (usize, f64) {
        if self.is_terminal(s) {
            return (s, 0.0);
        }
        // Reward = negative declared cost. Safety contributes NOTHING to the
        // reward (proof-pair discipline, L5): the gates are the only thing
        // that distinguishes the governed policy from the free one.
        (self.table.next[s][a] as usize, -self.table.cost(s, a))
    }

    fn position(&self, s: usize) -> (f64, f64) {
        let (sb, gb, lb) = bands(s);
        (
            sb as f64 + 0.5,
            gb as f64 + 0.5 + lb as f64 / LIN_BANDS as f64,
        )
    }

    fn charges(&self) -> Vec<Charge> {
        let mut charges = Vec::new();
        // The hot gas rows repel; the completed-fill column attracts.
        for sb in 0..SOC_BANDS {
            charges.push(Charge::new(sb as f64 + 0.5, GAS_BANDS as f64 - 0.5, 2.0));
        }
        charges.push(Charge::new(TARGET_BAND as f64 + 0.5, 1.5, -25.0));
        charges
    }

    fn pme_config(&self) -> PmeConfig {
        PmeConfig { grid: 32, length: 32.0, sigma: 1.5 }
    }

    fn state_graph(&self) -> SparseGraph {
        let mut g = SparseGraph::new(N_STATES);
        for sb in 0..SOC_BANDS {
            for gb in 0..GAS_BANDS {
                for lb in 0..LIN_BANDS {
                    let s = state_id(sb, gb, lb);
                    if sb + 1 < SOC_BANDS {
                        g.add_edge(s, state_id(sb + 1, gb, lb));
                    }
                    if gb + 1 < GAS_BANDS {
                        g.add_edge(s, state_id(sb, gb + 1, lb));
                    }
                    if lb + 1 < LIN_BANDS {
                        g.add_edge(s, state_id(sb, gb, lb + 1));
                    }
                }
            }
        }
        g
    }

    fn constraints(&self) -> Vec<ConstraintSpec> {
        if self.honor_gates {
            vec![ConstraintSpec { hard_limit: 1.0 }; 3]
        } else {
            vec![ConstraintSpec { hard_limit: f64::INFINITY }; 3]
        }
    }

    fn violations(&self, s: usize, a: Action) -> Vec<f64> {
        self.table.viol[s][a].to_vec()
    }
}

/// Plain backward-induction DP on the SAME declared table — the L9 probe.
/// The SoC band strictly increases every decision, so the state space is a
/// DAG layered by SoC and one backward sweep is exact and undiscounted.
/// Returns (value at start, action per state; `usize::MAX` where no
/// admissible action exists).
pub fn plain_dp(env: &FillEnv) -> (f64, Vec<usize>) {
    plain_dp_gated(env, true)
}

/// The same reference DP with the rulebook optionally switched off — the
/// model-level half of the proof pair (L5). Run with `honor_gates = false`
/// it computes the UNGOVERNED optimum on identical rewards; the governed
/// optimum can then only be worse, and the gap is exactly what the rulebook
/// costs. This comparison is made on the declared model rather than on
/// kernel rollouts because the kernel's runtime regulator also SOFT-damps
/// constraint-stressed actions, which would confound a reward comparison
/// between a gated and an ungated twin.
pub fn plain_dp_gated(env: &FillEnv, honor_gates: bool) -> (f64, Vec<usize>) {
    let mut val = vec![f64::INFINITY; N_STATES];
    let mut act = vec![usize::MAX; N_STATES];
    for gb in 0..GAS_BANDS {
        for lb in 0..LIN_BANDS {
            for sb in TARGET_BAND..SOC_BANDS {
                val[state_id(sb, gb, lb)] = 0.0;
            }
        }
    }
    for sb in (0..TARGET_BAND).rev() {
        for gb in 0..GAS_BANDS {
            for lb in 0..LIN_BANDS {
                let s = state_id(sb, gb, lb);
                let (mut best, mut ba) = (f64::INFINITY, usize::MAX);
                for a in 0..ACTIONS {
                    if honor_gates && env.table.viol[s][a] != [0.0; 3] {
                        continue;
                    }
                    let v = env.table.cost(s, a) + val[env.table.next[s][a] as usize];
                    if v < best {
                        best = v;
                        ba = a;
                    }
                }
                val[s] = best;
                act[s] = ba;
            }
        }
    }
    (val[env.start], act)
}

/// Walk a state-indexed action map from the start state, returning the
/// (state, action) pairs visited until the fill terminates.
pub fn dp_path(env: &FillEnv, act: &[usize]) -> Vec<(usize, usize)> {
    let mut s = env.start;
    let mut path = Vec::new();
    for _ in 0..(SOC_BANDS + 4) {
        if env.is_terminal(s) || act[s] >= ACTIONS {
            break;
        }
        let a = act[s];
        path.push((s, a));
        s = env.table.next[s][a] as usize;
    }
    path
}

/// Total declared cost of following `act` on the table from the start state.
pub fn table_cost_of(env: &FillEnv, act: &dyn Fn(usize) -> usize) -> Option<f64> {
    let mut s = env.start;
    let mut total = 0.0;
    for _ in 0..(SOC_BANDS + 4) {
        if env.is_terminal(s) {
            return Some(total);
        }
        let a = act(s);
        if a >= ACTIONS || env.table.viol[s][a] != [0.0; 3] {
            return None;
        }
        total += env.table.cost(s, a);
        s = env.table.next[s][a] as usize;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain_engine;

    fn env25() -> FillEnv {
        FillEnv::new(&TankParams::nominal(25.0), RESIDUAL_SOC)
    }

    #[test]
    fn state_space_is_sized_for_the_kernel() {
        assert_eq!(N_STATES, 2400);
        assert!(N_STATES < 5000);
        assert_eq!(ACTIONS, 24);
        // Band grids end exactly on their ceilings (see module docs).
        assert_eq!(GAS_BASE_C + GAS_BANDS as f64 * GAS_BAND_C, T_GAS_CEILING_C);
        assert_eq!(LIN_BASE_C + LIN_BANDS as f64 * LIN_BAND_C, T_LINER_CEILING_C);
        // Band round-tripping.
        for s in [0, 1, 137, 999, N_STATES - 1] {
            let (a, b, c) = bands(s);
            assert_eq!(state_id(a, b, c), s);
        }
    }

    #[test]
    fn characterization_is_physical() {
        let env = env25();
        let s = state_id(6, 3, 1);
        // Higher flow crosses a band faster.
        let fast = state_id(6, 3, 1);
        let slow_a = 0; // tier 0, -40
        let fast_a = 5 * 4; // tier 5, -40
        assert!(env.table.minutes[fast][fast_a] < env.table.minutes[fast][slow_a]);
        // Colder pre-cooling costs strictly more electricity.
        assert!(env.table.kwh[s][0] > env.table.kwh[s][3]);
        // The top flow tier with the WARMEST pre-cool must breach the gas
        // ceiling somewhere — otherwise the thermal gate is vacuous.
        let hot_fast = 5 * 4 + 3; // 120 g/s at -10 °C
        let any_thermal = (0..N_STATES).any(|st| env.table.viol[st][hot_fast][0] > 0.0);
        assert!(any_thermal, "the 85 °C gate must bind somewhere");
        // The pressure gate must bind near full SoC at high temperature.
        let any_press = (0..N_STATES).any(|st| env.table.viol[st][hot_fast][1] > 0.0);
        assert!(any_press, "the 875 bar gate must bind somewhere");
        // The structural gate must bind somewhere too.
        let any_struct = (0..N_STATES).any(|st| (0..ACTIONS).any(|a| env.table.viol[st][a][2] > 0.0));
        assert!(any_struct, "the structural gate must bind somewhere");
        // But not everywhere: the low-flow / T40 corner must stay clean at
        // ordinary states, or the problem is infeasible rather than governed.
        assert_eq!(env.table.viol[state_id(2, 2, 0)][0], [0.0; 3]);
    }

    #[test]
    fn governed_rollout_is_clean_and_completes_the_fill() {
        let env = env25();
        let (policy, report) = domain_engine().train(&env);
        assert!(report.converged, "engine must converge");
        let rollout = policy.rollout(&env, 64);
        assert!(rollout.reached_terminal, "the governed map must finish the fill");
        for (i, &s) in rollout.states.iter().enumerate().take(rollout.actions.len()) {
            let a = rollout.actions[i];
            assert_eq!(env.table.viol[s][a], [0.0; 3], "gate breach at decision {i}");
        }
    }

    #[test]
    fn proof_pair_ungoverned_is_cheaper_and_breaches() {
        // L5, model level: on IDENTICAL rewards, the ungoverned optimum must
        // strictly BEAT the governed optimum and must breach the rulebook,
        // while the governed optimum never does. That is the statement that
        // the CONSTRAINT -- not the reward -- produced the behaviour.
        let env = env25();
        let (gov_cost, gov_act) = plain_dp_gated(&env, true);
        let (free_cost, free_act) = plain_dp_gated(&env, false);
        assert!(gov_cost.is_finite() && free_cost.is_finite());
        assert!(
            free_cost < gov_cost - 1e-9,
            "ungoverned optimum must be strictly cheaper: free {free_cost} vs governed {gov_cost}"
        );
        let breaches = |act: &[usize]| {
            dp_path(&env, act)
                .into_iter()
                .filter(|&(s, a)| env.table.viol[s][a] != [0.0; 3])
                .count()
        };
        assert!(breaches(&free_act) > 0, "ungoverned optimum must breach");
        assert_eq!(breaches(&gov_act), 0, "governed optimum must never breach");

        // Kernel level: the gates-ignored twin's rollout also breaches, and
        // the governed twin's rollout never does.
        let free_env = env25().with_gates_ignored();
        let (gp, _) = domain_engine().train(&env);
        let (fp, _) = domain_engine().train(&free_env);
        let g_roll = gp.rollout(&env, 64);
        let f_roll = fp.rollout(&free_env, 64);
        assert!(f_roll.reached_terminal && g_roll.reached_terminal);
        let roll_breaches = |r: &qceom_core::Rollout| {
            r.states
                .iter()
                .enumerate()
                .take(r.actions.len())
                .filter(|(i, &s)| env.table.viol[s][r.actions[*i]] != [0.0; 3])
                .count()
        };
        assert!(roll_breaches(&f_roll) > 0, "ungoverned rollout must breach");
        assert_eq!(roll_breaches(&g_roll), 0, "governed rollout must never breach");
    }

    #[test]
    fn kernel_matches_the_plain_dp_reference() {
        // L9: the kernel must reproduce plain backward induction on the same
        // declared model, exactly.
        let env = env25();
        let (ref_cost, _) = plain_dp(&env);
        assert!(ref_cost.is_finite(), "the declared model must be feasible");
        let (policy, _) = domain_engine().train(&env);
        let kernel_cost = table_cost_of(&env, &|s| policy.action(s))
            .expect("kernel policy must be gate-clean and terminate");
        assert!(
            (kernel_cost - ref_cost).abs() < 1e-9,
            "kernel {kernel_cost} != plain-DP reference {ref_cost}"
        );
    }

    #[test]
    fn training_is_bit_deterministic() {
        let env = env25();
        let (p1, r1) = domain_engine().train(&env);
        let (p2, r2) = domain_engine().train(&env);
        assert_eq!(r1.iterations, r2.iterations);
        for s in 0..N_STATES {
            assert_eq!(p1.value(s).to_bits(), p2.value(s).to_bits());
            assert_eq!(p1.action(s), p2.action(s));
        }
    }
}
