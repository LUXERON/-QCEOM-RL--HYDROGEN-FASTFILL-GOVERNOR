"""GATE-A1 falsifier for the R3 HYDROGEN-FASTFILL-GOVERNOR.

Question: does exact DP over the banded thermal state beat (a) the
J2601-style static pressure-ramp table method and (b) a TUNED greedy
max-flow-until-hot station controller, on a two-lever objective
(fill minutes + pre-cooling kWh) under a hard rulebook?

KILL CRITERION (declared before running): if the greedy controller is
feasible AND its objective is within 1e-9 of the DP's on EVERY scenario,
the kernel adds nothing over reactive control and the harness DIES.

Why we expect it to live: heat deposited in a Type-IV composite wall does
not leave (UA_amb is ~40x smaller than UA_gas-liner; tau_wall-to-ambient
is hours). The wall temperature is an accumulating, shared,
history-dependent resource, and how much of the fill's enthalpy the wall
absorbs depends on the ORDER in which hot and cold gas is delivered
(rejection is int UA*(Tg-Tl) dt, so warm gas is worth more when the wall
is still cold). A reactive controller prices nothing about the future;
DP prices the whole remaining fill at every decision.

BASELINE DISCIPLINE (L1). The greedy is given every advantage a real
station does NOT have: it reads the TRUE in-tank gas temperature with no
sensor lag (real dispensers cannot -- that is precisely why J2601 and the
MC formula exist), and its hot threshold is ORACLE-TUNED per scenario
(best feasible objective over four thresholds). If DP still beats it, the
result is not an artifact of a strawman.

Declared model (identical in form to src/thermo.rs):

  Real gas, covolume EOS      P = nRT / (V - n*b),  b = 1.9e-5 m^3/mol
                              (exactly equivalent to Z = 1 + bP/(RT))
  Internal energy             u = c_v0 * T   (exact for a covolume gas:
                              (du/dV)_T = T(dP/dT)_V - P = 0)
  Inlet enthalpy              h_in = c_p0*T_in + b*P/M   (the inlet stream
                              is throttled to the current tank pressure)
  Gas energy balance          m*c_v0*dTg/dt = mdot*(h_in - u) - UA*(Tg-Tl)
  Liner energy balance        C_l*dTl/dt = UA*(Tg-Tl) - UA_amb*(Tl-Tamb)

MODEL/GATE SPLIT (the second defect this falsifier caught, recorded here
because the Rust harness inherits the fix). Characterizing the semi-MDP
TRANSITION from the worst-case band edge COMPOUNDS: each of the 15 band
crossings re-applies the hot edge to a state the previous crossing already
pushed up, ratcheting ~half a band per step until every path is gated and
the DP is vacuously infeasible (measured: 0/24 feasible). The fix, which
preserves the safety guarantee exactly: HARD GATES are characterized from
each constraint's worst-case band edge (L8), while the TRANSITION and COST
model uses the band-CENTRE trajectory. Safety is per-crossing and is
evaluated at whatever band the closed loop actually enters, so it does not
depend on the transition model being conservative; only optimality does,
and the closed-loop replay on the continuous model is the check.

Pure Python, no RNG library: scenario generation is splitmix64 bit-mixing,
so the whole run is bit-reproducible. Screening resolution: adaptive step
dt = min(1.0 s, 0.2*tau_gas) -- the same tau-resolving rule the harness
applies (L12), at a coarser cap; the Rust harness re-derives the check and
runs a fixed, finer step.

Run:  python falsifier/falsifier.py
"""

# ---------------------------------------------------------------------------
# Declared constants
# ---------------------------------------------------------------------------
R_U = 8.314462618          # J/(mol K)
M_H2 = 2.01588e-3          # kg/mol
B_COVOL = 1.9e-5           # m^3/mol   (declared H2 covolume)
CV0 = 10180.0              # J/(kg K)
CP0 = 14300.0              # J/(kg K)

V_TANK = 0.35              # m^3   (350 L heavy-duty module)
P_NWP = 700e5              # Pa
T_REF_SOC = 288.15         # K     (SAE 100% SoC reference: NWP at 15 degC)

T_CEILING_K = 273.15 + 85.0         # gas transient ceiling (SAE J2601)
P_CEILING = 875e5                   # Pa, 125% NWP
T_LINER_CEILING_K = 273.15 + 75.0   # sustained liner/resin service limit
RAMP_CAP = 0.45                     # K/s liner thermal-shock cap

FLOW_TIERS = [0.015, 0.030, 0.050, 0.070, 0.095, 0.120]   # kg/s
PRECOOL_C = [-40.0, -30.0, -20.0, -10.0]                  # degC
COP_PRECOOL = 0.8

# Band grids: the TOP band edge of each gated temperature dimension is
# ALIGNED WITH ITS CEILING. A band whose hot edge sits ABOVE the ceiling is
# unconditionally dead, and it is reachable (any legal crossing may end
# inside it), which strands the closed loop -- measured, and the reason the
# grids below end exactly at 85 / 75 degC.
SOC_BANDS = 12                           # screening resolution
GAS_BANDS = 15
LIN_BANDS = 6
GAS_BASE_C, GAS_BAND_C = 10.0, 5.0       # 10 .. 85 degC  (= gas ceiling)
LIN_BASE_C, LIN_BAND_C = 15.0, 10.0      # 15 .. 75 degC  (= liner ceiling)
TARGET_BAND = SOC_BANDS - 1
DT_CAP = 2.0                             # s, screening integrator cap

ALPHA = 1.0    # cost weight on minutes
# BETA is DERIVED, not tuned: the station's pre-cooler is the throughput
# bottleneck for back-to-back fills, so a kWh of pre-cooling duty is priced
# in the dispenser-minutes the chiller needs to recover it.
#   1 kWh_electrical x COP 0.8 = 0.8 kWh_thermal; a 16 kW pre-cooler
#   recovers that in 0.8/16 h = 3.0 min.
BETA = 3.0     # minutes-equivalent per kWh of pre-cooling

APRR = 16e6 / 60.0     # Pa/s -- J2601-style fixed average pressure ramp rate
APRR_GAIN = 5e-8       # kg/s per Pa of ramp-tracking error
STATION_MAX_FLOW = 0.150   # kg/s dispenser ceiling
GREEDY_THRESHOLDS = [60.0, 65.0, 70.0, 75.0]   # oracle-tuned per scenario

N_FULL = P_NWP * V_TANK / (R_U * T_REF_SOC + P_NWP * B_COVOL)   # mol at 100%
RESIDUAL_SOC = 0.05
MAX_FILL_S = 3600.0


def splitmix(x):
    x = (x + 0x9E3779B97F4A7C15) & (2**64 - 1)
    z = x
    z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & (2**64 - 1)
    z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & (2**64 - 1)
    return x, (z ^ (z >> 31))


# ---------------------------------------------------------------------------
# Physics
# ---------------------------------------------------------------------------
def pressure(n, tg):
    return n * R_U * tg / (V_TANK - n * B_COVOL)


def advance(n, tg, tl, mdot, t_in_k, sc, n_stop):
    """Integrate one band crossing.

    Returns (secs, tg, tl, max_tg, max_p, max_tl, max_ramp, stalled).
    """
    secs = 0.0
    max_tg, max_p, max_tl, max_ramp = tg, pressure(n, tg), tl, 0.0
    while n < n_stop and secs < MAX_FILL_S:
        m = n * M_H2
        tau = m * CV0 / (mdot * CV0 + sc["ua"])
        dt = DT_CAP if 0.2 * tau > DT_CAP else 0.2 * tau
        p = pressure(n, tg)
        h_in = CP0 * t_in_k + B_COVOL * p / M_H2
        q_wall = sc["ua"] * (tg - tl)
        dtg = (mdot * (h_in - CV0 * tg) - q_wall) / (m * CV0)
        dtl = (q_wall - sc["ua_amb"] * (tl - sc["t_amb"])) / sc["c_liner"]
        n += mdot / M_H2 * dt
        tg += dtg * dt
        tl += dtl * dt
        secs += dt
        if tg > max_tg:
            max_tg = tg
        p = pressure(n, tg)
        if p > max_p:
            max_p = p
        if tl > max_tl:
            max_tl = tl
        if dtl > max_ramp:
            max_ramp = dtl
    return secs, tg, tl, max_tg, max_p, max_tl, max_ramp, n < n_stop


# ---------------------------------------------------------------------------
# Banding
# ---------------------------------------------------------------------------
def soc_band(soc):
    return min(SOC_BANDS - 1, max(0, int(soc * SOC_BANDS)))


def gas_band(tg_k):
    return min(GAS_BANDS - 1,
               max(0, int((tg_k - 273.15 - GAS_BASE_C) / GAS_BAND_C)))


def lin_band(tl_k):
    return min(LIN_BANDS - 1,
               max(0, int((tl_k - 273.15 - LIN_BASE_C) / LIN_BAND_C)))


def sid(sb, gb, lb):
    return (sb * GAS_BANDS + gb) * LIN_BANDS + lb


N_STATES = SOC_BANDS * GAS_BANDS * LIN_BANDS
ACTIONS = [(f, c) for f in range(len(FLOW_TIERS)) for c in range(len(PRECOOL_C))]


def precool_kwh(dmass, set_c, t_amb_k):
    return dmass * CP0 * ((t_amb_k - 273.15) - set_c) / COP_PRECOOL / 3.6e6


def band_soc(sb):
    return max(RESIDUAL_SOC, sb / SOC_BANDS), (sb + 1) / SOC_BANDS


def characterize(sc):
    """cost, next-state, and hard gates for every (state, action).

    L8 per-constraint worst-case SENSE:
      gas ceiling + pressure + liner ceiling -> hot gas edge AND hot liner
          edge (least wall rejection => hottest gas => highest P; and the
          liner ceiling is worst from the hottest liner)
      liner thermal RAMP -> hot gas edge AND COLD liner edge (max gradient)
    Gate [2] (structural) is the UNION of two sub-checks whose worst-case
    liner edges are OPPOSITE, so it is characterized from BOTH edges.

    Transition + cost come from the band-CENTRE run (see module docstring:
    worst-case transitions compound over a 15-decision horizon).
    """
    cost = [[0.0] * len(ACTIONS) for _ in range(N_STATES)]
    nxt = [[0] * len(ACTIONS) for _ in range(N_STATES)]
    bad = [[True] * len(ACTIONS) for _ in range(N_STATES)]
    for sb in range(SOC_BANDS):
        soc0, soc1 = band_soc(sb)
        n0, n1 = soc0 * N_FULL, soc1 * N_FULL
        dmass = (n1 - n0) * M_H2
        for gb in range(GAS_BANDS):
            tg_hot = 273.15 + GAS_BASE_C + (gb + 1) * GAS_BAND_C
            tg_mid = 273.15 + GAS_BASE_C + (gb + 0.5) * GAS_BAND_C
            for lb in range(LIN_BANDS):
                tl_hot = 273.15 + LIN_BASE_C + (lb + 1) * LIN_BAND_C
                tl_mid = 273.15 + LIN_BASE_C + (lb + 0.5) * LIN_BAND_C
                tl_cold = 273.15 + LIN_BASE_C + lb * LIN_BAND_C
                s = sid(sb, gb, lb)
                for ai, (fi, ci) in enumerate(ACTIONS):
                    mdot = FLOW_TIERS[fi]
                    t_in = 273.15 + PRECOOL_C[ci]
                    # hot-edge run: gas ceiling, pressure, liner ceiling
                    _, _, _, mx_tg, mx_p, mx_tl, _, stall = advance(
                        n0, tg_hot, tl_hot, mdot, t_in, sc, n1)
                    # cold-liner-edge run: thermal-ramp sub-check
                    _, _, _, _, _, _, mx_ramp, _ = advance(
                        n0, tg_hot, tl_cold, mdot, t_in, sc, n1)
                    # band-centre run: transition + duration
                    secs, tge, tle, _, _, _, _, _ = advance(
                        n0, tg_mid, tl_mid, mdot, t_in, sc, n1)
                    bad[s][ai] = not (mx_tg <= T_CEILING_K
                                      and mx_p <= P_CEILING
                                      and mx_tl <= T_LINER_CEILING_K
                                      and mx_ramp <= RAMP_CAP
                                      and not stall)
                    cost[s][ai] = (ALPHA * secs / 60.0
                                   + BETA * precool_kwh(dmass, PRECOOL_C[ci],
                                                        sc["t_amb"]))
                    nxt[s][ai] = sid(min(sb + 1, SOC_BANDS - 1),
                                     gas_band(tge), lin_band(tle))
    return cost, nxt, bad


def exact_dp(table):
    """Backward induction over the SoC-layered DAG. Gates hard-excluded."""
    cost, nxt, bad = table
    INF = float("inf")
    val = [INF] * N_STATES
    act = [-1] * N_STATES
    for gb in range(GAS_BANDS):
        for lb in range(LIN_BANDS):
            for sb in range(TARGET_BAND, SOC_BANDS):
                val[sid(sb, gb, lb)] = 0.0
    for sb in range(TARGET_BAND - 1, -1, -1):
        for gb in range(GAS_BANDS):
            for lb in range(LIN_BANDS):
                s = sid(sb, gb, lb)
                best, ba = INF, -1
                for ai in range(len(ACTIONS)):
                    if bad[s][ai]:
                        continue
                    v = cost[s][ai] + val[nxt[s][ai]]
                    if v < best - 1e-12:
                        best, ba = v, ai
                val[s], act[s] = best, ba
    return val, act


# ---------------------------------------------------------------------------
# Closed-loop replay on the continuous model
# (L13: look up at band ENTRY, HOLD the command for the whole crossing)
# ---------------------------------------------------------------------------
class Run:
    def __init__(self):
        self.minutes = 0.0
        self.kwh = 0.0
        self.reached = False
        self.breaches = [0.0, 0.0, 0.0, 0.0]   # gas, pressure, liner, ramp
        self.peak_gas_c = -273.15
        self.peak_liner_c = -273.15

    @property
    def feasible(self):
        return self.reached and not any(b > 0.0 for b in self.breaches)

    def objective(self):
        return ALPHA * self.minutes + BETA * self.kwh


def replay(sc, controller):
    n = RESIDUAL_SOC * N_FULL if sc["soc0"] < RESIDUAL_SOC else sc["soc0"] * N_FULL
    tg = tl = sc["t_amb"]
    t = 0.0
    out = Run()
    target_n = (TARGET_BAND / SOC_BANDS) * N_FULL
    held, held_band = None, -1
    while t < MAX_FILL_S:
        band = soc_band(n / N_FULL)
        if band != held_band:
            held_band, held = band, None
        cmd = controller(n, tg, tl, t, band, held)
        if cmd is None:
            break
        held = cmd
        mdot, set_c = cmd
        m = n * M_H2
        tau = m * CV0 / (mdot * CV0 + sc["ua"])
        dt = DT_CAP if 0.2 * tau > DT_CAP else 0.2 * tau
        p = pressure(n, tg)
        h_in = CP0 * (273.15 + set_c) + B_COVOL * p / M_H2
        q_wall = sc["ua"] * (tg - tl)
        dtg = (mdot * (h_in - CV0 * tg) - q_wall) / (m * CV0)
        dtl = (q_wall - sc["ua_amb"] * (tl - sc["t_amb"])) / sc["c_liner"]
        n += mdot / M_H2 * dt
        tg += dtg * dt
        tl += dtl * dt
        t += dt
        out.kwh += precool_kwh(mdot * dt, set_c, sc["t_amb"])
        if tg > T_CEILING_K:
            out.breaches[0] += dt
        if pressure(n, tg) > P_CEILING:
            out.breaches[1] += dt
        if tl > T_LINER_CEILING_K:
            out.breaches[2] += dt
        if dtl > RAMP_CAP:
            out.breaches[3] += dt
        out.peak_gas_c = max(out.peak_gas_c, tg - 273.15)
        out.peak_liner_c = max(out.peak_liner_c, tl - 273.15)
        if n >= target_n:
            out.reached = True
            break
    out.minutes = t / 60.0
    return out


# Declared SAFE FALLBACK: the minimum-enthalpy-injection command (lowest
# mass-flow tier, coldest pre-cool setpoint). A deployed dispenser must have
# a defined command for every reachable state, so states with no gate-clean
# action get this one; the benchmark measures how often it is invoked and
# whether it ever breaches.
SAFE_FALLBACK = (FLOW_TIERS[0], PRECOOL_C[0])


def dp_controller(act, stats):
    def ctrl(n, tg, tl, t, band, held):
        if held is not None:
            return held
        ai = act[sid(band, gas_band(tg), lin_band(tl))]
        if ai < 0:
            stats["fallback"] += 1
            return SAFE_FALLBACK
        fi, ci = ACTIONS[ai]
        return (FLOW_TIERS[fi], PRECOOL_C[ci])
    return ctrl


def table_method_controller():
    """Incumbent (a): J2601-style fixed APRR + T40 (-40 degC) pre-cooling.

    A fixed average pressure ramp rate tracked by a proportional mass-flow
    controller -- how a station actually realizes an APRR.
    """
    p0 = [None]

    def ctrl(n, tg, tl, t, band, held):
        if p0[0] is None:
            p0[0] = pressure(n, tg)
        err = (p0[0] + APRR * t) - pressure(n, tg)
        return (min(STATION_MAX_FLOW, max(1e-4, APRR_GAIN * err)), -40.0)
    return ctrl


def greedy_controller(hot_c):
    """Incumbent (b): max-flow-until-hot, cheapest pre-cooling first.

    Reactive on the TRUE gas temperature, with both tier ladders available.
    """
    st = {"fi": len(FLOW_TIERS) - 1, "ci": len(PRECOOL_C) - 1}

    def ctrl(n, tg, tl, t, band, held):
        tc = tg - 273.15
        if tc > hot_c and st["ci"] > 0:
            st["ci"] -= 1
        if tc > hot_c + 7.0 and st["fi"] > 0:
            st["fi"] -= 1
        return (FLOW_TIERS[st["fi"]], PRECOOL_C[st["ci"]])
    return ctrl


def best_greedy(sc):
    """Oracle-tuned greedy: the best FEASIBLE threshold for this scenario."""
    best = None
    for thr in GREEDY_THRESHOLDS:
        r = replay(sc, greedy_controller(thr))
        key = (0 if r.feasible else 1, r.objective())
        if best is None or key < best[0]:
            best = (key, r, thr)
    return best[1], best[2]


# ---------------------------------------------------------------------------
# Scenario corpus
# ---------------------------------------------------------------------------
def gen_scenario(seed):
    s = seed
    s, r = splitmix(s)
    t_amb_c = [15.0, 25.0, 30.0, 35.0, 40.0][r % 5]
    s, r = splitmix(s)
    soc0 = [0.05, 0.08, 0.12, 0.20][r % 4]
    s, r = splitmix(s)
    ua = [300.0, 400.0, 500.0, 600.0, 700.0][r % 5]
    s, r = splitmix(s)
    c_liner = [0.9e5, 1.1e5, 1.3e5, 1.6e5][r % 4]
    return {"t_amb": 273.15 + t_amb_c, "t_amb_c": t_amb_c, "soc0": soc0,
            "ua": ua, "ua_amb": 12.0, "c_liner": c_liner}


def main():
    print("R3 HYDROGEN-FASTFILL falsifier -- exact DP vs J2601-style table "
          "method and TUNED greedy max-flow-until-hot")
    print(f"tank {V_TANK*1000:.0f} L, 100% SoC = {N_FULL*M_H2:.2f} kg H2 at "
          f"{P_NWP/1e5:.0f} bar / {T_REF_SOC-273.15:.0f} degC; "
          f"{N_STATES} states x {len(ACTIONS)} actions")
    print(f"objective J = {ALPHA}*minutes + {BETA}*kWh_precool; gates: gas "
          f"{T_CEILING_K-273.15:.0f} degC, {P_CEILING/1e5:.0f} bar, liner "
          f"{T_LINER_CEILING_K-273.15:.0f} degC, {RAMP_CAP} K/s ramp\n")

    n = 12
    fallbacks = 0
    dp_feas = tbl_feas = grd_feas = 0
    dp_beats_tbl = dp_beats_grd = grd_parity = 0
    kwh_win = 0
    best_gap = None
    rows = []
    for k in range(1, n + 1):
        sc = gen_scenario(k * 7919)
        _, act = exact_dp(characterize(sc))
        stats = {"fallback": 0}
        r_dp = replay(sc, dp_controller(act, stats))
        fallbacks += stats["fallback"]
        r_tb = replay(sc, table_method_controller())
        r_gr, thr = best_greedy(sc)
        dp_feas += r_dp.feasible
        tbl_feas += r_tb.feasible
        grd_feas += r_gr.feasible
        if r_dp.feasible and (not r_tb.feasible
                              or r_dp.objective() < r_tb.objective() - 1e-9):
            dp_beats_tbl += 1
        if r_dp.feasible and r_tb.feasible and r_dp.kwh < r_tb.kwh - 1e-9:
            kwh_win += 1
        if r_dp.feasible and (not r_gr.feasible
                              or r_dp.objective() < r_gr.objective() - 1e-9):
            dp_beats_grd += 1
            if r_gr.feasible:
                gap = r_gr.objective() - r_dp.objective()
                if best_gap is None or gap > best_gap[0]:
                    best_gap = (gap, k, r_dp, r_gr)
        if r_gr.feasible and (not r_dp.feasible
                              or r_gr.objective() <= r_dp.objective() + 1e-9):
            grd_parity += 1
        rows.append((k, sc, r_dp, r_tb, r_gr, thr))

    hdr = (f"{'sc':>3} {'amb':>4} {'soc0':>5} {'UA':>4} {'Cl':>6} | "
           f"{'DPmin':>6} {'kWh':>5} {'J':>6} {'gasC':>5} {'ok':>2} | "
           f"{'TBmin':>6} {'kWh':>5} {'J':>6} {'gasC':>5} {'ok':>2} | "
           f"{'GRmin':>6} {'kWh':>5} {'J':>6} {'gasC':>5} {'ok':>2} {'thr':>4}")
    print(hdr)
    for k, sc, a, b, c, thr in rows:
        print(f"{k:>3} {sc['t_amb_c']:>4.0f} {sc['soc0']:>5.2f} "
              f"{sc['ua']:>4.0f} {sc['c_liner']:>6.0f} | "
              f"{a.minutes:>6.2f} {a.kwh:>5.2f} {a.objective():>6.2f} "
              f"{a.peak_gas_c:>5.1f} {'Y' if a.feasible else 'N':>2} | "
              f"{b.minutes:>6.2f} {b.kwh:>5.2f} {b.objective():>6.2f} "
              f"{b.peak_gas_c:>5.1f} {'Y' if b.feasible else 'N':>2} | "
              f"{c.minutes:>6.2f} {c.kwh:>5.2f} {c.objective():>6.2f} "
              f"{c.peak_gas_c:>5.1f} {'Y' if c.feasible else 'N':>2} "
              f"{thr:>4.0f}")

    print(f"\ncorpus: {n} seeded fill scenarios "
          f"(ambient x initial SoC x UA x wall heat capacity)")
    print(f"feasible: DP {dp_feas}/{n} | table method {tbl_feas}/{n} | "
          f"tuned greedy {grd_feas}/{n}")
    print(f"DP beats J2601-style table method  : {dp_beats_tbl}/{n}")
    print(f"DP uses less pre-cool kWh than tbl : {kwh_win}/{n}")
    print(f"DP beats TUNED greedy              : {dp_beats_grd}/{n}")
    print(f"tuned greedy reaches parity with DP: {grd_parity}/{n}")
    print(f"safe-fallback invocations (DP)     : {fallbacks}")
    if best_gap:
        gap, k, a, c = best_gap
        print(f"largest cost gap where BOTH feasible: scenario {k}, "
              f"DP J={a.objective():.2f} ({a.minutes:.2f} min, "
              f"{a.kwh:.2f} kWh) vs greedy J={c.objective():.2f} "
              f"({c.minutes:.2f} min, {c.kwh:.2f} kWh), gap {gap:.2f}")
    print(f"\nVERDICT: {'HARNESS DIES' if grd_parity == n else 'HARNESS LIVES'}")


if __name__ == "__main__":
    main()
