# [QCEOM RL] HYDROGEN-FASTFILL-GOVERNOR

**A governed, exactly-solved fill map for 700 bar heavy-duty hydrogen
refuelling.** Given a declared tank model and a declared safety rulebook, an
exact dynamic program computes, offline, the (mass-flow tier, pre-cool
setpoint) command for every (SoC × gas-temperature × liner-temperature) band
the fill can occupy — and the resulting 2436-byte table *cannot* command an
action that breaches the rulebook, because gated actions are excluded from
the Bellman maximization rather than penalized inside it.

Embedded twin: [-QCEOM-RL--HYDROGEN-FASTFILL-GOVERNOR-NOSTD](https://github.com/LUXERON/-QCEOM-RL--HYDROGEN-FASTFILL-GOVERNOR-NOSTD)
(fail-closed `QCH2` image validator + dispenser-side executor, verified on
emulated Cortex-M55).

---

## Measured benchmark

350 L Type-IV heavy-duty module, 700 bar NWP, filled from a 5 % residual to
93.75 % SoC. Incumbent: the SAE J2601-style **lookup-table method** — a
fixed 16 MPa/min average pressure ramp rate plus T40 (−40 °C) pre-cooling
for the whole fill, which is the currently deployed standard. Closed-loop on
the continuous model; the banded model is used only for solving and gating.

Objective **J = 1.0·minutes + 3.0·kWh_precool**, where β = 3.0 min/kWh is
*derived*, not tuned: the pre-cooler is the throughput bottleneck for
back-to-back heavy-duty fills, and 1 kWh_e × COP 0.8 = 0.8 kWh_th takes a
16 kW chiller 3.0 min to recover.

| Ambient | Governor | Fill time | Pre-cool | J | Violations | Peak gas | Peak liner | Peak |
|---|---|---|---|---|---|---|---|---|
| 15 °C | **QCEOM governed map** | **3.13 min** | **1.87 kWh** | **8.74** | none | 81.7 °C | 47.1 °C | 778 bar |
| 15 °C | J2601-style table method | 4.45 min | 3.21 kWh | 14.09 | none | 58.0 °C | 36.9 °C | 729 bar |
| 15 °C | greedy max-flow (oracle 70 °C) | 1.65 min | 2.69 kWh | 9.71 | none | 77.1 °C | 31.9 °C | 771 bar |
| 25 °C | **QCEOM governed map** | **3.27 min** | **2.58 kWh** | **11.00** | none | 82.7 °C | 52.8 °C | 784 bar |
| 25 °C | J2601-style table method | 4.50 min | 3.80 kWh | 15.90 | none | 62.4 °C | 43.5 °C | 739 bar |
| 25 °C | greedy max-flow (oracle 75 °C) | 1.64 min | 3.22 kWh | 11.30 | none | 81.3 °C | 40.1 °C | 782 bar |
| 35 °C | **QCEOM governed map** | **3.38 min** | **3.37 kWh** | 13.48 | none | 83.1 °C | 58.5 °C | 785 bar |
| 35 °C | J2601-style table method | 4.56 min | 4.38 kWh | 17.70 | none | 66.8 °C | 50.2 °C | 748 bar |
| 35 °C | greedy max-flow (oracle 75 °C) | 1.70 min | 3.90 kWh | **13.40** | none | 82.1 °C | 47.8 °C | 781 bar |

**Against the incumbent it is asked to replace, at every ambient: 26–30 %
faster and 23–42 % less pre-cooling electricity, with zero violations.**

The greedy row is context, not a scoreboard — and it is reported honestly:
it wins on J at 35 °C by 0.6 %. It is also given a perfect in-tank gas
thermometer with zero lag, which no dispenser has (the absence of that
sensor is the entire reason SAE J2601 and the MC formula exist), and its hot
threshold is oracle-tuned per tank. No acceptance criterion was declared
against it, before or after the run.

### Criteria, frozen before the run

| # | Criterion | Verdict | Measured |
|---|---|---|---|
| C1 | Zero hard-gate violations (governed, all ambients) | **PASS** | 0 s on all four gates |
| C2 | Fill time ≤ J2601-style table method | **PASS** | worst margin +1.18 min |
| C3 | Pre-cool kWh strictly lower than table method | **PASS** | worst margin +1.01 kWh |
| C4 | Kernel = plain-DP reference (L9) | **PASS** | worst cost gap 1.8e-15 |
| C5 | Proof pair (ungoverned wins reward AND breaches) | **MISS** | see below |
| C6 | Bit-determinism across repeated solves | **PASS** | 2400 states compared |
| C7 | Solve < 10 s per tank | **PASS** | worst 2.9 s (wall clock; the only value in this table that is not bit-reproducible) |

**The C5 MISS, in full.** As frozen, C5 compares the *kernel's regulated
rollouts*: the gates-ignored twin must beat the governed twin on reward. It
does not — it realizes 0.117–0.472 minutes-equivalent *worse* reward (1–5 %)
while still selecting 4 gated actions per fill. Root cause, measured rather
than conjectured: `with_gates_ignored()` only raises the **hard** limit to
infinity, but the kernel's runtime regulator also applies **soft**
Dynamic-Resonance damping, and that damping is sign-sensitive — for a
negative score (this domain's rewards are costs) the term `score·(2 − mask)`
moves a constraint-stressed action *further* from zero. The twin is
therefore soft-governed, not ungoverned, so C5-as-frozen measures the
regulator rather than the cost of the rulebook.

The same proof pair stated on the **declared model**, where no regulator
exists at all, PASSes at every ambient and quantifies exactly what the
rulebook costs:

| Ambient | Ungoverned optimum | Governed optimum | Cost of the rulebook | Free-path breaches | Governed-path breaches |
|---|---|---|---|---|---|
| 15 °C | 6.0394 | 6.6920 | +0.6526 | 8 | 0 |
| 25 °C | 7.7919 | 8.6262 | +0.8343 | 10 | 0 |
| 35 °C | 9.5443 | 10.9806 | +1.4363 | 11 | 0 |

Fix path: state the proof pair at model level (available now, and what the
unit test asserts), or add a `DrrConfig` variant to the kernel that disables
soft damping — a core change, out of scope for a harness. Candidate factory
lesson recorded in `PLAN.md`.

---

## How it works

### 1. The declared physics (`src/thermo.rs`)

Hydrogen's compressibility deviates *upward* at fuelling pressure. We
declare the simplest form that captures it, `Z = 1 + bP/(RT)` with
b = 1.9e-5 m³/mol, which rearranges **exactly** into a covolume EOS:

```
P = n·R·T / (V − n·b)
```

This is a **DECLARED SIMPLIFICATION of NIST REFPROP, not a claim of REFPROP
fidelity**. It is chosen because one constant reproduces the headline
deviation (37.9 vs ≈39 kg/m³ at 700 bar / 15 °C) *and* because a covolume
gas has `(∂u/∂V)_T = 0` exactly, so `u = c_v0·T` with no departure function.
Every departure that matters then lands where it physically belongs — the
inlet stream:

```
h_in = c_p0·T_in + b·P_tank / M
```

At 700 bar that second term is ~660 kJ/kg, about 20 % of the inlet
enthalpy. Dropping it would have understated the fill's heating by roughly a
factor of two; hydrogen's negative Joule–Thomson coefficient above ~200 K is
precisely why fast fills get hot.

Lumped energy balances for the gas control volume and a single wall node:

```
m·c_v·dT_gas/dt     = ṁ·(h_in − u_gas) − UA·(T_gas − T_liner)
C_liner·dT_liner/dt = UA·(T_gas − T_liner) − UA_amb·(T_liner − T_amb)
```

**The Type-IV composite's high thermal resistance to ambient is the whole
point.** UA_amb is ~40× smaller than UA, so τ_wall→ambient ≈ 3 h against a
fill of a few minutes: heat pushed into the wall does not leave. The wall
temperature is an accumulating, shared, history-dependent resource.

**L12 check (mandatory).** The stiffest gated dynamic is the gas
temperature, τ = m·c_v/(ṁ·c_v + UA), smallest at minimum mass and maximum
flow: **τ = 3.92 s** at 5 % SoC and 120 g/s. The integration step
dt = 0.25 s resolves it 15.7× (rule: dt ≤ τ/5). Enforced by a test, not a
comment.

### 2. The semi-MDP (`src/fill_env.rs`)

2400 states = 16 SoC × 15 gas-T × 10 liner-T bands; 24 actions = 6 flow
tiers × 4 pre-cool setpoints. One decision carries the tank across one SoC
band and is **held** for the crossing (L13 — that is what a band-indexed
table means when deployed, and it is also the patent posture).

Each (state, action) is characterized by integrating the true model three
times:

| run | start edges | decides |
|---|---|---|
| HOT/HOT | gas hot edge, liner hot edge | gas-ceiling, pressure, liner-ceiling gates |
| HOT/COLD | gas hot edge, liner **cold** edge | liner thermal-ramp gate |
| CENTRE | both band centres | next state and duration |

The per-constraint worst-case *sense* (L8) is applied twice over: across
gates, and *within* the structural gate, whose two sub-checks — an absolute
liner ceiling and a liner temperature-gradient cap — have opposite
worst-case liner edges.

Two design defects the falsifier caught before any Rust was written, both
now structural properties of the harness:

- **Worst-case transitions compound.** Re-applying the hot edge at each of
  15 crossings ratchets ~half a band per step until every path is gated
  (measured: DP feasible 0/24). Gates come from worst-case edges;
  transitions come from band centres. Safety is unaffected — the gate is
  evaluated at whatever band the closed loop actually enters, from *that*
  band's worst edge — so only optimality ever depended on the transition
  model, and the closed-loop benchmark is the check on that.
- **Band grids must end on their ceilings.** A band whose hot edge sits
  above its ceiling is unconditionally dead *and* reachable (any legal
  crossing may end inside it), which strands the fill mid-way. The gas grid
  therefore ends exactly at 85 °C and the liner grid exactly at 75 °C.

### 3. The rulebook (hard gates, reward-neutral)

| gate | limit | authority |
|---|---|---|
| thermal | T_gas ≤ 85 °C anywhere in the crossing | SAE J2601 receptacle/tank limit |
| pressure | P ≤ 875 bar | 125 % of NWP |
| structural | T_liner ≤ 75 °C **and** dT_liner/dt ≤ 0.45 K/s **and** no stall | **DECLARED**, not standard values |

Safety contributes nothing to the reward. The only thing distinguishing the
governed policy from the free one is that gated actions are not in the
maximization.

### 4. The deployable artifact (`src/image.rs`)

A 2436-byte `QCH2` image: magic `0x3248_4351` · format version · dispenser
serial · tank+rulebook hash · map fingerprint · 2400 action bytes · CRC32.
Validation is fail-closed and ordered — magic → version → CRC →
fingerprint → provisioned tank hash. Re-identify the tank, or revise a
single limit or objective weight, and the hash changes and the stale map is
refused before a kilogram moves.

---

## Why dynamic programming, and the falsifier that had to say so first

The kill criterion was declared before the experiment: *if a greedy
max-flow-until-hot controller reaches parity with exact DP on every
scenario, the kernel adds nothing and the harness dies.*

Measured over 12 seeded scenarios (ambient × initial SoC × wall conductance
× wall heat capacity), with the greedy handed a perfect in-tank thermometer
and an oracle-tuned threshold:

```
feasible: DP 12/12 | table method 12/12 | tuned greedy 12/12
DP beats J2601-style table method  : 12/12
DP uses less pre-cool kWh than tbl : 12/12
DP beats TUNED greedy              : 7/12
tuned greedy reaches parity with DP: 5/12
VERDICT: HARNESS LIVES
```

The mechanism: how much of a fill's enthalpy the wall absorbs is
`∫UA·(T_gas − T_liner) dt`, so a kilogram of cheap warm hydrogen is worth
more *when the wall is still cold*. That makes the fill's thermal budget
order-dependent and non-recoverable. A reactive controller prices none of
it; DP prices the entire remaining fill at every decision.

---

## Patent and standards posture (read before any commercial conversation)

Full scan in [PATENT-LANDSCAPE.md](PATENT-LANDSCAPE.md). In short:

- The deployed artifact is a **static governed fill-map**, read at band entry
  and held — the *shape* of the J2601 lookup-table method (standardized,
  safe art). There is **no in-fill adaptive feedback loop**, no online
  thermal observer, and **no hardware claim**. That is deliberate: the live
  claim mass sits on adaptive, measurement-driven, in-fill control and on
  pre-cooling hardware.
- Adaptation happens **between** sessions: re-characterize → re-solve →
  re-fingerprint → re-burn. The image is the unit of change.
- **Two standing blockers for commercialization**: (1) formal FTO review by
  counsel; (2) a protocol-acceptance path agreed with the station operator
  and the receiving-vehicle OEM. J2601 compliance is a certification
  question with an authority, never self-awarded.

## Honest caveats

1. The real-gas model is a declared covolume simplification, not REFPROP.
2. Tank parameters are representative of a 350 L heavy-duty module, not
   vendor-identified. Identification replaces them through the same struct
   and forces a re-solve by construction (the provenance hash changes).
3. The liner ceiling and thermal-ramp cap are **declared** limits, not
   standard values. A customer's rulebook replaces them.
4. The greedy baseline is context only and is stronger than any deployable
   dispenser.
5. Nothing here has been run against a physical tank or a real dispenser.

## Reproduce

```bash
# the falsifier (pure Python, deterministic, no RNG library)
python falsifier/falsifier.py

# 19 tests: physics, MDP, incumbents, provenance image
CARGO_NET_GIT_FETCH_WITH_CLI=true cargo test --release

# the benchmark — rewrites DOMAIN-BENCHMARK.md
CARGO_NET_GIT_FETCH_WITH_CLI=true cargo run --release --bin h2-bench

# the golden cross-target vector for the NOSTD twin
CARGO_NET_GIT_FETCH_WITH_CLI=true cargo run --release --bin emit_test_vector
```

## Layout

```
PATENT-LANDSCAPE.md   P0 — written before any code
PLAN.md               the 12-field spec, phases, candidate factory lessons
falsifier/            P1 — the experiment that could have killed this
src/thermo.rs         P2 — declared real-gas + lumped-wall model, L12 check
src/fill_env.rs       P3 — the semi-MDP, gates, plain-DP reference
src/incumbent.rs      P4 — J2601-style table method + greedy baselines
src/main.rs           P4 — criteria frozen in a comment block, then scored
src/image.rs          P5 — the QCH2 provenance image
DOMAIN-BENCHMARK.md   P4 — measured, regenerated by the bench binary
```

Part of the QCEOM-RL domain-harness factory (R3). One invariant kernel; the
harness is the product.


## Physical silicon — triple-target closed

Measured on an STM32N6570-DK on 2026-08-09: **4 passed, 0 failed**
(mailbox `QH2F`, status 2). The map fingerprint
`0xa0954ab04324380d` is identical on x86-64, QEMU mps3-an547 and
physical STM32N657. Total ≈ 4.29 M cycles ≈ **67 ms @ 64 MHz**,
including every fail-closed refusal. Full mailbox decode:
[docs/N657-RUN.md](docs/N657-RUN.md) in the NOSTD repo.
