# [QCEOM RL] Hydrogen Fast-Fill Governor (R3) — Spec & Phased Plan

## QCEOM-RL DOMAIN HARNESS SPEC — the 12-field meta-prompt

1. **DOMAIN & BUYER.** 700 bar heavy-duty hydrogen refuelling. Buyers:
   hydrogen-refuelling-station operators and dispenser/station-control
   vendors (Nel, Linde, Air Liquide class), and HDV fleet depots whose
   economics are throughput-per-dispenser. Component-IP sale into an
   existing station-control stack is the primary commercial thesis.
2. **DECISION PROBLEM.** Choose, at the entry of each state-of-charge band,
   the (mass-flow tier, pre-cool setpoint) pair that carries a Type-IV tank
   across that band, minimizing a two-lever objective — dispenser occupancy
   AND pre-cooling electricity — without ever violating the gas-temperature,
   pressure, or liner-thermal rulebook. Solved OFFLINE per tank model;
   deployed as a static table to a station controller.
3. **STATE ENCODER.** Lattice = (SoC band 0..15 × gas-temperature band 0..14
   × liner-temperature band 0..9) = **2400 states**. Gas bands are 5 K wide
   from 10 °C, liner bands 6 K from 15 °C, and **each grid's top edge is
   aligned with its ceiling** (85 °C / 75 °C). Semi-MDP: one decision
   carries the tank across ONE SoC band and is HELD for the crossing (L13).
4. **ACTION ENCODER.** (mass-flow tier 0..5) × (pre-cool setpoint 0..3) =
   **24 actions**. Flow tiers {15, 30, 50, 70, 95, 120} g/s per 350 L
   module; setpoints {−40, −30, −20, −10} °C (SAE T40/T30/T20/T10).
5. **CHARGE ENCODER.** Repulsors on the hot-gas rows; attractor on the
   completed-fill column. PME grid 32. *(Inert in this harness:
   `shaping_weight = 0.0` per L9 — see field 9.)*
6. **CONSTRAINT DECLARATIONS (hard, reward-neutral).**
   `[0]` **thermal** — T_gas > 85 °C anywhere during the crossing (SAE
   receptacle/tank limit), characterized from the HOT gas edge AND HOT liner
   edge (least wall rejection ⇒ hottest gas).
   `[1]` **pressure** — P > 875 bar (125 % NWP) anywhere, same worst-case
   edges (higher T ⇒ higher P at equal mass).
   `[2]` **structural** — the UNION of a declared liner/resin sustained
   ceiling (75 °C, worst from the HOT liner edge), a declared liner
   thermal-shock cap dT_liner/dt ≤ 0.45 K/s (worst from the **COLD** liner
   edge), and a stalled crossing. L8 applied *within* one gate: its two
   sub-checks have opposite worst-case senses, so it is characterized from
   both liner edges.
7. **CORPUS / MODEL.** Declared real-gas covolume EOS
   `P = nRT/(V − nb)`, b = 1.9e-5 m³/mol — exactly equivalent to
   `Z = 1 + bP/(RT)` and a DECLARED simplification of NIST REFPROP, not a
   claim of REFPROP fidelity. Internal energy `u = c_v0·T` (exact for a
   covolume gas). Inlet enthalpy `h_in = c_p0·T_in + b·P/M` (derived; the
   departure term is ~20 % of h_in at 700 bar and is why H₂ fills get hot).
   Lumped gas + wall energy balances; UA_amb ≈ UA/40, so the composite wall
   accumulates and does not drain. **L12**: worst-case gas time constant
   τ = 3.92 s (residual SoC, top flow tier); dt = 0.25 s resolves it 15.7×.
8. **ACCEPTANCE CRITERIA (frozen before any run; verbatim in
   `src/main.rs`).**
   - **C1** zero hard-gate violations in the governed closed loop at every
     ambient;
   - **C2** governed fill time ≤ the J2601-style table method's;
   - **C3** governed pre-cooling kWh strictly lower than the table method's;
   - **C4** kernel = plain-DP reference to within 1e-9 of declared cost, at
     every ambient (L9 as a standing criterion);
   - **C5** proof pair: the gates-ignored twin BEATS the governed policy on
     reward AND breaches, while the governed policy never does (L5);
   - **C6** bit-determinism across repeated solves;
   - **C7** solve (characterize + train) < 10 s per tank.
   No criterion is declared against the greedy baseline: it is given a
   perfect in-tank thermometer no dispenser has, so it is not a fair
   acceptance bar in either direction. It is reported for context.
9. **ENGINE CONFIG.** γ = **0.9999** (L10: 15-decision undiscounted DAG;
   γ=1.0 collapses the kernel's contraction machinery, γ=0.95 shades the
   late pressure-critical bands). `shaping_weight = **0.0**` (L9: with PME
   shaping off the kernel reproduces the plain-DP optimum exactly). All
   other kernel defaults.
10. **DEPLOYMENT TIER.** Hybrid. Hosted solve (< 4 s per tank); the map is
    packed into a 2436-byte `QCH2` provenance image (magic / version /
    dispenser serial / tank+rulebook hash / map fingerprint / 2400 action
    bytes / CRC32) and executed by the NOSTD twin on a station-controller
    MCU. A dispenser controller **is** the N657 form factor.
11. **EVIDENCE PLAN.** `falsifier/falsifier.py` verdict reported either way;
    `DOMAIN-BENCHMARK.md` with the criteria table and measured values; proof
    pair; determinism; L2 MISS policy — magnitude, root cause, fix path,
    never hidden.
12. **REPO.** `[QCEOM-RL]-HYDROGEN-FASTFILL-GOVERNOR` (GitHub sanitizes to
    `-QCEOM-RL--HYDROGEN-FASTFILL-GOVERNOR`) + the `-NOSTD` twin.
    **PATENT POSTURE (PATENT-LANDSCAPE.md):** static governed fill-map at
    runtime — no in-fill adaptive feedback loop, no online thermal observer,
    no hardware claim; adaptation only between sessions via
    re-characterize → re-solve → re-burn. FTO by counsel AND a
    protocol-acceptance path with the operator and OEM before
    commercialization.

---

## Phases (DONE only with evidence)

- **P0 GATE-ZERO — DONE.** `PATENT-LANDSCAPE.md`, written before any code.
  Findings: the protocol layer is a *standard* (J2601 lookup-table and
  MC-formula methods) and therefore strong prior art; the live claim mass is
  in adaptive, in-fill, measurement-driven control (US12313224, US12429852,
  US11920736, US10451219) and in pre-cooling *hardware* (Linde ionic
  compressor / cryo pump, US10724767); offline optimal-policy synthesis is
  publication-rich and patent-thin. Verdict **PROCEED** under the
  design-around posture; commercialization blocked on FTO **and** protocol
  acceptance.

- **P1 FALSIFIER — DONE, HARNESS LIVES.** `falsifier/falsifier.py`, measured
  2026-08-09, 12 seeded scenarios (ambient × initial SoC × UA × wall heat
  capacity), deterministic splitmix64, no RNG library:
  - exact DP feasible **12/12**;
  - DP beats the J2601-style table method **12/12** on the objective and
    **12/12** on pre-cooling kWh;
  - DP beats the **oracle-tuned** greedy max-flow-until-hot **7/12** (largest
    gap: scenario 6, J 6.61 vs 7.86);
  - greedy reaches parity **5/12** — recorded as an honest caveat, and note
    that this greedy is given the true in-tank gas temperature with zero lag,
    which no real dispenser has.
  - Kill criterion (greedy at parity on EVERY scenario) not met.
  - **The falsifier also killed two design defects before they reached
    Rust**, which is the falsifier working as intended: (i) worst-case-edge
    *transitions* compound over a 15-decision horizon and ratchet the DP into
    dead states (measured 0/24 feasible) — fixed by taking gates from
    worst-case edges and transitions from band centres, which preserves the
    safety guarantee exactly; (ii) a band whose hot edge sits above its
    ceiling is unconditionally dead *and reachable*, stranding the closed
    loop — fixed by aligning every gated grid's top edge with its ceiling.

- **P2 PHYSICS — DONE.** `src/thermo.rs`; 6 tests including the L12
  dt-vs-τ check, an adiabatic energy-balance closure to < 1e-6 relative, and
  the "the danger is real" pair (un-pre-cooled fast fill must exceed 85 °C;
  T20 fast fill must breach; pre-cooling is monotone).

- **P3 MDP — DONE.** `src/fill_env.rs`; characterization physicality, clean
  governed rollout, model-level proof pair, kernel = plain-DP, determinism.

- **P4 BENCHMARK — DONE, 6 of 7 criteria PASS.** See `DOMAIN-BENCHMARK.md`.
  C1/C2/C3/C4/C6/C7 PASS; **C5 MISS**, reported with magnitude, measured
  root cause (DRR *soft* damping is sign-sensitive, so
  `with_gates_ignored()` does not produce a genuinely ungoverned twin in a
  cost-shaped domain), and fix path. The same proof pair stated on the
  declared model — where no regulator is involved — PASSes at every ambient
  and quantifies what the rulebook costs (+0.65 / +0.83 / +1.44
  minutes-equivalent).

- **P5 PROVENANCE — DONE.** `src/image.rs`, 2436-byte `QCH2` image,
  fail-closed magic → version → CRC32 → fingerprint → provisioned tank hash.
  Known-vector CRC (`0xCBF43926`), round-trip, and four refusal paths tested.

- **P6 NOSTD + QEMU — DONE.** `-NOSTD` twin: 5/5 host tests, **4/4 on QEMU
  mps3-an547 / Cortex-M55**, map fingerprint `0xa0954ab04324380d` identical
  on x86-64 and M55.

- **P7 SILICON — STAGED, NOT RUN.** `mailbox_burn.bin` built
  (13,380 B, reset vector `0x3410_07C1`) with the exact run sequence in
  `docs/N657-RUN.md`. The board is a shared resource; board access is
  coordinated by the lead dev, so no physical run is claimed.

- **P8 SHIP — DONE.** README, PLAN, benchmark, both repos pushed.

## Candidate factory lessons from this harness

- **L15?** In cost-shaped (negative-reward) domains, `with_gates_ignored()`
  does NOT yield an ungoverned twin: DRR soft damping is sign-sensitive
  (`score·(2 − mask)` pushes a negative score further from zero). Proof
  pairs in such domains must be stated at model level, or the criterion must
  name the regulator explicitly.
- **L16?** Worst-case band-edge characterization must be split: **gates**
  from the worst edge (safety, evaluated at the band actually entered),
  **transitions** from the band centre. Applying the worst edge to the
  transition compounds over the horizon and makes the DP vacuously
  infeasible. Safety does not depend on the transition model.
- **L17?** Align every gated dimension's top band edge with its ceiling. A
  band whose worst edge exceeds the ceiling is unconditionally dead and is
  reachable, which strands the closed loop mid-episode.
