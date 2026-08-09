# [QCEOM RL] Hydrogen Fast-Fill Governor — measured benchmark

Closed-loop on the continuous real-gas + lumped-wall model (350 L Type-IV heavy-duty module, 700 bar NWP). Incumbent: the SAE J2601-style lookup-table method (fixed 16 MPa/min APRR + T40 pre-cooling). Criteria frozen in `src/main.rs` and PLAN.md §8 BEFORE this run. Patent posture per PATENT-LANDSCAPE.md (static governed fill-map at runtime; no in-fill adaptation).

Objective J = 1.0·minutes + 3.0·kWh_precool (β derived from the pre-cooler's recovery burden — see `fill_env::BETA`). Terminal SoC 93.75%.

**L12 integration check.** Worst-case gas-temperature time constant τ = m·c_v/(ṁ·c_v+UA) = **3.92 s** (residual SoC, top flow tier); integration step dt = 0.25 s resolves it 15.7× (rule: dt ≤ τ/5). State space 2400 = 16 SoC × 15 gas-T × 10 liner-T bands, 24 actions.

| Ambient | Governor | Fill time | Pre-cool | J | Gas viol. | Press viol. | Liner viol. | Ramp viol. | Peak gas °C | Peak liner °C | Peak bar |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 15 °C | **QCEOM governed map** | 3.13 min | 1.87 kWh | 8.74 | 0 s | 0 s | 0 s | 0 s | 81.7 | 47.1 | 778 |
| 15 °C | J2601-style table method | 4.45 min | 3.21 kWh | 14.09 | 0 s | 0 s | 0 s | 0 s | 58.0 | 36.9 | 729 |
| 15 °C | greedy max-flow (oracle thr 70 °C) | 1.65 min | 2.69 kWh | 9.71 | 0 s | 0 s | 0 s | 0 s | 77.1 | 31.9 | 771 |
| 25 °C | **QCEOM governed map** | 3.27 min | 2.58 kWh | 11.00 | 0 s | 0 s | 0 s | 0 s | 82.7 | 52.8 | 784 |
| 25 °C | J2601-style table method | 4.50 min | 3.80 kWh | 15.90 | 0 s | 0 s | 0 s | 0 s | 62.4 | 43.5 | 739 |
| 25 °C | greedy max-flow (oracle thr 75 °C) | 1.64 min | 3.22 kWh | 11.30 | 0 s | 0 s | 0 s | 0 s | 81.3 | 40.1 | 782 |
| 35 °C | **QCEOM governed map** | 3.38 min | 3.37 kWh | 13.48 | 0 s | 0 s | 0 s | 0 s | 83.1 | 58.5 | 785 |
| 35 °C | J2601-style table method | 4.56 min | 4.38 kWh | 17.70 | 0 s | 0 s | 0 s | 0 s | 66.8 | 50.2 | 748 |
| 35 °C | greedy max-flow (oracle thr 75 °C) | 1.70 min | 3.90 kWh | 13.40 | 0 s | 0 s | 0 s | 0 s | 82.1 | 47.8 | 781 |

## Verdicts against the criteria frozen before the run

| # | Criterion | Verdict | Measured |
|---|---|---|---|
| C1 | Zero hard-gate violations (governed, all ambients) | **PASS** | see table |
| C2 | Fill time ≤ J2601-style table method | **PASS** | worst margin +1.18 min |
| C3 | Pre-cooling kWh strictly lower than table method | **PASS** | worst margin +1.01 kWh |
| C4 | Kernel = plain-DP reference (L9) | **PASS** | worst cost gap 1.78e-15 |
| C5 | Proof pair (ungoverned wins reward AND breaches) | **MISS** | per-ambient |
| C6 | Bit-determinism across repeated solves | **PASS** | 2400 states compared |
| C7 | Solve < 10 s per tank | **PASS** | worst 2.90 s |

Deployable artifact at 25 °C: 2436 B QCH2 image, map fingerprint `0xa0954ab04324380d`, tank hash `0x0723da1ccdc8bb94`, CRC32 `0xe3e6a21e`. Safe-fallback invocations across all governed replays: **1**.

### C5 diagnostic — the proof pair stated on the declared model

The frozen C5 compares the *kernel's regulated rollouts*. The same statement made on the declared model — plain backward induction, no runtime regulator anywhere — is the mechanism-level measurement that names the C5 root cause. Cost is in dispenser-minutes-equivalent; lower is better.

| Ambient | Ungoverned optimum | Governed optimum | What the rulebook costs | Free-path breaches | Governed-path breaches | Verdict |
|---|---|---|---|---|---|---|
| 15 °C | 6.0394 | 6.6920 | +0.6526 | 8 | 0 | **PASS** |
| 25 °C | 7.7919 | 8.6262 | +0.8343 | 10 | 0 | **PASS** |
| 35 °C | 9.5443 | 10.9806 | +1.4363 | 11 | 0 | **PASS** |

Model-level proof pair: **PASS at every ambient**.

## MISSes (reported, not hidden — L2)

- **C5 at 15 °C — MISS.** Kernel-rollout half: the gates-ignored twin's realized reward is -6.9164 against the governed twin's -6.6920 (deficit 0.2244), even though the free twin still SELECTS 4 gated actions and the governed twin selects 0.
- **C5 at 25 °C — MISS.** Kernel-rollout half: the gates-ignored twin's realized reward is -9.0980 against the governed twin's -8.6262 (deficit 0.4718), even though the free twin still SELECTS 4 gated actions and the governed twin selects 0.
- **C5 at 35 °C — MISS.** Kernel-rollout half: the gates-ignored twin's realized reward is -11.0973 against the governed twin's -10.9806 (deficit 0.1167), even though the free twin still SELECTS 4 gated actions and the governed twin selects 0.

**C5 root cause (measured, not conjectured).** The kernel's runtime regulator applies *soft* Dynamic-Resonance damping in addition to the hard gates, and `with_gates_ignored()` only raises the HARD limit to infinity. The soft damping still fires on constraint-stressed actions, and because this domain's rewards are negative (a cost), the damping term `score·(2 − mask)` moves a stressed action's score FURTHER from zero rather than closer. The gates-ignored twin is therefore not ungoverned but *soft-governed*: it still SELECTS gated actions — 4 of them at every ambient, so the constraint demonstrably binds — while realizing a worse reward than the governed twin. As frozen, C5 measures the regulator, not the cost of the rulebook.

**Magnitude.** Reward deficit 0.117–0.472 minutes-equivalent across the three ambients — 1–5 % of the governed total. A small distortion, not a sign error, and it does not touch C1–C4/C6/C7.

**Fix path.** Two options, neither applied retroactively to this run. (i) *Harness side, available now*: state the proof pair on the declared model — `plain_dp_gated(env, false)` versus `plain_dp_gated(env, true)` — which involves no regulator, and which the diagnostic table above measures as PASS at every ambient with the rulebook's true cost quantified. The unit test `proof_pair_ungoverned_is_cheaper_and_breaches` already asserts exactly this. (ii) *Kernel side*: a `DrrConfig` variant that disables soft damping so a gates-ignored twin is genuinely ungoverned. (ii) is a core-kernel change and out of scope for a harness.

**Candidate factory lesson (L15?).** In cost-shaped (negative-reward) domains, `with_gates_ignored()` does NOT produce an ungoverned twin, because DRR soft damping is sign-sensitive. Proof pairs in such domains must be stated at model level, or the criterion must name the regulator explicitly.

## Reading

The governed fill-map exploits the one thing a reactive controller cannot see: the composite wall is a heat sink that **accumulates and does not drain** (τ_wall→ambient ≈ 3 h against a fill of a few minutes). How much of the fill's enthalpy the wall absorbs depends on the ORDER in which warm and cold gas is delivered — rejection is ∫UA·(T_gas−T_liner)dt, so a kilogram of cheap warm hydrogen is worth more when the wall is still cold. The map spends its warm-pre-cool budget where it is cheapest and buys back margin with mass flow where the pressure gate tightens, which is why it undercuts the table method on pre-cooling energy without paying for it in time.

**Honest caveats.** (1) The real-gas model is a DECLARED covolume simplification of NIST REFPROP, not a claim of REFPROP fidelity; (2) tank parameters are representative of a 350 L heavy-duty module, not vendor-identified — per-tank identification replaces them through the same struct and triggers a re-solve by construction (the provenance hash changes); (3) the liner ceiling and thermal-ramp cap are DECLARED limits, not standard values, and a customer's rulebook replaces them; (4) the greedy baseline is reported for context only and is deliberately stronger than any deployable dispenser (perfect in-tank thermometer, oracle-tuned threshold); (5) J2601 compliance is a certification path with an authority and an OEM, never self-awarded; (6) FTO review by counsel is required before commercialization (PATENT-LANDSCAPE.md).

