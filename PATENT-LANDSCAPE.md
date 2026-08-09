# Patent Landscape Scan — Hydrogen Fast-Fill Governor (R3)

**Purpose:** the factory's GATE-ZERO feasibility gate for this harness — a
cheapest-falsifying scan of the hydrogen-refuelling control thicket *before*
any code is written. **This is engineering strategy, NOT legal advice; a
formal freedom-to-operate (FTO) opinion by patent counsel is required before
any commercialization.** Scan date: 2026-08-09.

---

## Finding 1 — The protocol layer is a STANDARD, and the standard is the art

SAE J2601 defines two fuelling methods for 70 MPa gaseous hydrogen:

- **Lookup-table method** — a fixed *Average Pressure Ramp Rate* (APRR)
  selected from a table indexed by ambient temperature, initial tank
  pressure, tank volume category and station pre-cooling category (T40 =
  −40 °C, T30, T20). APRR does not change during the fill, so the fill
  duration is a constant once the table row is picked.
- **MC-formula method** — Honda's contribution, standardized by SAE: a
  dynamic pressure ramp rate continuously recomputed during the fill from
  the *measured* pre-cooling temperature and a lumped "MC" (mass ×
  specific-heat) characterization of the receiving tank, which yields
  faster fills, particularly at high ambient temperature.

Both are published, balloted standards documents. That cuts two ways:

1. **They are prior art of the strongest kind.** Anyone attempting to fence
   "compute a pressure ramp from a table" or "compute a ramp from a lumped
   thermal-mass formula" now faces J2601 itself.
2. **They are also a compliance surface, not just a patent surface.** A
   station that does not implement a recognized protocol has a *certification*
   problem long before it has an infringement problem. This is a real
   commercial constraint on the harness and is stated in the posture below.

J2601/5 extends prescriptive high-flow protocols to medium- and heavy-duty
vehicles — the segment this harness targets.

## Finding 2 — The claim mass sits in ADAPTIVE, IN-FILL, MEASUREMENT-DRIVEN control

The granted claims that actually bite are the ones that (a) *measure*
something during the fill (tank temperature, pre-cooling temperature,
mass-average temperature, flow) and (b) *modify* the fill in response,
usually inside apparatus claims that recite the sensor set. Representative
live art surfaced in this scan:

| Reference | Center of gravity |
|---|---|
| **US12313224** — *Hydrogen filling method, hydrogen filling apparatus, program, and record medium* (2025) | flow meter + temperature sensor + pressure sensor + valves controlling hydrogen flow during the fill |
| **US12429852** — *Hydrogen filling apparatus and hydrogen filling method* | explicitly recites the compression-heat temperature rise in the tank and an **allowable-temperature threshold** driving the fill |
| **US11920736** — *Method and system for filling tanks of hydrogen-fueled vehicles* (filed 2021) | mixing vaporized + liquid H₂ streams under valve control to hit a low dispense temperature for fast fills |
| **US10451219** (Air Liquide) — *Method and device for filling a hydrogen tank* | filling method/device claims from the industrial-gas incumbent |
| **US10724767** — *High-pressure hydrogen filling system with expansion turbine* | pre-cooling hardware architecture |
| **US10960783** (Honda) — *Communication systems and methods for hydrogen fueling and electric charging* | the SAE J2799 IrDA vehicle→station data channel that in-fill adaptation depends on |
| Linde ionic-compressor and cryo-pump portfolios; Air Liquide station portfolios | **hardware**: compression, pre-cooling, cryogenic pumping |

Two structural observations:

- **The hardware thicket (compressors, chillers, cryo-pumps, expansion
  turbines) is dense and is not our layer.** We command a *setpoint*; we do
  not claim a machine that achieves it.
- **The control thicket is concentrated on closed-loop, in-fill adaptation**
  — precisely the Qnovo-shaped pattern already met in the fast-charge
  program (H2): *measure the receiver's response, modify the delivery*.

## Finding 3 — Offline optimal-policy synthesis for fuelling is publication-rich, patent-thin

Searches surface active academic work on optimizing J2601 fills (modified MC
formula estimation, neural-network MC parameter estimation, CFD pre-cooling
studies) but no granted claims found in this scan on *the method class of
solving the fill policy offline by exact dynamic programming and deploying
the result as a static table*. As with fast charge, that cuts favourably
twice: the space we occupy is not visibly fenced, and the publication
density limits anyone's ability to fence it broadly later.

---

## The design posture this harness adopts (design-around by architecture)

1. **Static governed fill-map at runtime; no in-fill adaptive feedback
   loop.** The deployed artifact is a precomputed
   (SoC band × gas-temperature band × liner-temperature band) →
   (mass-flow tier, pre-cool setpoint) lookup table. That is the *shape* of
   the J2601 lookup-table method (old, standardized, safe art) — a table
   indexed by the fill's state — differing only in *how the table's entries
   were computed offline*. During a fill the dispenser reads the table at
   band entry and **holds** the commanded tier for the whole band crossing
   (the semi-MDP contract, L13). It does not estimate, identify, or adapt.
2. **Solved offline, re-solved between sessions.** Adaptation happens
   *between* fills, never inside one: re-characterize the tank parameters →
   re-solve → re-fingerprint → re-burn the image. This is deliberately the
   same lifecycle posture that cleared the fast-charge scan.
3. **Declared characterized limits, not an online observer.** The 85 °C
   gas-temperature gate, the 87.5 MPa pressure gate and the liner thermal-
   ramp gate are evaluated against a model characterized at *solve* time
   from worst-case band edges — not from a runtime state estimator or a
   virtual thermocouple. We steer clear of observer-implementation claims
   (and of the vehicle-communication claims: the map is indexed by station-
   side state, and any vehicle data merely selects which pre-solved map is
   loaded).
4. **No hardware claim.** Chillers, compressors, expansion turbines,
   cryo-pumps and dispensers are the incumbents' territory and we do not
   enter it. The pre-cool "action" is a setpoint request to whatever
   pre-cooler the station already owns.
5. **The claimable novelty we keep** (and should consider filing on):
   exact-DP fill-policy synthesis with the safety rulebook excluded from
   the Bellman maximization itself (governance hard gates — the policy
   *cannot* select a gated action, rather than being penalized for it);
   per-constraint worst-case band-edge characterization by sensitivity
   sign; bit-reproducible cross-target policy artifacts with a fail-closed
   provenance image; and the between-session re-solve cadence as a
   lifecycle. None of these appeared in the claims surveyed.

### Standards posture (separate from, and additional to, FTO)

J2601 compliance is a **certification** question, not a patent question, and
it is not self-awardable. The honest framing for any buyer conversation:

> This harness produces a *station-side fill map* under a declared tank
> model and a declared rulebook. Deployment into a public retail dispenser
> requires the map to be shown compliant with — or accepted as an
> alternative to — the applicable J2601/J2601-5 protocol by the relevant
> authority and the OEM. The harness's contribution to that conversation is
> that the map is deterministic, replayable, and provably cannot command a
> gated action; it is not a claim of compliance.

---

## GATE-ZERO verdict

**PROCEED** for research, benchmarking and internal evaluation under the
posture above.

**Standing blocking conditions for commercialization (both required):**

1. Formal FTO review by patent counsel of, at minimum, US12313224,
   US12429852, US11920736, US10451219, US10724767, US10960783 and their
   continuations, plus the Air Liquide / Linde / Nel station-control
   families.
2. Protocol-acceptance path agreed with the station operator and the
   receiving-vehicle OEM before any fill of a real vehicle tank.

This document is a standing chapter of the harness's whitepaper and README.

---

## Sources

- [SAE J2601 — Fueling Protocols for Light Duty Gaseous Hydrogen Surface Vehicles (H2tools)](https://h2tools.org/fuel-cell-codes-and-standards/sae-j2601-fueling-protocols-light-duty-gaseous-hydrogen-surface)
- [SAE J2601/5 — High-Flow Prescriptive Fueling Protocols for Medium and Heavy-Duty Vehicles (H2tools)](https://h2tools.org/fuel-cell-codes-and-standards/sae-j26015-high-flow-prescriptive-fueling-protocols-gaseous-hydrogen)
- [J2601_201612 standard record (SAE Mobilus)](https://saemobilus.sae.org/standards/j2601_201612-fueling-protocols-light-duty-gaseous-hydrogen-surface-vehicles)
- [Impact of hydrogen SAE J2601 fueling methods on fueling time (Argonne / OSTI)](https://www.osti.gov/servlets/purl/1389635)
- [Overview of the SAE J2601 MC Formula H2 Fueling Protocol (CEP)](https://cep.expert/wp-content/uploads/2025/09/CEP_REC_Overview-of-the-SAE-J2601-MC-Formula-H2-Fueling-Protocol_EN.pdf)
- [Understanding the SAE J2601 standard for hydrogen refuelling (Atawey)](https://atawey.com/en/sae-j2601-understanding-the-international-standard-for-gaseous-hydrogen-refueling/)
- [US12313224 — Hydrogen filling method, hydrogen filling apparatus, program, and record medium](https://patents.justia.com/patent/12313224)
- [US12429852 — Hydrogen filling apparatus and hydrogen filling method](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/12429852)
- [US11920736 — Method and system for filling tanks of hydrogen-fueled vehicles](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/11920736)
- [US10451219 (Air Liquide) — Method and device for filling a hydrogen tank](https://uspto.report/patent/grant/10,451,219)
- [US10724767 — High-pressure hydrogen filling system with expansion turbine](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/10724767)
- [US10960783 (Honda) — Communication systems and methods for hydrogen fueling and electric charging](https://patents.justia.com/patent/10960783)
- [Hydrogen station technology development review through patent analysis (Clean Energy, OUP)](https://academic.oup.com/ce/article/2/1/29/4994821)
- [Optimal estimation of MC parameter in SAE J2601 based on modified formula and ANNs (Fuel)](https://www.sciencedirect.com/science/article/abs/pii/S0016236124004629)
- [Liquid pump-enabled hydrogen refueling for heavy-duty FCEVs: J2601-compliant fills with precooling (IJHE)](https://www.sciencedirect.com/science/article/abs/pii/S0360319921013483)
- [Linde — High-Performance Hydrogen Refueling Technologies](https://www.linde-engineering.com/products-and-services/plant-components/powering-sustainable-mobility-for-generations-to-come)
- [Air Liquide Advanced Technologies — Hydrogen stations](https://advancedtech.airliquide.com/hydrogen-stations)
