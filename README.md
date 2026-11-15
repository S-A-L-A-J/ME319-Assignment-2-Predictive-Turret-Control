# ME319 Assignment 2 - complete predictive elevation-control project

Public repository: https://github.com/S-A-L-A-J/ME319-Assignment-2-Predictive-Turret-Control

This repository integrates the supplied target datasets, selected
estimator/predictor, turret plant, LTI controller, reference manager, and
rising-edge laser command logic. MATLAB R2025b is the authoritative execution
path, as required by the assignment.

## Finalized architecture

1. Convert noisy range/elevation measurements to Cartesian position and mapped
   covariance.
2. Estimate position, velocity, acceleration, and model probability with a
   two-model CV/CA interacting multiple model (IMM) filter.
3. Predict Cartesian position 0.50 s ahead and convert it to a future elevation
   reference.
4. Track that future reference with a two-degree-of-freedom controller:
   `V_m = Gc(s)(theta_ref-theta) + Gff(s)theta_ref`, where
   `Gc(s) = 7000/(0.05s+1)` is the feedback compensator and
   `Gff(s) = 1001.5s/(0.02s+1)` is filtered reference-rate feedforward.
   Feedback sets stability and rejects model error; feedforward reduces
   moving-reference lag without changing the feedback characteristic equation.
5. Schedule an intercept from the 0.50 s predicted LOS and hold that elevation
   until the corresponding intercept time. Fire a one-step rising-edge pulse
   only when the instantaneous estimated ray miss is at most 1.25 m; enforce
   the 1.0 s recharge time. Success is checked against the 2 m radius.

The same parameters are used for all scenarios and all frozen test cases.

## Reproduction in MATLAB

Set this directory as the MATLAB current folder, then run:

```matlab
addpath('matlab','matlab/strategies');
verify_assignment_contract;
controller_analysis;
summary = run_all(20);
```

The evaluator-facing definitions match the assignment exactly:

```matlab
[r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)
[V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
[theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)
```

`fire_cmd` is binary and is asserted for one controller call only. Therefore each
accepted event is a 0-to-1 transition, followed by a return to zero; the frozen
controller also prevents a new pulse until the 1.0 s recharge interval has elapsed.

The verified MATLAB run reports 20/20 successful engagements in each of A, B,
and C. It writes all numerical results to `results/` and report figures to
`figures/`.

Generated outputs are in `results/` and `figures/`.

## Repository map

- `run_all.m` - authoritative complete MATLAB simulation and result generator.
- `data/` - frozen standardized scenarios, tuning set, and held-out test set.
- `matlab/target_motion_generator.m` - required scenario interface.
- `matlab/turret_controller.m` - required evaluator-facing controller function.
- `matlab/turret_plant.m` - required evaluator-facing physical plant function.
- `matlab/controller_analysis.m` - plant derivation and LTI-loop evidence.
- `matlab/strategies/` - all compared estimator implementations.
- `verify_assignment_contract.m` - automated check of the three mandatory
  evaluator signatures, output counts, finite values, and binary commands.
- `report/ME319_Assignment2_IEEE_Final.pdf` - six-page IEEE-style main paper
  plus the one-page mandatory AI appendix.
- `report/TECHNICAL_REPORT.md` - technical derivation and verified tables.
- `report/AI_APPENDIX.md` - mandatory AI-use appendix.
- `report/ASSUMPTIONS_AND_LIMITS.md` - prescribed facts versus design choices.
- `report/make_report_assets.py` - regenerates the architecture and plant block diagrams.

## Verification record

The frozen estimator was reproduced over 15 held-out trajectories with 100
noise realizations per trajectory. The integrated plant/controller evaluation
uses 20 noise realizations for each standardized scenario, including the
maximum-duration measurement outage in scenario C. The exact function
signatures, binary pulse semantics, dataset envelopes, prediction pipeline,
and report figures are covered by the supplied verification scripts.
