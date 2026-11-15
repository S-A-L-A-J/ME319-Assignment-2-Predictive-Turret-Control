# Assumptions, design choices, and limits

## Prescribed by the assignment

- Two-dimensional elevation plane; co-located radar and turret.
- Range/elevation at 20 Hz; range noise standard deviation 1 m; angle-noise
  standard deviation 0.02 deg; angles at the interface are radians.
- Prediction horizon 0.50 s; missing measurements last at most 0.50 s and begin
  no earlier than 1.00 s.
- Target range 800-2000 m, speed at most 150 m/s, acceleration magnitude at
  most 20 m/s^2, and simulation interval 0-10 s.
- Plant parameters: Jt=1200 kg m^2, Jm=0.20 kg m^2, N=100, b=1500 N m s/rad,
  Kt=10 N m/A, Ke=10 V s/rad, R=1 ohm, L=0.10 H.
- Elevation stops -10 to +85 deg, recharge 1.0 s, engagement radius 2 m, and
  negligible laser propagation time.
- A conventional LTI feedback compensator, designed once and frozen.

## Explicit project assumptions and frozen choices

- Simulated sensor noise is independent, Gaussian, and white. The assignment gives
  only means and standard deviations, so this is not an official distribution.
- Motor voltage is clipped to +/-400 V. The assignment does not prescribe a
  voltage limit; this is a transparent drive-sizing assumption.
- Controller design targets (not assignment thresholds): crossover 5-8 rad/s,
  phase margin >=60 deg, gain margin >=12 dB, and 2% settling time <0.5 s.
- IMM parameters: CV acceleration noise 2 m/s^2; CA jerk-noise parameter
  10 m/s^3/sqrt(Hz) under the discrete white-jerk convention; symmetric model
  switch probability 0.02 per 50 ms; initial model probabilities [0.7,0.3].
- Intercept policy: capture the 0.50 s predicted LOS and hold that elevation
  until its due time. The instantaneous estimated ray gate of 1.25 m provides
  margin inside the 2 m success radius.
- Engagement is armed at t=1.0 s in the simulation to avoid a trivial
  pulse at the initialized state. This is a project choice, not a requirement.
- Hard stops are modeled as inelastic. No current limit, friction nonlinearity,
  gearbox backlash, structural flexibility, or rate limit was specified.

## Known limitations

- Converted-measurement covariance is a first-order local approximation.
- The IMM likelihood uses the locally mapped Cartesian covariance.
- The intercept scheduler and ray gate introduce discrete supervisory actions
  around an otherwise LTI feedback loop.
- Standardized scenarios B and C share a maneuver; the assignment does
  not require that relationship.
