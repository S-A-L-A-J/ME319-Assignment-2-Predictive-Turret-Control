# Predictive Target Tracking and Elevation Control for ME319 Assignment 2

Salaj Sonar (24B2133) and Adarsh Rao (24B2132)  
Department of Mechanical Engineering, Indian Institute of Technology Bombay

Public repository:
https://github.com/S-A-L-A-J/ME319-Assignment-2-Predictive-Turret-Control

## I. Introduction

This work develops a single predictive-feedback design that estimates a target's
two-dimensional motion from noisy 20 Hz polar radar observations, predicts its
position 0.50 s ahead, generates an elevation reference, and controls the
specified electromechanical turret. Temporary measurement loss is handled by
model propagation without a measurement update. The design combines a
two-model interacting multiple model (IMM) estimator with a conventional LTI
gain-plus-low-pass compensator and a model-based rate feedforward term. All
parameters are frozen across scenarios.

## II. Architecture and assumptions

The chain is radar -> polar-to-Cartesian conversion -> CV/CA IMM -> 0.50 s
prediction -> elevation reference -> two-degree-of-freedom controller -> motor
and turret. The controller block diagram explicitly splits the reference into
error feedback and filtered reference-rate feedforward, sums both voltage
contributions, applies the +/-400 V clamp, and then drives the plant. The
engagement supervisor captures a 0.50 s predicted intercept elevation and
holds it until the corresponding intercept time. It then issues a one-sample
pulse only when the instantaneous estimated beam miss is within 1.25 m. The
1.0 s recharge is enforced between accepted pulses.

The prescribed quantities and all additional choices are separated in
`ASSUMPTIONS_AND_LIMITS.md`. In particular, independent Gaussian radar noise and
the +/-400 V drive limit are project modeling assumptions, not assignment facts.

## III. Mathematical modeling

The converted position measurement is

`z_k = [r_k cos(theta_k), r_k sin(theta_k)]^T`,

with covariance `R_k = J_k diag(sigma_r^2,sigma_theta^2) J_k^T`, where `J_k` is
the polar-to-Cartesian Jacobian. Two six-state models use
`[x,vx,ax,y,vy,ay]^T`. The CV branch resets acceleration dynamically and uses
piecewise-constant acceleration noise; the CA branch uses white jerk. With
transition matrix `Pi`, mixed initial conditions are formed from
`mu_(i|j) = Pi_ij mu_i / c_j`. Each branch executes the standard Kalman predict
and measurement update. Branch likelihoods update the model probabilities.
During radar loss, only prediction and probability propagation are performed.
The future position is the model-probability-weighted state propagated by
`T_p=0.50 s`, and `theta_p=atan2(y_p,x_p)`.

Reflecting the turret through the 100:1 gearbox gives
`J_eq=J_m+J_t/N^2=0.32 kg m^2` and `b_eq=b/N^2=0.15 N m s/rad`. With motor speed
`omega_m=N theta_dot`, the electrical and mechanical equations are

`L i_dot = V - R i - K_e omega_m`,

`J_eq omega_dot_m = K_t i - b_eq omega_m`, and

`theta_dot = omega_m/N`.

Therefore

`G_p(s)=0.1/[s(0.032s^2+0.335s+100.15)]`.

The implemented law is

`V_m = G_c(s)[theta_ref-theta] + G_ff(s)theta_ref`,

where `G_c(s)=7000/(0.05s+1)` is the feedback compensator and
`G_ff(s)=(1/K_vp)s/(0.02s+1)=1001.5s/(0.02s+1)` is the filtered reference-rate
feedforward. Here `K_vp=0.000998 rad/(V s)` is the plant's low-frequency
velocity constant. Consequently,
`theta/theta_ref=G_p(G_c+G_ff)/(1+G_pG_c)`: feedforward changes the tracking
numerator but not the feedback characteristic equation.

## IV. Controller design and analysis

Proportional control at 6 rad/s crossover had only 4.9 dB gain margin and 0.83 s
settling. A conventional lead increased gain near the lightly damped 55.9 rad/s
motor mode and failed the margin target. Adding one real pole at 20 rad/s
attenuated that mode while preserving the low-frequency crossover. A notch met
the margin targets but was more complex, noisier, and slightly missed the 0.5 s
settling target. This is a two-degree-of-freedom design. Feedback determines
stability, disturbance rejection, and correction of motor-model mismatch.
Feedforward supplies the nominal voltage for reference angular motion, reducing
moving-LOS lag without altering the feedback poles or margins. The 0.02 s
filter limits high-frequency differentiation, while the shared +/-400 V
saturation bounds the combined command. Feedforward accuracy depends on the
plant model, especially back-emf and damping, so residual error is left to
feedback. The selected feedback loop has the following verified nominal results:

| Crossover | Phase margin | Gain margin | 2% settling | Overshoot |
|---:|---:|---:|---:|---:|
| 6.72 rad/s | 70.1 deg | 15.6 dB | 0.330 s | 1.2% |

Across Jt +/-20%, b +/-10%, and Ke +/-10%, the computed phase margin remained
68.0-71.8 deg and gain margin 13.9-17.1 dB. A nonlinear +/-400 V simulation
respected the -10/+85 deg stops. The worst-case 0.1875 rad/s ramp required about
188 V feedforward; nominal post-transient angular error was 0.0054 deg. Ke error
directly affects feedforward matching and is the principal modeled limitation.

## V. Simulation and results

Estimator parameters were tuned only on the tuning set and then frozen. On the
separate 15-trajectory test set with 100 measurement-noise realizations per
trajectory, the selected IMM reproduced these results:

| Metric | Result |
|---|---:|
| Current-position RMSE, mean | 0.503 m |
| 0.50 s prediction RMSE, mean | 1.506 m |
| 0.50 s prediction RMSE, worst trajectory | 2.961 m |
| Loss/recovery-window prediction RMSE, mean | 2.411 m |
| Loss/recovery-window prediction RMSE, worst | 6.678 m |

The complete MATLAB simulation was then run for 20 independent noise
realizations in each standardized scenario. The same estimator, controller and
firing parameters were used throughout.

| Scenario | Success | Est. RMSE | Pred. RMSE | Median first hit | Worst best-pulse miss |
|---|---:|---:|---:|---:|---:|
| A | 20/20 | 0.365 m | 0.696 m | 1.000 s | 0.158 m |
| B | 20/20 | 0.465 m | 1.513 m | 1.000 s | 0.310 m |
| C | 20/20 | 0.531 m | 1.655 m | 1.000 s | 0.223 m |

Scenario B increases prediction error because the maneuver violates either
single kinematic model locally. Scenario C adds propagation-only updates during
the blackout and therefore produces the largest mean prediction error. The IMM
keeps the estimated position error below 0.6 m on average in these standardized
cases. Holding each predicted intercept reference converts the 0.50 s forecast
into a fixed aim direction that is valid at its scheduled time.

## VI. Generalization and limitations

Generalization is supported by the 15-trajectory, 1500-realization held-out
estimator study and the 60-run integrated engagement evaluation. The principal
limitations are model dependence during measurement loss, first-order
polar-covariance conversion, feedforward sensitivity to motor-model error, and
the assumed voltage limit.

## VII. Conclusion

The finalized design separates statistical motion inference from linear turret
control. A receding-horizon scheduler holds the predicted intercept reference
through its due time, while a ray gate preserves instantaneous pulse geometry.
Independent numerical reproduction supports the selected IMM, and analytical
plus nonlinear checks support the accepted compensator. The implementation,
frozen parameters, figures, metrics, and AI documentation are included for
reproducibility.
