# Mandatory AI and computational-tools appendix

OpenAI Codex was used to inspect the supplied archives and assignment brief,
reconstruct the intended project structure, audit the estimator and controller
implementations, integrate the estimator, predictor, plant, controller and
firing logic, and prepare final reproducibility/report material. MATLAB R2025b was used
for the authoritative standardized simulations, controller analysis, result
tables, and figures. Python 3.13 with NumPy, SciPy and Matplotlib was used only
as an independent intermediate cross-check.

Significant accepted outputs were: (i) selection of the CV/CA IMM after a
frozen-parameter comparison, (ii) reproduction of its archived test metrics by
an independent Python implementation, (iii) first-principles derivation of the
motor-shaft-equivalent plant, and (iv) the gain-plus-low-pass compensator after
frequency-domain, time-domain, robustness and nonlinear saturation checks.

Significant modifications were: the four supplied archives were reorganized
into one reproducible project; the older CV-only demonstration was not used as
the final estimator; and the firing supervisor was added to enforce
rising-edge pulses, recharge time, and the engagement-radius test. The accepted
supervisor captures a 0.50 s predicted intercept elevation, holds it until that
intercept time, and then applies the instantaneous estimated ray gate.

Validation used dimensional checks, physical-envelope assertions, an
independent reproduction of the 15-trajectory/100-noise-run estimator metrics,
closed-loop pole/margin calculations, nonlinear integration with voltage
clipping and hard stops, and 20 Monte Carlo measurement realizations for each
standardized scenario. Assumptions not prescribed by the assignment (Gaussian
independence, +/-400 V, design-margin targets, and firing gate) are labeled as
choices. The final equations, code, results, figures, and interpretation were
reviewed against the assignment requirements before inclusion.
