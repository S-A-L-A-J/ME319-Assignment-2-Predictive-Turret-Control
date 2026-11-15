from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


OUT = Path(__file__).resolve().parents[1] / "figures"


def box(ax, x, y, w, h, text, fc="#edf4fb"):
    patch = FancyBboxPatch(
        (x, y), w, h, boxstyle="round,pad=0.02", fc=fc, ec="#1d3557", lw=1.2
    )
    ax.add_patch(patch)
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=8)


def arrow(ax, start, end, style="-"):
    ax.add_patch(
        FancyArrowPatch(
            start,
            end,
            arrowstyle="-|>",
            mutation_scale=10,
            lw=1.0,
            color="#333",
            linestyle=style,
        )
    )


fig, ax = plt.subplots(figsize=(13, 3.25))
ax.set_xlim(0, 13)
ax.set_ylim(0, 3.25)
ax.axis("off")
top = [
    ("Radar\n$r,\\theta$", 0.20, 1.25),
    ("Polar-to-Cartesian\nmeasurement", 1.75, 1.65),
    ("CV/CA IMM\nstate estimate", 3.70, 1.45),
    ("0.50 s state\nprediction", 5.45, 1.45),
    ("Intercept scheduler\nand reference hold", 7.20, 1.70),
]
for text, x, width in top:
    box(ax, x, 2.35, width, 0.55, text)
for start, end in [
    ((1.45, 2.625), (1.75, 2.625)),
    ((3.40, 2.625), (3.70, 2.625)),
    ((5.15, 2.625), (5.45, 2.625)),
    ((6.90, 2.625), (7.20, 2.625)),
]:
    arrow(ax, start, end)
ax.text(0.20, 3.06, "(a) Predictive intercept generation", fontsize=9, fontweight="bold")
ax.text(9.02, 2.625, r"$t_{fire}$", fontsize=10, va="center")
box(ax, 9.40, 2.35, 1.35, 0.55, "Ray gate at\nintercept time", fc="#fef2df")
box(ax, 11.10, 2.35, 1.45, 0.55, "Rising-edge\n" r"$fire\_cmd$ pulse", fc="#e8f5e9")
arrow(ax, (8.90, 2.625), (9.40, 2.625))
arrow(ax, (10.75, 2.625), (11.10, 2.625))

ax.text(0.20, 2.10, "(b) Two-degree-of-freedom turret circuit", fontsize=9, fontweight="bold")
ax.text(0.18, 1.08, r"$\theta_r$", fontsize=10, va="center")
ax.plot([0.50, 0.78], [1.08, 1.08], color="#333", lw=1)
ax.plot([0.64, 0.64, 1.28], [1.08, 1.72, 1.72], color="#333", lw=1)
ax.add_patch(plt.Circle((1.00, 1.08), 0.12, fill=False, ec="#1d3557", lw=1.2))
ax.text(1.00, 1.08, "+", ha="center", va="center", fontsize=8)
ax.text(0.90, 0.86, "-", ha="center", va="center", fontsize=9)
arrow(ax, (0.78, 1.08), (0.88, 1.08))
box(ax, 1.28, 0.78, 1.75, 0.60, "Feedback $G_c(s)$\n$7000/(0.05s+1)$")
arrow(ax, (1.12, 1.08), (1.28, 1.08))
box(
    ax,
    1.28,
    1.45,
    2.35,
    0.55,
    "Reference feedforward $G_{ff}(s)$\n$1001.5s/(0.02s+1)$",
    fc="#e8f5e9",
)
arrow(ax, (0.64, 1.72), (1.28, 1.72))
ax.add_patch(plt.Circle((4.05, 1.08), 0.12, fill=False, ec="#1d3557", lw=1.2))
ax.text(4.05, 1.08, "+", ha="center", va="center", fontsize=8)
arrow(ax, (3.03, 1.08), (3.93, 1.08))
ax.plot([3.63, 4.05], [1.72, 1.72], color="#333", lw=1)
arrow(ax, (4.05, 1.72), (4.05, 1.20))
box(ax, 4.42, 0.78, 1.22, 0.60, "Voltage limit\n+/-400 V", fc="#fef2df")
arrow(ax, (4.17, 1.08), (4.42, 1.08))
box(ax, 6.00, 0.78, 1.42, 0.60, "Turret plant\n$G_p(s)$", fc="#fef2df")
arrow(ax, (5.64, 1.08), (6.00, 1.08))
arrow(ax, (7.42, 1.08), (8.00, 1.08))
ax.text(8.10, 1.08, r"$\theta$", fontsize=10, va="center")
ax.plot([7.72, 7.72, 1.00, 1.00], [1.08, 0.30, 0.30, 0.96], color="#333", lw=1)
arrow(ax, (1.00, 0.30), (1.00, 0.96))
ax.text(10.55, 1.42, "Feedback: stability and\ndisturbance rejection", ha="center", va="center", fontsize=8)
ax.text(10.55, 0.68, "Feedforward: reduced\nmoving-reference lag", ha="center", va="center", fontsize=8)
fig.tight_layout()
fig.savefig(OUT / "architecture.png", dpi=220, bbox_inches="tight")
plt.close(fig)

fig, ax = plt.subplots(figsize=(13, 2.3))
ax.set_xlim(0, 13)
ax.set_ylim(0, 2.3)
ax.axis("off")
box(ax, 0.3, 0.82, 1.65, 0.68, "Voltage $V_m$\n$R=1$ ohm, $L=0.10$ H")
box(ax, 2.45, 0.82, 1.45, 0.68, "Motor\n$K_t=K_e=10$")
box(ax, 4.45, 0.82, 1.35, 0.68, "Gear\n$N=100:1$", fc="#fef2df")
box(ax, 6.35, 0.82, 1.85, 0.68, "Equivalent inertia\n$J_{eq}=0.32$ kg m$^2$")
box(ax, 8.75, 0.82, 1.25, 0.68, "Turret\n$\\theta$", fc="#e8f5e9")
for start, end in [
    ((1.95, 1.16), (2.45, 1.16)),
    ((3.90, 1.16), (4.45, 1.16)),
    ((5.80, 1.16), (6.35, 1.16)),
    ((8.20, 1.16), (8.75, 1.16)),
]:
    arrow(ax, start, end)
ax.plot([3.90, 4.15, 4.15, 3.16], [1.16, 1.16, 2.00, 2.00], color="#333", lw=1)
arrow(ax, (3.16, 2.00), (3.16, 1.50))
ax.text(3.62, 2.12, r"back-emf $K_e\omega_m$", ha="center", fontsize=8)
ax.text(11.25, 1.48, r"$J_{eq}\dot{\omega}_m+b_{eq}\omega_m=K_t i$", ha="center", fontsize=9)
ax.text(11.25, 0.86, r"$\dot{\theta}=\omega_m/N$", ha="center", fontsize=9)
ax.text(11.25, 0.36, r"$b_{eq}=0.15$ N m s/rad", ha="center", fontsize=9)
fig.tight_layout()
fig.savefig(OUT / "plant_schematic.png", dpi=220, bbox_inches="tight")
plt.close(fig)
