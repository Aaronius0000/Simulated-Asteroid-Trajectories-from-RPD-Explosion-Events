"""
RPD Monte Carlo Simulation - Layer 2: Orbital Position Probability
===================================================================
Rapid Planetary Disassembly Hypothesis - Aaron Riley Hurst

LAYER 2 QUESTION:
  Given that three impact events occur with correct timing (Layer 1),
  what is the probability that each impact arrives at the precise
  orbital position required by RPD mechanics?

THE THREE ORBITAL CONSTRAINTS:

  Event 1 — Super-Earth Disruption at 2.19 AU:
    The impactor must intersect the super-Earth's orbital path.
    The super-Earth occupies a specific arc of its orbit at any
    given moment. A relativistic impactor on a hyperbolic/highly
    eccentric trajectory has a narrow intersection window.
    Required: Impact within ±15° of optimal disruption geometry
    (head-on to maximize energy coupling into rocky core)
    Orbital fraction: 30°/360° = 0.0833

  Event 2 — Perihelion Adjustment (must hit at APOGEE):
    To lower Earth Mark Two's perihelion, the retrograde momentum
    impulse must be delivered near apogee of the post-disruption
    orbit (~2.19 AU). Hitting at wrong orbital phase produces
    wrong eccentricity vector — either re-raises apogee or ejects.
    Required: Impact within ±20° of apogee position
    Orbital fraction: 40°/360° = 0.1111

  Event 3 — Apogee Stabilization (double constraint):
    CONSTRAINT 3A: Earth Mark Two must already BE at its new
    perihelion (~1 AU) when Event 3 occurs — it cannot be
    intercepted mid-orbit for this maneuver.
    Required: Earth within ±10° of perihelion at moment of impact
    Orbital fraction: 20°/360° = 0.0556

    CONSTRAINT 3B: The impulse direction must reduce apogee
    specifically away from the asteroid belt (not toward inner
    solar system). This constrains the impact azimuth.
    Required: Impact vector within ±25° of optimal apogee-reducing
    direction
    Azimuth fraction: 50°/360° = 0.1389

RPD ORBITAL PARAMETERS (from paper):
  Super-Earth orbit:   2.19 AU, period ~3.241 years
  Post-E1 orbit:       highly eccentric, a ~2.19 AU, e ~0.1-0.2
  Post-E2 orbit:       e > 0.5, perihelion ~1 AU, apogee ~3+ AU
  Final orbit:         e < 0.1, a ~1 AU (current Earth)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import time

# ─────────────────────────────────────────────────────────────────────────────
# PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────

TOTAL_RUNS   = 1_000_000_000
BATCH_SIZE   =    10_000_000
N_BATCHES    = TOTAL_RUNS // BATCH_SIZE
RANDOM_SEED  = 43

# ── Orbital window constraints (degrees, then converted to fractions) ─────────

# Event 1: optimal disruption geometry (head-on ± 15°)
E1_WINDOW_DEG    = 30.0
E1_ORBITAL_FRAC  = E1_WINDOW_DEG / 360.0

# Event 2: must hit near apogee (± 20°)
E2_WINDOW_DEG    = 40.0
E2_ORBITAL_FRAC  = E2_WINDOW_DEG / 360.0

# Event 3A: Earth must be at perihelion (± 10°)
E3A_WINDOW_DEG   = 20.0
E3A_ORBITAL_FRAC = E3A_WINDOW_DEG / 360.0

# Event 3B: impact azimuth for apogee reduction (± 25°)
E3B_WINDOW_DEG   = 50.0
E3B_AZIMUTH_FRAC = E3B_WINDOW_DEG / 360.0

# Combined Event 3 constraint (both 3A AND 3B must be satisfied)
E3_COMBINED_FRAC = E3A_ORBITAL_FRAC * E3B_AZIMUTH_FRAC

# Layer 1 analytical result (carried forward)
LAYER1_P = 2.100245e-11

print("=" * 65)
print("  RPD Monte Carlo — Layer 2: Orbital Position Probability")
print("=" * 65)
print(f"\n  Event 1 window:  ±{E1_WINDOW_DEG/2:.0f}° of optimal geometry  "
      f"({E1_WINDOW_DEG:.0f}°/360° = {E1_ORBITAL_FRAC:.4f})")
print(f"  Event 2 window:  ±{E2_WINDOW_DEG/2:.0f}° of apogee position  "
      f"({E2_WINDOW_DEG:.0f}°/360° = {E2_ORBITAL_FRAC:.4f})")
print(f"  Event 3A window: ±{E3A_WINDOW_DEG/2:.0f}° of perihelion       "
      f"({E3A_WINDOW_DEG:.0f}°/360° = {E3A_ORBITAL_FRAC:.4f})")
print(f"  Event 3B window: ±{E3B_WINDOW_DEG/2:.0f}° azimuth constraint  "
      f"({E3B_WINDOW_DEG:.0f}°/360° = {E3B_AZIMUTH_FRAC:.4f})")
print(f"\n  Total runs:      {TOTAL_RUNS:,}")
print(f"\n  Running...", end="", flush=True)

# ─────────────────────────────────────────────────────────────────────────────
# EMPIRICAL SIMULATION
# ─────────────────────────────────────────────────────────────────────────────

rng = np.random.default_rng(RANDOM_SEED)
start = time.time()

count_E1        = 0
count_E1_E2     = 0
count_E1_E2_E3A = 0
count_all       = 0

for i in range(N_BATCHES):

    # Each event's orbital phase is uniformly random [0°, 360°)
    # We check if it falls within the required window centered on optimal

    # Event 1: phase must be within E1_WINDOW_DEG of optimal (set at 0°)
    phase1 = rng.uniform(0, 360, BATCH_SIZE)
    hit1   = (phase1 <= E1_WINDOW_DEG / 2) | (phase1 >= 360 - E1_WINDOW_DEG / 2)

    # Event 2: must be within E2_WINDOW_DEG of apogee (180° from perihelion)
    phase2 = rng.uniform(0, 360, BATCH_SIZE)
    hit2   = np.abs(phase2 - 180) <= E2_WINDOW_DEG / 2

    # Event 3A: Earth must be within E3A_WINDOW_DEG of perihelion (0°)
    phase3 = rng.uniform(0, 360, BATCH_SIZE)
    hit3A  = (phase3 <= E3A_WINDOW_DEG / 2) | (phase3 >= 360 - E3A_WINDOW_DEG / 2)

    # Event 3B: impact azimuth within E3B_WINDOW_DEG of apogee-reducing vector
    azimuth3 = rng.uniform(0, 360, BATCH_SIZE)
    hit3B    = (azimuth3 <= E3B_WINDOW_DEG / 2) | (azimuth3 >= 360 - E3B_WINDOW_DEG / 2)

    # Cumulative constraint masks
    mask_E1        = hit1
    mask_E1_E2     = hit1 & hit2
    mask_E1_E2_E3A = hit1 & hit2 & hit3A
    mask_all       = hit1 & hit2 & hit3A & hit3B

    count_E1        += np.sum(mask_E1)
    count_E1_E2     += np.sum(mask_E1_E2)
    count_E1_E2_E3A += np.sum(mask_E1_E2_E3A)
    count_all       += np.sum(mask_all)

    if (i + 1) % 10 == 0:
        print(".", end="", flush=True)

elapsed = time.time() - start

# Empirical probabilities
p_E1_emp        = count_E1        / TOTAL_RUNS
p_E1_E2_emp     = count_E1_E2     / TOTAL_RUNS
p_E1_E2_E3A_emp = count_E1_E2_E3A / TOTAL_RUNS
p_all_emp       = count_all       / TOTAL_RUNS

# ─────────────────────────────────────────────────────────────────────────────
# ANALYTICAL CALCULATION
# ─────────────────────────────────────────────────────────────────────────────

p_E1_analytical   = E1_ORBITAL_FRAC
p_E2_analytical   = E2_ORBITAL_FRAC
p_E3A_analytical  = E3A_ORBITAL_FRAC
p_E3B_analytical  = E3B_AZIMUTH_FRAC
p_layer2_analytical = p_E1_analytical * p_E2_analytical * p_E3A_analytical * p_E3B_analytical

# ── Combined with Layer 1 ─────────────────────────────────────────────────────
p_layers_1_and_2 = LAYER1_P * p_layer2_analytical

# ─────────────────────────────────────────────────────────────────────────────
# PRINT RESULTS
# ─────────────────────────────────────────────────────────────────────────────

print(f"\n\n  Time elapsed: {elapsed:.1f} sec")
print("\n" + "=" * 65)
print("  LAYER 2 RESULTS — Orbital Position Probability")
print("=" * 65)

print("\n  ── EMPIRICAL (1 Billion Runs) ──────────────────────────────")
print(f"  [E1]      Impact at correct geometry:  {count_E1:>12,}  P = {p_E1_emp:.4e}")
print(f"  [E1+E2]   + Apogee hit:                {count_E1_E2:>12,}  P = {p_E1_E2_emp:.4e}")
print(f"  [E1+E2+E3A] + Earth at perihelion:     {count_E1_E2_E3A:>12,}  P = {p_E1_E2_E3A_emp:.4e}")
print(f"  [ALL]     + Correct azimuth:           {count_all:>12,}  P = {p_all_emp:.4e}")

print("\n  ── ANALYTICAL (Exact) ──────────────────────────────────────")
print(f"  P(E1)  — impact geometry:    {p_E1_analytical:.6f}  ({E1_WINDOW_DEG:.0f}°/360°)")
print(f"  P(E2)  — apogee position:    {p_E2_analytical:.6f}  ({E2_WINDOW_DEG:.0f}°/360°)")
print(f"  P(E3A) — perihelion phase:   {p_E3A_analytical:.6f}  ({E3A_WINDOW_DEG:.0f}°/360°)")
print(f"  P(E3B) — azimuth constraint: {p_E3B_analytical:.6f}  ({E3B_WINDOW_DEG:.0f}°/360°)")
print(f"\n  P(Layer 2 combined):         {p_layer2_analytical:.6e}")
print(f"  Odds against (Layer 2):      1 in {1/p_layer2_analytical:,.0f}")

print("\n  ── COMBINED LAYERS 1 + 2 ───────────────────────────────────")
print(f"  Layer 1 (timing):            {LAYER1_P:.6e}")
print(f"  Layer 2 (orbital position):  {p_layer2_analytical:.6e}")
print(f"\n  P(Layers 1+2 combined):      {p_layers_1_and_2:.6e}")
print(f"  Odds against (L1+L2):        1 in {1/p_layers_1_and_2:,.4e}")
print(f"\n  NOTE: Layer 3 (energy delivery) will reduce this further.")

# ─────────────────────────────────────────────────────────────────────────────
# PLOTS
# ─────────────────────────────────────────────────────────────────────────────

fig = plt.figure(figsize=(18, 11))
fig.suptitle(
    "RPD Monte Carlo — Layer 2: Orbital Position Probability\n"
    f"N = {TOTAL_RUNS:,} runs  |  Layer 2 P = {p_layer2_analytical:.3e}  |  "
    f"Combined L1+L2 P = {p_layers_1_and_2:.3e}",
    fontsize=13, fontweight='bold'
)

# ── Plot 1: Orbital diagram — Event 1 ─────────────────────────────────────────
ax1 = fig.add_subplot(2, 3, 1, projection='polar')
theta = np.linspace(0, 2*np.pi, 500)
ax1.plot(theta, np.ones_like(theta) * 2.19, 'b-', lw=1.5, alpha=0.5, label='Super-Earth orbit')
# Highlight valid impact window
window_rad = np.radians(E1_WINDOW_DEG / 2)
theta_win = np.linspace(-window_rad, window_rad, 50)
ax1.fill_between(theta_win, 0, 2.5, alpha=0.3, color='red', label=f'Valid window (±{E1_WINDOW_DEG/2:.0f}°)')
ax1.set_rticks([1, 2, 3])
ax1.set_title(f'Event 1: Impact Geometry\n{E1_WINDOW_DEG:.0f}°/360° = {E1_ORBITAL_FRAC:.3f}',
              fontsize=10, pad=15)
ax1.legend(fontsize=7, loc='lower right')

# ── Plot 2: Orbital diagram — Event 2 ─────────────────────────────────────────
ax2 = fig.add_subplot(2, 3, 2, projection='polar')
# Eccentric post-E1 orbit
ecc = 0.5
a = 2.19
r_ellipse = a * (1 - ecc**2) / (1 + ecc * np.cos(theta))
r_ellipse = np.clip(r_ellipse, 0, 6)
ax2.plot(theta, r_ellipse, 'g-', lw=1.5, alpha=0.6, label='Post-E1 orbit (e=0.5)')
# Apogee window at 180°
window_rad2 = np.radians(E2_WINDOW_DEG / 2)
theta_win2  = np.linspace(np.pi - window_rad2, np.pi + window_rad2, 50)
ax2.fill_between(theta_win2, 0, 6, alpha=0.3, color='orange', label=f'Apogee window (±{E2_WINDOW_DEG/2:.0f}°)')
ax2.set_rticks([1, 2, 3, 4])
ax2.set_title(f'Event 2: Must Hit at Apogee\n{E2_WINDOW_DEG:.0f}°/360° = {E2_ORBITAL_FRAC:.3f}',
              fontsize=10, pad=15)
ax2.legend(fontsize=7, loc='lower right')

# ── Plot 3: Orbital diagram — Event 3 ─────────────────────────────────────────
ax3 = fig.add_subplot(2, 3, 3, projection='polar')
ecc3 = 0.5
a3   = 2.0
r3   = a3 * (1 - ecc3**2) / (1 + ecc3 * np.cos(theta))
r3   = np.clip(r3, 0, 5)
ax3.plot(theta, r3, 'm-', lw=1.5, alpha=0.6, label='Post-E2 orbit')
# Perihelion window at 0°
window_rad3 = np.radians(E3A_WINDOW_DEG / 2)
theta_win3  = np.linspace(-window_rad3, window_rad3, 50)
ax3.fill_between(theta_win3, 0, 1.5, alpha=0.35, color='green',
                 label=f'Perihelion window (±{E3A_WINDOW_DEG/2:.0f}°)')
# Earth at 1 AU reference
ax3.plot(theta, np.ones_like(theta), 'b--', lw=1, alpha=0.4, label='1 AU (Earth target)')
ax3.set_rticks([1, 2, 3])
ax3.set_title(f'Event 3: Earth at Perihelion + Azimuth\n'
              f'{E3A_WINDOW_DEG:.0f}°×{E3B_WINDOW_DEG:.0f}°/360°² = {E3_COMBINED_FRAC:.4f}',
              fontsize=10, pad=15)
ax3.legend(fontsize=7, loc='lower right')

# ── Plot 4: Layer 2 constraint cascade ────────────────────────────────────────
ax4 = fig.add_subplot(2, 3, 4)
labels4  = ['E1\nGeometry', 'E1+E2\n+Apogee', 'E1+E2+E3A\n+Perihelion', 'ALL\n+Azimuth']
emp_vals = [p_E1_emp, p_E1_E2_emp, p_E1_E2_E3A_emp, p_all_emp]
ana_vals = [p_E1_analytical,
            p_E1_analytical * p_E2_analytical,
            p_E1_analytical * p_E2_analytical * p_E3A_analytical,
            p_layer2_analytical]
x4    = np.arange(len(labels4))
width = 0.35
ax4.bar(x4 - width/2, emp_vals, width, label='Empirical', color='steelblue',  alpha=0.8)
ax4.bar(x4 + width/2, ana_vals, width, label='Analytical', color='darkorange', alpha=0.8)
ax4.set_yscale('log')
ax4.set_xticks(x4)
ax4.set_xticklabels(labels4, fontsize=9)
ax4.set_ylabel('Probability (log scale)')
ax4.set_title('Layer 2 Constraint Cascade')
ax4.legend(fontsize=9)

# ── Plot 5: Cumulative probability L1 × L2 ────────────────────────────────────
ax5 = fig.add_subplot(2, 3, 5)
layer_labels = ['Layer 1\nTiming', 'Layer 2\nOrbital', 'L1 × L2\nCombined']
layer_probs  = [LAYER1_P, p_layer2_analytical, p_layers_1_and_2]
colors5      = ['#1565C0', '#2E7D32', '#B71C1C']
bars5 = ax5.bar(layer_labels, layer_probs, color=colors5, alpha=0.85, edgecolor='white', width=0.5)
for bar, p in zip(bars5, layer_probs):
    ax5.text(bar.get_x() + bar.get_width()/2, p * 2,
             f'{p:.2e}', ha='center', va='bottom', fontsize=10, fontweight='bold')
ax5.set_yscale('log')
ax5.set_ylabel('Probability (log scale)')
ax5.set_title('Cumulative Probability\nLayers 1 and 2')

# ── Plot 6: Summary text panel ────────────────────────────────────────────────
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')
summary = (
    f"LAYERS 1 + 2 COMBINED\n"
    f"{'─'*36}\n\n"
    f"Layer 1 (Timing):\n"
    f"  P = {LAYER1_P:.4e}\n"
    f"  1 in 47,613,490,767\n\n"
    f"Layer 2 (Orbital Position):\n"
    f"  E1 geometry:   {p_E1_analytical:.4f}  (30°/360°)\n"
    f"  E2 apogee:     {p_E2_analytical:.4f}  (40°/360°)\n"
    f"  E3A perihelion:{p_E3A_analytical:.4f}  (20°/360°)\n"
    f"  E3B azimuth:   {p_E3B_analytical:.4f}  (50°/360°)\n"
    f"  P = {p_layer2_analytical:.4e}\n"
    f"  1 in {1/p_layer2_analytical:,.0f}\n\n"
    f"COMBINED L1 × L2:\n"
    f"  P = {p_layers_1_and_2:.4e}\n"
    f"  1 in {1/p_layers_1_and_2:.4e}\n\n"
    f"Layer 3 (Energy) pending...\n"
    f"Final P will be smaller still."
)
ax6.text(0.05, 0.97, summary, transform=ax6.transAxes,
         fontsize=9, verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.tight_layout()
plt.savefig('rpd_layer2_results.png', dpi=150, bbox_inches='tight')
plt.show()

print(f"\n  Plot saved: rpd_layer2_results.png")
print("=" * 65)
print("  Layer 2 complete. Ready for Layer 3: Energy Delivery.")
print("=" * 65)