"""
RPD Monte Carlo Simulation - Layer 1B: 1 Billion Runs + Analytical Proof
=========================================================================
Rapid Planetary Disassembly Hypothesis - Aaron Riley Hurst

This script does TWO things simultaneously:

  1. EMPIRICAL: Runs 1,000,000,000 Monte Carlo iterations to establish
     a hard empirical upper bound on timing probability. Runs in batches
     to avoid memory overflow.

  2. ANALYTICAL: Calculates the exact theoretical probability using
     conditional probability mathematics — giving a precise p-value
     that doesn't depend on run count.

Both methods should converge on the same answer, providing
independent validation of each other for peer review.

RPD CONSTRAINTS BEING TESTED (Layer 1 only — timing):
  [A] All 3 events within 3.5 Myr window
  [B] Event 1 occurs in first 10 Myr of 10,000 Myr stellar life
  [C] E1→E2 gap = 1.0 Myr ± 0.5 Myr
  [D] E1→E3 gap = 3.5 Myr ± 0.5 Myr
"""

import numpy as np
import matplotlib.pyplot as plt
import time
from datetime import datetime

# ─────────────────────────────────────────────────────────────────────────────
# PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────

TOTAL_RUNS          = 1_000_000_000   # 1 billion
BATCH_SIZE          =    10_000_000   # 10 million per batch (memory safe)
N_BATCHES           = TOTAL_RUNS // BATCH_SIZE
RANDOM_SEED         = 42

# Stellar parameters (Myr)
STAR_LIFESPAN       = 10_000.0
CLUSTER_PHASE       =    10.0

# RPD timing constraints (Myr)
RPD_WINDOW          =  3.5
E1_E2_GAP           =  1.0
E1_E3_GAP           =  3.5
GAP_TOLERANCE       =  0.5

print("=" * 65)
print("  RPD Monte Carlo — Layer 1B")
print("  1,000,000,000 Runs + Analytical Probability")
print("=" * 65)
print(f"\n  Total runs:        {TOTAL_RUNS:>15,}")
print(f"  Batch size:        {BATCH_SIZE:>15,}")
print(f"  Number of batches: {N_BATCHES:>15,}")
print(f"\n  Started: {datetime.now().strftime('%H:%M:%S')}")
print("\n  Running batches", end="", flush=True)

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 1: EMPIRICAL — 1 BILLION RUN MONTE CARLO (batched)
# ─────────────────────────────────────────────────────────────────────────────

rng = np.random.default_rng(RANDOM_SEED)

# Counters for each constraint level
count_A = 0   # Window only
count_B = 0   # Window + early life
count_C = 0   # Window + early life + E1→E2 gap
count_D = 0   # Window + early life + E1→E2 gap + E1→E3 gap (all constraints)

start_time = time.time()

for batch_num in range(N_BATCHES):

    # Sample Event 1 uniformly across stellar lifetime
    e1 = rng.uniform(0, STAR_LIFESPAN, BATCH_SIZE)

    # Sample Event 2 gap (0 to remaining life)
    e2_gap = rng.uniform(0, STAR_LIFESPAN - e1, BATCH_SIZE)
    e2 = e1 + e2_gap

    # Sample Event 3 gap (0 to remaining life after e2)
    e3_gap = rng.uniform(0, STAR_LIFESPAN - e2, BATCH_SIZE)
    e3 = e2 + e3_gap

    # Derived values
    total_span  = e3 - e1
    gap_12      = e2 - e1
    gap_13      = e3 - e1

    # Apply constraints cumulatively
    mask_A = total_span <= RPD_WINDOW
    mask_B = mask_A & (e1 <= CLUSTER_PHASE)
    mask_C = mask_B & (np.abs(gap_12 - E1_E2_GAP) <= GAP_TOLERANCE)
    mask_D = mask_C & (np.abs(gap_13 - E1_E3_GAP) <= GAP_TOLERANCE)

    count_A += np.sum(mask_A)
    count_B += np.sum(mask_B)
    count_C += np.sum(mask_C)
    count_D += np.sum(mask_D)

    # Progress indicator every 10 batches
    if (batch_num + 1) % 10 == 0:
        print(".", end="", flush=True)

elapsed = time.time() - start_time
print(f"\n\n  Finished: {datetime.now().strftime('%H:%M:%S')}")
print(f"  Time elapsed: {elapsed:.1f} sec")

# Empirical probabilities
p_A = count_A / TOTAL_RUNS
p_B = count_B / TOTAL_RUNS
p_C = count_C / TOTAL_RUNS
p_D = count_D / TOTAL_RUNS

# ─────────────────────────────────────────────────────────────────────────────
# SECTION 2: ANALYTICAL — Exact Theoretical Probability
# ─────────────────────────────────────────────────────────────────────────────

# The analytical solution breaks down each constraint mathematically
# using conditional probability:  P(A∩B∩C∩D) = P(A) × P(B|A) × P(C|A,B) × P(D|A,B,C)

# ── P(A): 3 random events fall within a 3.5 Myr window ───────────────────────
# For 3 uniform random order statistics on [0, L], the probability that
# the range (max - min) ≤ w is:
# P = (w/L)^2 × (3 - 2w/L)  for w << L   [exact formula for n=3 uniform]
w = RPD_WINDOW
L = STAR_LIFESPAN
p_A_analytical = (w/L)**2 * (3 - 2*(w/L))

# ── P(B|A): Given events are clustered, Event 1 is in first 10 Myr ───────────
# If 3 events are constrained to a 3.5 Myr window anywhere in 10,000 Myr,
# the probability the window starts in the first 10 Myr is:
# The window start can be anywhere from 0 to (L - w), so:
p_B_given_A_analytical = CLUSTER_PHASE / (STAR_LIFESPAN - RPD_WINDOW)

# ── P(C|A,B): E1→E2 gap is 1.0 ± 0.5 Myr given window constraint ─────────────
# Within the 3.5 Myr window, the gap is uniform on [0, 3.5]
# Probability of landing in [0.5, 1.5] Myr band:
p_C_given_AB_analytical = (2 * GAP_TOLERANCE) / RPD_WINDOW

# ── P(D|A,B,C): E1→E3 gap is 3.5 ± 0.5 Myr given all above ──────────────────
# Given E1→E2 is fixed at ~1.0 Myr and total span ≤ 3.5 Myr,
# E1→E3 must land in [3.0, 3.5] Myr (upper end of window)
# Remaining range for E3 is [E2_time, 3.5 Myr], which is ~2.5 Myr wide
# Probability of landing in the 0.5 Myr RPD band:
remaining_range = RPD_WINDOW - E1_E2_GAP   # ~2.5 Myr
p_D_given_ABC_analytical = GAP_TOLERANCE / remaining_range

# ── Combined analytical probability ──────────────────────────────────────────
p_combined_analytical = (p_A_analytical *
                         p_B_given_A_analytical *
                         p_C_given_AB_analytical *
                         p_D_given_ABC_analytical)

# ─────────────────────────────────────────────────────────────────────────────
# PRINT RESULTS
# ─────────────────────────────────────────────────────────────────────────────

print("\n" + "=" * 65)
print("  LAYER 1B RESULTS")
print("=" * 65)

print("\n  ── EMPIRICAL (Monte Carlo) ─────────────────────────────────")
print(f"\n  [A] 3 events in {RPD_WINDOW} Myr window:")
print(f"      Passes: {count_A:>12,}  P = {p_A:.6e}")
print(f"\n  [B] + Event 1 in first {CLUSTER_PHASE:.0f} Myr:")
print(f"      Passes: {count_B:>12,}  P = {p_B:.6e}")
print(f"\n  [C] + E1→E2 gap = {E1_E2_GAP}±{GAP_TOLERANCE} Myr:")
print(f"      Passes: {count_C:>12,}  P = {p_C:.6e}")
print(f"\n  [D] + E1→E3 gap = {E1_E3_GAP}±{GAP_TOLERANCE} Myr:")
print(f"      Passes: {count_D:>12,}  P = {p_D:.6e}")

if count_D == 0:
    print(f"\n  *** EMPIRICAL RESULT: ZERO PASSES in {TOTAL_RUNS:,} runs ***")
    print(f"  *** Upper bound: P < 1 / {TOTAL_RUNS:,}  =  {1/TOTAL_RUNS:.3e} ***")

print("\n  ── ANALYTICAL (Exact Mathematical) ─────────────────────────")
print(f"\n  P(A) — window constraint:        {p_A_analytical:.6e}")
print(f"  P(B|A) — early stellar life:     {p_B_given_A_analytical:.6e}")
print(f"  P(C|A,B) — E1→E2 gap:           {p_C_given_AB_analytical:.6e}")
print(f"  P(D|A,B,C) — E1→E3 gap:         {p_D_given_ABC_analytical:.6e}")
print(f"\n  P(COMBINED) — Layer 1 total:     {p_combined_analytical:.6e}")

if p_combined_analytical > 0:
    print(f"\n  Odds against (Layer 1 only):     1 in {1/p_combined_analytical:,.0f}")

print(f"\n  ── Interpretation ──────────────────────────────────────────")
print(f"\n  The Layer 1 timing constraints alone produce a probability")
print(f"  of approximately {p_combined_analytical:.2e}.")
print(f"  This means roughly 1 in {1/p_combined_analytical:,.0f} randomly")
print(f"  initialized star systems would produce the RPD timing sequence")
print(f"  by chance — BEFORE applying orbital position or energy")
print(f"  constraints (Layers 2 and 3).")
print(f"\n  NOTE: This is Layer 1 ONLY. The final combined probability")
print(f"  across all layers will be orders of magnitude smaller.")

# ─────────────────────────────────────────────────────────────────────────────
# PLOTS
# ─────────────────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(1, 3, figsize=(16, 6))
fig.suptitle(
    "RPD Monte Carlo Layer 1B — Timing Probability\n"
    f"N = {TOTAL_RUNS:,} runs  |  Analytical P = {p_combined_analytical:.3e}",
    fontsize=13, fontweight='bold'
)

# Plot 1: Constraint cascade — empirical vs analytical
ax1 = axes[0]
constraints   = ['[A]\nWindow\nonly', '[B]\n+Early\nlife', '[C]\n+E1→E2\ngap', '[D]\n+E1→E3\ngap']
empirical_p   = [p_A, p_B, p_C, p_D]
analytical_p  = [p_A_analytical,
                 p_A_analytical * p_B_given_A_analytical,
                 p_A_analytical * p_B_given_A_analytical * p_C_given_AB_analytical,
                 p_combined_analytical]

x = np.arange(len(constraints))
width = 0.35
bars1 = ax1.bar(x - width/2, empirical_p,   width, label='Empirical',   color='steelblue',  alpha=0.8)
bars2 = ax1.bar(x + width/2, analytical_p,  width, label='Analytical',  color='darkorange', alpha=0.8)

# Add value labels
for bar, p in zip(bars1, empirical_p):
    if p > 0:
        ax1.text(bar.get_x() + bar.get_width()/2, p * 2,
                 f'{p:.1e}', ha='center', va='bottom', fontsize=7, rotation=45)
for bar, p in zip(bars2, analytical_p):
    if p > 0:
        ax1.text(bar.get_x() + bar.get_width()/2, p * 2,
                 f'{p:.1e}', ha='center', va='bottom', fontsize=7, rotation=45)

ax1.set_yscale('log')
ax1.set_xticks(x)
ax1.set_xticklabels(constraints, fontsize=9)
ax1.set_ylabel('Probability (log scale)')
ax1.set_title('Constraint Cascade\nEmpirical vs Analytical')
ax1.legend(fontsize=9)

# Plot 2: Probability multiplication waterfall
ax2 = axes[1]
steps = ['P(A)\nWindow', '×P(B|A)\nEarly life', '×P(C|AB)\nE1→E2 gap', '×P(D|ABC)\nE1→E3 gap']
step_probs = [p_A_analytical,
              p_B_given_A_analytical,
              p_C_given_AB_analytical,
              p_D_given_ABC_analytical]
colors = ['#2196F3', '#FF9800', '#4CAF50', '#F44336']
bars = ax2.bar(steps, step_probs, color=colors, alpha=0.85, edgecolor='white')
for bar, p in zip(bars, step_probs):
    ax2.text(bar.get_x() + bar.get_width()/2, p * 1.3,
             f'{p:.2e}', ha='center', va='bottom', fontsize=9, fontweight='bold')
ax2.set_yscale('log')
ax2.set_ylabel('Individual Constraint Probability')
ax2.set_title('Each Constraint Factor\n(multiply together for combined P)')

# Plot 3: Final result visualization
ax3 = axes[2]
ax3.axis('off')
result_text = (
    f"LAYER 1 TIMING PROBABILITY\n"
    f"{'─'*38}\n\n"
    f"Analytical Result:\n"
    f"  P = {p_combined_analytical:.4e}\n\n"
    f"  1 in {1/p_combined_analytical:,.0f}\n"
    f"  solar systems\n\n"
    f"Empirical Result (N=1B):\n"
)
if count_D == 0:
    result_text += f"  ZERO passes\n  P < {1/TOTAL_RUNS:.1e}\n\n"
else:
    result_text += f"  P = {p_D:.4e}\n\n"

result_text += (
    f"Constraint factors:\n"
    f"  Window (3.5/10,000 Myr²): {p_A_analytical:.2e}\n"
    f"  Early stellar life:        {p_B_given_A_analytical:.2e}\n"
    f"  E1→E2 gap match:          {p_C_given_AB_analytical:.2e}\n"
    f"  E1→E3 gap match:          {p_D_given_ABC_analytical:.2e}\n\n"
    f"⚠ LAYER 1 ONLY\n"
    f"Layers 2+3 will reduce\n"
    f"this further."
)
ax3.text(0.05, 0.95, result_text, transform=ax3.transAxes,
         fontsize=10, verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.tight_layout()
plt.savefig('rpd_layer1B_results.png', dpi=150, bbox_inches='tight')
plt.show()

print(f"\n  Plot saved: rpd_layer1B_results.png")
print("=" * 65)
print("  Layer 1B complete.")
print("  Next: Layer 2 — Orbital Position Probability")
print("=" * 65)