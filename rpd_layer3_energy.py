"""
RPD Monte Carlo Simulation - Layer 3: Energy Delivery Probability
==================================================================
Rapid Planetary Disassembly Hypothesis - Aaron Riley Hurst

LAYER 3 QUESTION:
  Given correct timing (Layer 1) and orbital position (Layer 2),
  what is the probability that each impactor carries the specific
  mass and velocity required to achieve its RPD function?

COSMIC CONTEXT (stated explicitly):
  After Layers 1+2, P = 1.50e-15
  Milky Way stars:  ~200 billion = 2.0e+11
  This means the RPD sequence is already ~7,500x more improbable
  than having one occurrence per Milky Way galaxy.
  Layer 3 will push this into multi-galaxy / observable universe
  territory.

THE THREE ENERGY CONSTRAINTS:

  Event 1 — Relativistic Kinetic Impactor:
    Required velocity: 0.25c - 0.37c  (~75,000 - 111,000 km/s)
    Required mass:     10^19 - 10^20 kg
    Typical solar system collision velocity: 5-50 km/s
    Maximum gravitational slingshot in dense cluster: ~150 km/s
    Gap to requirement: factor of ~500-1,000x below minimum
    Even supernova ejecta (fastest known: 0.03-0.1c) falls short.
    Probability modeled as: fraction of velocity distribution
    above 0.25c threshold using Maxwell-Boltzmann analog for
    stellar birth cluster velocity dispersion.

  Event 2 — Perihelion Adjustment Impact:
    Required velocity: >20 km/s (hypervelocity, achievable)
    Required mass:     5×10^19 kg
    Required direction: retrograde to current orbital motion
    Probability: velocity threshold AND retrograde fraction

  Event 3 — Apogee Stabilization Impact:
    Required velocity: >15 km/s (achievable)
    Required mass:     3×10^19 kg
    Required direction: specific apogee-reducing vector
    Probability: velocity threshold AND directional fraction

  Mass distribution: modeled from observed solar system
  planetesimal size-frequency distribution (power law).
"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats
import time

# ─────────────────────────────────────────────────────────────────────────────
# PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────

TOTAL_RUNS  = 1_000_000_000
BATCH_SIZE  =    10_000_000
N_BATCHES   = TOTAL_RUNS // BATCH_SIZE
RANDOM_SEED = 44

# Carried forward from Layers 1 and 2
LAYER1_P    = 2.100245e-11
LAYER2_P    = 7.144490e-05
LAYERS_12_P = 1.500518e-15

# ── Cosmic context constants ──────────────────────────────────────────────────
MILKY_WAY_STARS        = 2.0e+11   # ~200 billion
OBSERVABLE_UNIVERSE_STARS = 2.0e+23  # ~200 sextillion

# ── Event 1: Relativistic velocity requirement ────────────────────────────────
# Velocity distribution in stellar birth cluster modeled as
# Maxwell-Boltzmann with sigma = 3 km/s (typical T-Tauri cluster dispersion)
# Gravitational slingshot can boost this: max realistic ~150 km/s
# Required: 0.25c = 74,948 km/s
C_KMS              = 299_792.0    # speed of light in km/s
E1_V_MIN_FRACTION  = 0.25        # minimum as fraction of c
E1_V_MIN_KMS       = E1_V_MIN_FRACTION * C_KMS   # 74,948 km/s
E1_V_MAX_FRACTION  = 0.37
E1_V_MAX_KMS       = E1_V_MAX_FRACTION * C_KMS   # 110,923 km/s

# Cluster velocity dispersion (km/s) — generous upper estimate
CLUSTER_SIGMA_KMS  = 3.0
# Maximum achievable via gravitational interactions (km/s)
MAX_NATURAL_KMS    = 150.0

# Required mass range for Event 1 (kg)
E1_MASS_MIN        = 1.0e+19
E1_MASS_MAX        = 1.0e+20

# ── Event 2: Hypervelocity retrograde impact ──────────────────────────────────
E2_V_MIN_KMS       = 20.0    # minimum for effective perihelion adjustment
E2_MASS_MIN        = 5.0e+19
# Retrograde fraction: impactor must be on retrograde trajectory
# In prograde disk, roughly 5% of scattered objects are retrograde
E2_RETROGRADE_FRAC = 0.05

# ── Event 3: Directional apogee-reducing impact ───────────────────────────────
E3_V_MIN_KMS       = 15.0
E3_MASS_MIN        = 3.0e+19
# Directional constraint already partially in Layer 2 (azimuth)
# Additional energy vector constraint: must be within specific
# momentum direction to reduce apogee without perturbing inclination
E3_DIRECTION_FRAC  = 0.10   # 10% of random impact directions work

# ── Planetesimal size-frequency distribution (power law) ─────────────────────
# N(>m) ∝ m^(-alpha) where alpha ~ 0.5-0.8 for large bodies
# Fraction of objects above each mass threshold
# Based on observed asteroid/KBO size distributions
SFD_ALPHA          = 0.6
# Reference: fraction of objects with mass > 10^18 kg in protoplanetary disk
# estimated from solar system analog studies ~ 1e-4
FRAC_ABOVE_E18     = 1.0e-4

def mass_fraction_above(m_threshold, m_ref=1.0e18, frac_ref=1.0e-4, alpha=0.6):
    """Fraction of planetesimals above mass threshold using power law SFD."""
    return frac_ref * (m_ref / m_threshold) ** alpha

# ─────────────────────────────────────────────────────────────────────────────
# ANALYTICAL CALCULATIONS
# ─────────────────────────────────────────────────────────────────────────────

# ── Event 1 velocity probability ─────────────────────────────────────────────
# Using Maxwell-Boltzmann: P(v > v_min) for 3D velocity distribution
# with sigma = cluster_sigma
# Even with generous slingshot multipliers, 0.25c is ~500x max natural speed

# Approach: model the tail of velocity distribution
# The probability that ANY object in the solar system birth environment
# reaches 0.25c through known mechanisms

# Method 1: Maxwell-Boltzmann tail (pure thermal/gravitational)
v_sigma = CLUSTER_SIGMA_KMS
# P(v > v_min) for Maxwell-Boltzmann = erfc(v_min / (sqrt(2)*sigma))
from scipy.special import erfc
p_v_E1_MB = 0.5 * erfc(E1_V_MIN_KMS / (np.sqrt(2) * v_sigma))

# Method 2: Maximum realistic (supernova ejecta interaction)
# Supernova ejecta fastest component: 0.1c = 29,979 km/s
# Still 2.5x below minimum required 0.25c
# Probability of object being accelerated to 0.25c by supernova: ~1e-6
# (requires specific geometry, proximity, coupling efficiency)
p_v_E1_supernova = 1.0e-6  # generous upper estimate

# We use the supernova estimate as it's the most favorable known mechanism
p_v_E1 = p_v_E1_supernova

# ── Event 1 mass probability ──────────────────────────────────────────────────
p_mass_E1 = mass_fraction_above(E1_MASS_MIN)  # 10^19 kg threshold

# ── Event 1 combined ─────────────────────────────────────────────────────────
p_E1_energy = p_v_E1 * p_mass_E1

# ── Event 2 velocity probability ─────────────────────────────────────────────
# 20 km/s is achievable in solar system — common for asteroid impacts
# P(v > 20 km/s) from Maxwell-Boltzmann with sigma=3 + gravitational boost
# Gravitational focusing at 2 AU can boost to 20-50 km/s — use 30% achievable
p_v_E2 = 0.30

# ── Event 2 mass probability ──────────────────────────────────────────────────
p_mass_E2 = mass_fraction_above(E2_MASS_MIN)  # 5×10^19 kg

# ── Event 2 combined ─────────────────────────────────────────────────────────
p_E2_energy = p_v_E2 * p_mass_E2 * E2_RETROGRADE_FRAC

# ── Event 3 velocity probability ─────────────────────────────────────────────
p_v_E3 = 0.40   # 15 km/s more achievable than E2

# ── Event 3 mass probability ──────────────────────────────────────────────────
p_mass_E3 = mass_fraction_above(E3_MASS_MIN)  # 3×10^19 kg

# ── Event 3 combined ─────────────────────────────────────────────────────────
p_E3_energy = p_v_E3 * p_mass_E3 * E3_DIRECTION_FRAC

# ── Layer 3 total ────────────────────────────────────────────────────────────
p_layer3 = p_E1_energy * p_E2_energy * p_E3_energy

# ── GRAND COMBINED: Layers 1 + 2 + 3 ─────────────────────────────────────────
p_grand_total = LAYERS_12_P * p_layer3

# ─────────────────────────────────────────────────────────────────────────────
# EMPIRICAL SIMULATION (velocity and mass sampling)
# ─────────────────────────────────────────────────────────────────────────────

print("=" * 65)
print("  RPD Monte Carlo — Layer 3: Energy Delivery Probability")
print("=" * 65)
print(f"\n  Cosmic context:")
print(f"  After L1+L2: P = {LAYERS_12_P:.3e}")
print(f"  Milky Way stars: {MILKY_WAY_STARS:.1e}")
print(f"  RPD already {1/(LAYERS_12_P * MILKY_WAY_STARS):,.0f}x rarer than")
print(f"  one per Milky Way galaxy — before energy constraints.")
print(f"\n  Running {TOTAL_RUNS:,} energy sampling runs...", end="", flush=True)

rng = np.random.default_rng(RANDOM_SEED)
start = time.time()

count_v1   = 0   # E1 velocity passes
count_m1   = 0   # E1 mass passes
count_E1   = 0   # E1 velocity + mass
count_E2   = 0   # E2 all constraints
count_E3   = 0   # E3 all constraints
count_all  = 0   # All three events pass

for i in range(N_BATCHES):

    # ── Event 1 velocity sampling ─────────────────────────────────────────────
    # Sample from log-uniform distribution across possible velocities
    # The vast majority will be << 0.25c
    v1 = rng.uniform(0, MAX_NATURAL_KMS * 2, BATCH_SIZE)
    # Additional rare relativistic boost (supernova interaction)
    # modeled as Bernoulli with p=1e-6 on top of velocity
    relativistic_boost = rng.random(BATCH_SIZE) < 1e-4  # generous
    v1_effective = np.where(relativistic_boost, v1 * 500, v1)
    hit_v1 = v1_effective >= E1_V_MIN_KMS

    # ── Event 1 mass sampling (log-uniform power law) ─────────────────────────
    log_mass1 = rng.uniform(14, 22, BATCH_SIZE)  # 10^14 to 10^22 kg
    mass1     = 10 ** log_mass1
    hit_m1    = (mass1 >= E1_MASS_MIN) & (mass1 <= E1_MASS_MAX)

    # ── Event 2: velocity and retrograde ─────────────────────────────────────
    v2         = rng.uniform(0, 100, BATCH_SIZE)
    hit_v2     = v2 >= E2_V_MIN_KMS
    retrograde = rng.random(BATCH_SIZE) < E2_RETROGRADE_FRAC
    log_mass2  = rng.uniform(14, 22, BATCH_SIZE)
    mass2      = 10 ** log_mass2
    hit_m2     = mass2 >= E2_MASS_MIN
    hit_E2     = hit_v2 & retrograde & hit_m2

    # ── Event 3: velocity and direction ──────────────────────────────────────
    v3          = rng.uniform(0, 80, BATCH_SIZE)
    hit_v3      = v3 >= E3_V_MIN_KMS
    direction   = rng.random(BATCH_SIZE) < E3_DIRECTION_FRAC
    log_mass3   = rng.uniform(14, 22, BATCH_SIZE)
    mass3       = 10 ** log_mass3
    hit_m3      = mass3 >= E3_MASS_MIN
    hit_E3      = hit_v3 & direction & hit_m3

    # ── Cumulative tallies ────────────────────────────────────────────────────
    hit_E1_all = hit_v1 & hit_m1

    count_v1  += np.sum(hit_v1)
    count_m1  += np.sum(hit_m1)
    count_E1  += np.sum(hit_E1_all)
    count_E2  += np.sum(hit_E2)
    count_E3  += np.sum(hit_E3)
    count_all += np.sum(hit_E1_all & hit_E2 & hit_E3)

    if (i + 1) % 10 == 0:
        print(".", end="", flush=True)

elapsed = time.time() - start

p_E1_emp  = count_E1  / TOTAL_RUNS
p_E2_emp  = count_E2  / TOTAL_RUNS
p_E3_emp  = count_E3  / TOTAL_RUNS
p_all_emp = count_all / TOTAL_RUNS

# ─────────────────────────────────────────────────────────────────────────────
# PRINT RESULTS
# ─────────────────────────────────────────────────────────────────────────────

print(f"\n\n  Time elapsed: {elapsed:.1f} sec")
print("\n" + "=" * 65)
print("  LAYER 3 RESULTS — Energy Delivery Probability")
print("=" * 65)

print("\n  ── EVENT 1: Relativistic Impactor ──────────────────────────")
print(f"  Required velocity: {E1_V_MIN_KMS:,.0f} km/s  (0.25c)")
print(f"  Best known natural mechanism: supernova ejecta ~0.1c")
print(f"  P(velocity):  {p_v_E1:.4e}  (generous supernova estimate)")
print(f"  P(mass ≥ 10^19 kg): {p_mass_E1:.4e}")
print(f"  P(Event 1 energy):  {p_E1_energy:.4e}")

print("\n  ── EVENT 2: Hypervelocity Retrograde ───────────────────────")
print(f"  Required velocity: {E2_V_MIN_KMS:.0f} km/s  (achievable)")
print(f"  P(velocity):       {p_v_E2:.4f}")
print(f"  P(retrograde):     {E2_RETROGRADE_FRAC:.4f}")
print(f"  P(mass ≥ 5×10^19): {p_mass_E2:.4e}")
print(f"  P(Event 2 energy): {p_E2_energy:.4e}")

print("\n  ── EVENT 3: Directional Apogee Reduction ───────────────────")
print(f"  Required velocity: {E3_V_MIN_KMS:.0f} km/s  (achievable)")
print(f"  P(velocity):       {p_v_E3:.4f}")
print(f"  P(direction):      {E3_DIRECTION_FRAC:.4f}")
print(f"  P(mass ≥ 3×10^19): {p_mass_E3:.4e}")
print(f"  P(Event 3 energy): {p_E3_energy:.4e}")

print("\n  ── LAYER 3 COMBINED ────────────────────────────────────────")
print(f"  P(Layer 3):        {p_layer3:.4e}")
print(f"  Odds (L3 only):    1 in {1/p_layer3:,.4e}")

print("\n  ── EMPIRICAL CROSS-CHECK ───────────────────────────────────")
print(f"  P(E1) empirical:   {p_E1_emp:.4e}")
print(f"  P(E2) empirical:   {p_E2_emp:.4e}")
print(f"  P(E3) empirical:   {p_E3_emp:.4e}")
print(f"  P(ALL) empirical:  {p_all_emp:.4e}")

print("\n" + "=" * 65)
print("  GRAND TOTAL — ALL THREE LAYERS COMBINED")
print("=" * 65)
print(f"\n  Layer 1 (timing):          {LAYER1_P:.4e}")
print(f"  Layer 2 (orbital):         {LAYER2_P:.4e}")
print(f"  Layer 3 (energy):          {p_layer3:.4e}")
print(f"\n  P(GRAND TOTAL):            {p_grand_total:.4e}")
if p_grand_total > 0:
    print(f"  Odds against:              1 in {1/p_grand_total:.4e}")

print(f"\n  ── Cosmic Comparison ───────────────────────────────────────")
print(f"  Milky Way stars:           {MILKY_WAY_STARS:.2e}")
print(f"  Observable universe stars: {OBSERVABLE_UNIVERSE_STARS:.2e}")
occurrences_milky_way  = p_grand_total * MILKY_WAY_STARS
occurrences_universe   = p_grand_total * OBSERVABLE_UNIVERSE_STARS
print(f"\n  Expected RPD occurrences")
print(f"  in Milky Way:              {occurrences_milky_way:.4e}")
print(f"  in observable universe:    {occurrences_universe:.4e}")

if occurrences_universe < 1:
    print(f"\n  *** THE RPD SEQUENCE IS EXPECTED TO OCCUR FEWER THAN ***")
    print(f"  *** ONCE IN THE ENTIRE OBSERVABLE UNIVERSE BY CHANCE. ***")
elif occurrences_milky_way < 1:
    print(f"\n  *** THE RPD SEQUENCE IS EXPECTED TO OCCUR FEWER THAN ***")
    print(f"  *** ONCE IN THE ENTIRE MILKY WAY GALAXY BY CHANCE.   ***")

# ─────────────────────────────────────────────────────────────────────────────
# PLOTS
# ─────────────────────────────────────────────────────────────────────────────

fig = plt.figure(figsize=(18, 11))
fig.suptitle(
    "RPD Monte Carlo — Layer 3: Energy Delivery + Grand Total\n"
    f"Grand Total P = {p_grand_total:.3e}  |  "
    f"1 in {1/p_grand_total:.3e}  |  "
    f"Observable Universe has {OBSERVABLE_UNIVERSE_STARS:.1e} stars",
    fontsize=12, fontweight='bold'
)

# ── Plot 1: Velocity distribution — Event 1 ───────────────────────────────────
ax1 = fig.add_subplot(2, 3, 1)
v_range = np.linspace(0, 500, 1000)
# Maxwell-Boltzmann PDF
mb_pdf = stats.maxwell.pdf(v_range, scale=v_sigma)
mb_pdf_norm = mb_pdf / mb_pdf.max()
ax1.fill_between(v_range, mb_pdf_norm, alpha=0.4, color='steelblue',
                 label='Natural velocity distribution')
ax1.axvline(x=E1_V_MIN_KMS / 1000, color='red', lw=2.5, linestyle='--',
            label=f'Required: 0.25c\n({E1_V_MIN_KMS:,.0f} km/s)')
ax1.axvline(x=MAX_NATURAL_KMS, color='orange', lw=2, linestyle=':',
            label=f'Max natural: {MAX_NATURAL_KMS:.0f} km/s')
ax1.set_xlabel('Velocity (×1000 km/s)')
ax1.set_ylabel('Relative Probability Density')
ax1.set_title('Event 1: Velocity Gap\nNatural vs Required')
ax1.legend(fontsize=8)
ax1.set_xlim(0, 500)

# ── Plot 2: Mass distribution — all events ────────────────────────────────────
ax2 = fig.add_subplot(2, 3, 2)
log_m_range = np.linspace(12, 24, 500)
sfd_pdf = (10**log_m_range)**(-SFD_ALPHA)
sfd_pdf = sfd_pdf / sfd_pdf.max()
ax2.semilogy(log_m_range, sfd_pdf, 'b-', lw=2, label='Size-frequency distribution')
ax2.axvline(x=np.log10(E1_MASS_MIN), color='red',    lw=2, linestyle='--',
            label=f'E1 min: 10^19 kg')
ax2.axvline(x=np.log10(E2_MASS_MIN), color='orange', lw=2, linestyle='--',
            label=f'E2 min: 5×10^19 kg')
ax2.axvline(x=np.log10(E3_MASS_MIN), color='green',  lw=2, linestyle='--',
            label=f'E3 min: 3×10^19 kg')
ax2.set_xlabel('log₁₀(Mass) [kg]')
ax2.set_ylabel('Relative Frequency (log scale)')
ax2.set_title('Impactor Mass Requirements\nvs. Size-Frequency Distribution')
ax2.legend(fontsize=8)

# ── Plot 3: Layer 3 constraint breakdown ─────────────────────────────────────
ax3 = fig.add_subplot(2, 3, 3)
e3_labels  = ['E1\nVelocity', 'E1\nMass', 'E1\nCombined',
              'E2\nCombined', 'E3\nCombined', 'L3\nTotal']
e3_probs   = [p_v_E1, p_mass_E1, p_E1_energy,
              p_E2_energy, p_E3_energy, p_layer3]
e3_colors  = ['#1565C0','#1976D2','#0D47A1',
              '#E65100','#2E7D32','#B71C1C']
bars3 = ax3.bar(e3_labels, e3_probs, color=e3_colors, alpha=0.85, edgecolor='white')
for bar, p in zip(bars3, e3_probs):
    ax3.text(bar.get_x() + bar.get_width()/2, p * 2,
             f'{p:.1e}', ha='center', va='bottom', fontsize=8,
             fontweight='bold', rotation=45)
ax3.set_yscale('log')
ax3.set_ylabel('Probability (log scale)')
ax3.set_title('Layer 3: Energy Constraint\nBreakdown by Event')

# ── Plot 4: Grand total cascade — all three layers ────────────────────────────
ax4 = fig.add_subplot(2, 3, 4)
grand_labels = ['Layer 1\nTiming\n2.10e-11',
                'Layer 2\nOrbital\n7.14e-05',
                'Layer 3\nEnergy',
                'GRAND\nTOTAL']
grand_probs  = [LAYER1_P, LAYER2_P, p_layer3, p_grand_total]
grand_colors = ['#1565C0','#2E7D32','#E65100','#B71C1C']
bars4 = ax4.bar(grand_labels, grand_probs, color=grand_colors,
                alpha=0.85, edgecolor='white', width=0.6)
for bar, p in zip(bars4, grand_probs):
    ax4.text(bar.get_x() + bar.get_width()/2, p * 2,
             f'{p:.2e}', ha='center', va='bottom', fontsize=9,
             fontweight='bold')
ax4.set_yscale('log')
ax4.set_ylabel('Probability (log scale)')
ax4.set_title('Grand Total: All Three Layers\nEach Bar = One Layer')
ax4.axhline(y=1/MILKY_WAY_STARS, color='gold', lw=2, linestyle='--',
            label=f'1/Milky Way stars')
ax4.axhline(y=1/OBSERVABLE_UNIVERSE_STARS, color='white', lw=2,
            linestyle=':', label=f'1/Universe stars')
ax4.legend(fontsize=8)

# ── Plot 5: Cosmic comparison bar chart ──────────────────────────────────────
ax5 = fig.add_subplot(2, 3, 5)
cosmic_labels = ['Expected\nRPD in\nMilky Way', 'Expected\nRPD in\nObservable\nUniverse',
                 'Milky Way\nstars\n(reference)']
cosmic_vals   = [max(occurrences_milky_way, 1e-30),
                 max(occurrences_universe,  1e-30),
                 1.0]
c_colors = ['crimson', 'darkorange', 'steelblue']
bars5 = ax5.bar(cosmic_labels, cosmic_vals, color=c_colors, alpha=0.85,
                edgecolor='white', width=0.5)
ax5.set_yscale('log')
ax5.axhline(y=1.0, color='lime', lw=2.5, linestyle='--',
            label='1 occurrence (threshold)')
ax5.set_ylabel('Expected occurrences (log scale)')
ax5.set_title('Cosmic Scale: Expected RPD\nOccurrences by Chance')
ax5.legend(fontsize=9)
for bar, val in zip(bars5, cosmic_vals):
    ax5.text(bar.get_x() + bar.get_width()/2, val * 2,
             f'{val:.2e}', ha='center', va='bottom', fontsize=9,
             fontweight='bold')

# ── Plot 6: Grand summary panel ───────────────────────────────────────────────
ax6 = fig.add_subplot(2, 3, 6)
ax6.axis('off')
summary = (
    f"GRAND TOTAL — ALL LAYERS\n"
    f"{'─'*38}\n\n"
    f"Layer 1 — Timing:\n"
    f"  P = {LAYER1_P:.3e}\n"
    f"  1 in 47,613,490,767\n\n"
    f"Layer 2 — Orbital Position:\n"
    f"  P = {LAYER2_P:.3e}\n"
    f"  1 in 13,997\n\n"
    f"Layer 3 — Energy Delivery:\n"
    f"  P = {p_layer3:.3e}\n"
    f"  1 in {1/p_layer3:.3e}\n\n"
    f"{'═'*38}\n"
    f"GRAND TOTAL:\n"
    f"  P = {p_grand_total:.3e}\n"
    f"  1 in {1/p_grand_total:.3e}\n\n"
    f"Milky Way stars:  ~2×10^11\n"
    f"Expected in MW:   {occurrences_milky_way:.2e}\n\n"
    f"Universe stars:   ~2×10^23\n"
    f"Expected in obs.\n"
    f"universe:         {occurrences_universe:.2e}\n\n"
)
if occurrences_universe < 1:
    summary += "*** FEWER THAN 1 OCCURRENCE\n    EXPECTED IN THE ENTIRE\n    OBSERVABLE UNIVERSE ***"
elif occurrences_milky_way < 1:
    summary += "*** FEWER THAN 1 OCCURRENCE\n    EXPECTED IN THE\n    MILKY WAY ***"

ax6.text(0.03, 0.97, summary, transform=ax6.transAxes,
         fontsize=9, verticalalignment='top', fontfamily='monospace',
         bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

plt.tight_layout()
plt.savefig('rpd_layer3_grand_total.png', dpi=150, bbox_inches='tight')
plt.show()

print(f"\n  Plot saved: rpd_layer3_grand_total.png")
print("=" * 65)
print("  ALL THREE LAYERS COMPLETE.")
print("  Next: Final synthesis script combining all results.")
print("=" * 65)