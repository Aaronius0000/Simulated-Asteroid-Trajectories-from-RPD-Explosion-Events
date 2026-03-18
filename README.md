# RPD Hypothesis — Simulation Code Repository

Supplementary simulation code for:

**"A Supercharged Earth: Evidence for Catastrophic Planetary Disassembly 
and Its Implications for Solar System Architecture and Habitability"**  
Aaron Riley Hurst (2026)  
Submitted to *Icarus* (Editor-in-Chief: Alessandro Morbidelli)

---

## Repository Contents

### 1. Joint Monte Carlo Probability Analysis (Primary Citation)
**`rpd_joint_monte_carlo.py`**  
The primary statistical simulation cited in Sections 8.5 and 10 of the 
manuscript. Evaluates all three constraint families simultaneously in a 
single integrated loop (N = 5 × 10⁷ joint trials). Implements Fréchet 
copula bounds to model timing-energy correlation across ρ = 0.0 to ρ = 1.0.

**Results:**
- P_L1 (timing): 2.10 × 10⁻¹¹
- P_L2 (orbital position): 7.14 × 10⁻⁵
- P_L3 (energy delivery): 7.998 × 10⁻¹⁸
- Joint probability (ρ=0, independent): 1.20 × 10⁻³²
- Joint probability (ρ=1.0, max correlation): 5.71 × 10⁻²²
- Sub-galactic expected frequency at all tested correlation values

---

### 2. Analytical Verification — Original Layer Scripts
Independent analytical verification scripts run at N = 10⁹ per layer 
(Section 10.6). Results are referenced in the sensitivity analysis (Section 10.5.3).

**`rpd_layer1B_billion.py`** — Layer 1 timing probability. Runs 1 billion 
trials with both empirical and exact analytical derivation. Validates 
P(Layer 1) = 2.10 × 10⁻¹¹. Runtime: ~66 seconds.

**`rpd_layer2_orbital.py`** — Layer 2 orbital position probability. Samples 
all four orbital phase constraints across 1 billion trials. Validates 
P(Layer 2) = 7.14 × 10⁻⁵. Runtime: ~42 seconds.

**`rpd_layer3_energy.py`** — Layer 3 energy delivery probability. Models 
relativistic velocity requirements, mass constraints, and directional 
probabilities for all three impact events. Runtime: ~110 seconds.

**`rpd_sensitivity_analysis.py`** — 17 independent parameter sweeps plus 
one combined maximum-generosity sweep. Produces Table 6 of the manuscript.

**`rpd_final_synthesis.py`** — Combines all layer results into summary 
visualizations.

---

### 3. N-Body Trajectory Simulation (Figure 6)
**`RPDNBodyCodePrime3.0.py`**  
REBOUND N-body simulation of asteroid trajectories ejected from three 
sequential RPD explosion events. Models the immediate dispersal of debris 
from precursor-Earth's disruption at ~4.55 Ga, demonstrating how debris 
intersects Earth's orbit and populates the asteroid belt — supporting the 
RPD explanation for the Late Heavy Bombardment and annual meteor showers.

**Simulation setup:**
- Sun at origin; Earth Mark Two at 1 AU circular orbit
- Three explosion events: t=0 yr at 2.8 AU (pink), t=20 yr at 3.0 AU 
  (yellow), t=40 yr at 1.0 AU (green)
- 50 massless asteroids per event, random ejection velocities up to 
  1.2 × escape velocity
- REBOUND ias15 integrator; 100-year run in 0.1-year steps
- Output: 3D trajectory plot limited to ±5 AU

---

## Requirements
```python
Python 3.11+
numpy >= 2.0
scipy >= 1.0
matplotlib >= 3.0
rebound        # for RPDNBodyCodePrime3.0.py only
```

Install all dependencies:
```bash
pip install numpy scipy matplotlib rebound
```

---

## Running the Simulations

**Joint Monte Carlo (primary, ~15 seconds):**
```bash
python rpd_joint_monte_carlo.py
```
Output: printed results table + rpd_joint_monte_carlo.png

**Full analytical verification at N = 10⁹ per layer:**
```bash
python rpd_layer1B_billion.py
python rpd_layer2_orbital.py
python rpd_layer3_energy.py
```

**N-body trajectory simulation:**
```bash
python RPDNBodyCodePrime3.0.py
```
A 3D interactive plot will appear. Use your mouse to rotate and explore.

---

## Fréchet Correlation Sweep

The joint simulation sweeps ρ from 0.0 to 1.0 automatically. The 
interpolation formula is:

P_joint(ρ) = [(1 − ρ) × P_L1 × P_L3 + ρ × min(P_L1, P_L3)] × P_L2

This provides a model-free upper bound on joint probability under any 
positive dependence structure (Fréchet, 1951).

---

## Modifying Parameters (Optional)

**Monte Carlo:** Edit N_TRIALS, BATCH_SIZE, or RHO_VALUES at the top 
of rpd_joint_monte_carlo.py.

**N-body simulation:** Edit explosion_times, explosion_positions, 
number of asteroids per event (range(50)), plot limits, or simulation 
duration in RPDNBodyCodePrime3.0.py.

---

## Citation

If using this code, please cite:

Hurst, A.R., 2026. A Supercharged Earth: Evidence for Catastrophic Planetary 
Disassembly and Its Implications for Solar System Architecture and 
Habitability. *Icarus*, submitted.

Fréchet, M., 1951. Sur les tableaux de corrélation dont les marges sont 
données. Ann. Univ. Lyon Sci., Sect. A 14, 53–77.

---

## Contact

Independent researcher. Brandon, FL.  
GitHub: https://github.com/Aaronius0000
