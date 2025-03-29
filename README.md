# Simulated-Asteroid-Trajectories-from-RPD-Explosion-Events
Overview and Instructions for Reviewers: Simulation of Asteroid Trajectories from RPD Explosion Events
1. Purpose of the Simulation
This simulation models the trajectories of asteroids ejected from three distinct explosion events of a hypothetical super-Earth, "Earth Mark One" (EM1), as part of the Rapid Planetary Disassembly (RPD) Hypothesis. The simulation demonstrates:

The immediate dispersal of debris from EM1’s disruption at ~4.55 Ga, supporting the RPD’s explanation for the Late Heavy Bombardment (LHB) and annual meteor showers.
How debris from these events forms the asteroid belt and intersects Earth’s orbit, contrasting with the Nice Model’s gradual scattering.
The simulation uses the REBOUND library for high-precision N-body integration, NumPy for numerical operations, and Matplotlib for 3D visualization.

2. Installation and Dependencies
To run the simulation, reviewers will need the following Python libraries installed:

REBOUND: For N-body simulation.
NumPy: For numerical computations.
Matplotlib: For plotting the 3D trajectories.
Install these dependencies using pip:

bash

Collapse

Wrap

Copy
pip install rebound numpy matplotlib
Ensure Python 3.x is installed, as the code is written in Python.

3. Simulation Setup
The simulation is set up as follows:

Sun: Placed at the origin (0, 0, 0) with a mass of 1 solar mass.
Earth Mark Two: Positioned at 1 AU from the Sun with a circular orbit in the xy-plane, representing a stable reference body (mass ~3.003e-6 solar masses).
Explosion Events: Three events occur at:
t=0 years, position (2.8, 0, 0) AU.
t=20 years, position (3.0, 0, 0) AU.
t=40 years, position (1.0, 0, 0) AU. Each event ejects 50 massless asteroids with random velocities combining their orbital velocity and an explosion velocity up to 1.2 times the escape velocity from their respective positions.
Asteroids are added to the simulation at each explosion time, and their trajectories are computed using REBOUND’s ias15 integrator for high accuracy, with an initial timestep of 0.001 years.

4. Running the Simulation
The simulation runs from t=0 to t=100 years in 0.1-year steps, totaling 1001 time steps. At each step:

If the current time matches an explosion event (t=0, 20, or 40), 50 asteroids are added with small random displacements (up to ±1e-6 AU) and velocities (orbital + random explosion component).
The simulation integrates forward in time using REBOUND, tracking the positions of all particles (Sun, Earth Mark Two, and up to 150 asteroids by t=40).
Progress is printed every 10 years (e.g., "Simulated up to t = 10.0 years") for tracking.
The total number of particles by the end is 152 (Sun, Earth Mark Two, and 150 asteroids).

5. Plotting the Results
The simulation generates a 3D plot with the following elements:

Sun: Yellow scatter point at (0, 0, 0), size 100 for visibility.
Earth Mark Two: Blue line showing its circular orbit at 1 AU.
Asteroid Trajectories:
Pink: 50 asteroids from the first event (t=0, 2.8 AU), particles 2–51.
Yellow: 50 asteroids from the second event (t=20, 3.0 AU), particles 52–101.
Green: 50 asteroids from the third event (t=40, 1.0 AU), particles 102–151. Each trajectory is plotted with 50% transparency (alpha=0.5) for clarity.
The plot is limited to ±5 AU in all axes (x, y, z) to focus on the inner solar system, where the Sun, Earth Mark Two, and bound asteroids are visible. Escaping asteroids may extend beyond this range but are truncated to maintain focus. Labels and a legend identify the Sun, Earth Mark Two, and each explosion event.

6. Key Points for Reviewers
Relevance to RPD Hypothesis: The simulation illustrates how debris from EM1’s disruption could immediately intersect Earth’s orbit, supporting the RPD’s claims about the LHB and meteor showers. The green trajectories (from 1.0 AU at t=40) are particularly relevant for Earth-intersecting debris, while pink and yellow trajectories (from 2.8 and 3.0 AU) contribute to the asteroid belt.
Inner Solar System Focus: The ±5 AU plot limits ensure visibility of key dynamics, though some escaping asteroids exceed this range.
Randomness in Asteroid Velocities: Asteroids are ejected with random directions and magnitudes (up to 1.2 × escape velocity), modeling the chaotic nature of explosive debris. Reviewers can run the code multiple times to observe different realizations, but the qualitative behavior (some asteroids remain bound, others escape) remains consistent.
7. Usage Instructions
To run the simulation:

Save the Code: Copy the provided script into a file, e.g., simulation.py.
Install Dependencies: Run the pip command above in your terminal.
Execute the Script: In the terminal, navigate to the script’s directory and run:
bash

Collapse

Wrap

Copy
python simulation.py
View the Plot: A 3D plot will appear, showing the trajectories over 100 years. Use your mouse to rotate and explore the view.
Modifying Parameters (Optional):

Explosion Times/Positions: Edit explosion_times = [0, 20, 40] and explosion_positions = [[2.8, 0, 0], [3.0, 0, 0], [1.0, 0, 0]] to test different scenarios.
Number of Asteroids: Change range(50) in add_asteroids to simulate more or fewer asteroids per event.
Plot Limits: Adjust ax.set_xlim([-5, 5]), ax.set_ylim([-5, 5]), and ax.set_zlim([-5, 5]) to zoom in or out.
Simulation Duration/Steps: Modify times = np.arange(0, 100.1, 0.1) to change the time range or step size.
This overview and set of instructions provide reviewers with everything needed to understand, run, and evaluate your simulation code.
