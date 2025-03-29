import rebound
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Initialize simulation
sim = rebound.Simulation()
sim.units = ('yr', 'AU', 'Msun')
sim.integrator = "ias15"  # High-accuracy integrator
sim.dt = 0.001  # Initial timestep for precision

# Add Sun at origin
sim.add(m=1.0, x=0, y=0, z=0, vx=0, vy=0, vz=0)

# Add Earth Mark Two at 1 AU with circular orbit
G = 4 * np.pi**2  # Gravitational constant in AU^3 / Msun / yr^2
a_earth = 1.0
v_earth = np.sqrt(G / a_earth)
sim.add(m=3.003e-6, x=a_earth, y=0, z=0, vx=0, vy=v_earth, vz=0)  # Earth’s mass ~3.003e-6 Msun

# Define explosion times and positions (third event at 1 AU)
explosion_times = [0, 20, 40]
explosion_positions = [[2.8, 0, 0], [3.0, 0, 0], [1.0, 0, 0]]  # Positions in AU

# Function to add asteroids at a given time and position
def add_asteroids(sim, t, pos):
    r = np.linalg.norm(pos)
    v_orb = np.sqrt(G / r)  # Orbital velocity for circular orbit
    v_esc = np.sqrt(2 * G / r)  # Escape velocity
    v_orb_vec = [0, v_orb, 0]  # Circular orbit in xy-plane
    for _ in range(50):
        # Small random displacement from explosion point
        delta_r = np.random.uniform(-1e-6, 1e-6, 3)
        asteroid_pos = [pos[i] + delta_r[i] for i in range(3)]
        # Random explosion velocity direction
        theta = np.random.uniform(0, 2 * np.pi)
        phi = np.arccos(np.random.uniform(-1, 1))
        # Explosion velocity magnitude up to 1.2 * v_esc
        v_mag = np.random.uniform(0, 1.2 * v_esc)
        v_expl = v_mag * np.array([np.sin(phi) * np.cos(theta),
                                   np.sin(phi) * np.sin(theta),
                                   np.cos(phi)])
        # Total velocity: orbital + explosion
        v_total = [v_orb_vec[j] + v_expl[j] for j in range(3)]
        sim.add(m=0, x=asteroid_pos[0], y=asteroid_pos[1], z=asteroid_pos[2],
                vx=v_total[0], vy=v_total[1], vz=v_total[2])

# Simulation times (0 to 100 years with 0.1-year steps)
times = np.arange(0, 100.1, 0.1)

# Store positions as a list of lists
positions = []

# Run simulation, adding asteroids at specified times
event_idx = 0
for t in times:
    if event_idx < len(explosion_times) and abs(t - explosion_times[event_idx]) < 1e-6:
        add_asteroids(sim, t, explosion_positions[event_idx])
        event_idx += 1
    sim.integrate(t)
    pos = [p.xyz for p in sim.particles]
    positions.append(pos)
    if int(t) % 10 == 0:  # Progress tracking
        print(f"Simulated up to t = {t:.1f} years")

# Organize trajectories for each particle
N_particles = sim.N  # Total particles after all additions (152)
asteroid_trajectories = [[] for _ in range(N_particles)]
for pos in positions:
    for i, p in enumerate(pos):
        asteroid_trajectories[i].append(p)
asteroid_trajectories = [np.array(traj) for traj in asteroid_trajectories]

# 3D Plotting
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Plot Sun as a fixed point
ax.scatter([0], [0], [0], color='yellow', s=100, label='Sun')  # Larger size for visibility

# Plot Earth Mark Two trajectory
earth_traj = asteroid_trajectories[1]
ax.plot(earth_traj[:, 0], earth_traj[:, 1], earth_traj[:, 2], color='blue', label='Earth Mark Two')

# Plot asteroid trajectories for each event
# First event: particles 2 to 51, color pink
for i in range(2, 52):
    traj = asteroid_trajectories[i]
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color='pink', alpha=0.5)

# Second event: particles 52 to 101, color yellow
for i in range(52, 102):
    traj = asteroid_trajectories[i]
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color='yellow', alpha=0.5)

# Third event: particles 102 to 151, color green
for i in range(102, 152):
    traj = asteroid_trajectories[i]
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color='green', alpha=0.5)

# Add dummy lines for legend to represent each event
ax.plot([], [], color='pink', label='First Event (t=0, 2.8 AU)')
ax.plot([], [], color='yellow', label='Second Event (t=20, 3.0 AU)')
ax.plot([], [], color='green', label='Third Event (t=40, 1.0 AU)')

# Set plot limits to focus on the inner solar system
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_zlim([-5, 5])

# Customize plot
ax.set_xlabel('X (AU)')
ax.set_ylabel('Y (AU)')
ax.set_zlabel('Z (AU)')
ax.set_title('Asteroid Trajectories from Three RPD Explosion Events (50 Asteroids per Event)')
ax.legend()
plt.show()