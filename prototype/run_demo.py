# run_demo.py
import numpy as np
import matplotlib.pyplot as plt

from smoothing import smooth_path
from trajectory import compute_arc_lengths, time_parameterize
from controller import PurePursuitController
from sim import simulate_unicycle

def main():
    # Example waypoints (you can change these)
    waypoints = [
        (0.0, 0.0),
        (1.0, 0.2),
        (2.0, -0.2),
        (3.0, 0.5),
        (4.0, 0.0),
        (5.0, 0.0)
    ]

    # 1) Smooth
    path, s_samples, csx, csy = smooth_path(waypoints, n_samples=1200)

    # 2) Time parameterize (simple)
    t_traj, v_profile = time_parameterize(path, s_samples, csx, csy,
                                          V_cruise=0.6, a_max=0.5,
                                          curvature_slowdown=True, kappa_gain=0.4)

    # 3) Controller & simulation
    controller = PurePursuitController(path, lookahead=0.6, v_cmd=0.55, slowdown_gain=2.0)
    times, states = simulate_unicycle((0.0, -0.1, 0.0), controller, path, t_final=60.0, dt=0.02)

    # 4) Plotting
    states = np.array(states)
    plt.figure(figsize=(10,6))
    plt.plot(path[:,0], path[:,1], '--', label='Planned path')
    wps = np.array(waypoints)
    plt.plot(wps[:,0], wps[:,1], 'ro', label='Waypoints')
    plt.plot(states[:,0], states[:,1], '-', label='Robot')
    plt.axis('equal')
    plt.xlabel('x (m)'); plt.ylabel('y (m)')
    plt.title('Path tracking demo')
    plt.legend()

    # Error plots
    pos_err = np.hypot(states[:,0] - np.interp(times, t_traj, path[:,0], left=path[0,0], right=path[-1,0]),
                       states[:,1] - np.interp(times, t_traj, path[:,1], left=path[0,1], right=path[-1,1]))

    plt.figure(figsize=(8,3))
    plt.plot(times[:len(pos_err)], pos_err)
    plt.xlabel('time (s)'); plt.ylabel('position error (m)'); plt.title('Position error vs time')

    plt.show()

if __name__ == '__main__':
    main()

