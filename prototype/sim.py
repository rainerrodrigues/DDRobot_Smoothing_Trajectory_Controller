# sim.py
import numpy as np

def simulate_unicycle(initial_state, controller, path, t_final=20.0, dt=0.02):
    """
    Simulate until t_final seconds, or until robot reaches the end of path (within tolerance)
    controller: object with compute_control(state)->(v,omega)
    path: Nx2 array of (x,y) planned points (used by controller)
    Returns:
      times, states (Mx3 array)
    """
    x,y,theta = initial_state
    times = []
    states = []
    t = 0.0
    goal_tol = 0.15
    while t < t_final:
        v, omega = controller.compute_control((x,y,theta))
        # integrate simple unicycle
        x += v * np.cos(theta) * dt
        y += v * np.sin(theta) * dt
        theta += omega * dt
        theta = (theta + np.pi) % (2*np.pi) - np.pi
        times.append(t)
        states.append([x,y,theta])
        t += dt
        # termination: near end of path
        if np.hypot(x - path[-1,0], y - path[-1,1]) < goal_tol:
            # one final append and break
            times.append(t)
            states.append([x,y,theta])
            break
    return np.array(times), np.array(states)

