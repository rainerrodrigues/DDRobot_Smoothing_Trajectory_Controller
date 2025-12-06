# tests/test_basic.py
import numpy as np
from smoothing import smooth_path
from trajectory import compute_arc_lengths, time_parameterize
from controller import PurePursuitController
from sim import simulate_unicycle

def test_smoothing_basic():
    wps = [(0,0),(1,0),(2,0)]
    path, s, csx, csy = smooth_path(wps, n_samples=50)
    assert path.shape[0] == 50
    assert abs(path[0,0]) < 1e-6
    assert path[-1,0] > 1.9

def test_time_param_basic():
    wps = [(0,0),(1,0),(2,0)]
    path, s, csx, csy = smooth_path(wps, n_samples=50)
    t, v = time_parameterize(path, s, csx, csy, V_cruise=0.5)
    assert (t >= 0).all()
    assert len(t) == len(path)

def test_controller_reaches_goal():
    wps = [(0,0),(1,0),(2,0)]
    path, s, csx, csy = smooth_path(wps, n_samples=200)
    controller = PurePursuitController(path, lookahead=0.3, v_cmd=0.2)
    times, states = simulate_unicycle((0.0, -0.1, 0.0), controller, path, t_final=10.0, dt=0.05)
    # final state near last waypoint
    final = states[-1]
    dist = ((final[0] - path[-1,0])**2 + (final[1] - path[-1,1])**2)**0.5
    assert dist < 0.25

