# trajectory.py
import numpy as np

def compute_arc_lengths(path):
    diffs = np.hypot(np.diff(path[:,0]), np.diff(path[:,1]))
    s = np.concatenate(([0.0], np.cumsum(diffs)))
    return s

def curvature_from_spline(csx, csy, s_points):
    """Compute curvature kappa(s) from parametric spline csx, csy at s_points."""
    dx = csx.derivative(1)(s_points)
    ddx = csx.derivative(2)(s_points)
    dy = csy.derivative(1)(s_points)
    ddy = csy.derivative(2)(s_points)
    denom = (dx*dx + dy*dy)**1.5
    # protect denom
    denom[denom == 0] = 1e-8
    kappa = (dx * ddy - dy * ddx) / denom
    return kappa

def time_parameterize(path, s_samples, csx=None, csy=None,
                      V_cruise=0.5, a_max=0.5, curvature_slowdown=True,
                      kappa_gain=0.5):
    """
    Simple time parameterization:
      - optional curvature based speed limits
      - integrate dt = ds / v(s)
    Returns:
      t: timestamps for each path sample
      v_profile: speed at each sample
    """
    ds = np.hypot(np.diff(path[:,0]), np.diff(path[:,1]))
    ds = np.concatenate(([0.0], ds))
    N = len(ds)

    v = np.ones(N) * V_cruise

    if curvature_slowdown and csx is not None and csy is not None:
        kappas = curvature_from_spline(csx, csy, s_samples)
        # safe v_max: decrease speed when curvature is high
        eps = 1e-6
        v_curv = kappa_gain / (np.abs(kappas) + eps)
        # clamp between small min and V_cruise
        v = np.minimum(v, np.maximum(0.05, np.minimum(v_curv, V_cruise)))

    # simple trapezoidal acceleration at start and end (ramp)
    # ramp length in seconds roughly
    # here we simply smooth first & last few samples based on accel
    # compute timestamps
    t = np.zeros(N)
    for i in range(1, N):
        # ensure not dividing by zero
        vi = max(1e-3, v[i])
        t[i] = t[i-1] + ds[i]/vi
    return t, v

