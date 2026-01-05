# smoothing.py
import numpy as np
from scipy.interpolate import CubicSpline

def smooth_path(waypoints, n_samples=1000):
    """
    Parametric cubic spline on arc-length.
    waypoints: iterable of (x,y)
    Returns:
      path: (N,2) samples
      s_samples: (N,) arc-length coordinates
      csx, csy: CubicSpline objects (param = s)
    """
    waypoints = np.asarray(waypoints, dtype=float)
    if waypoints.ndim != 2 or waypoints.shape[1] != 2:
        raise ValueError("waypoints must be Nx2")

    xs = waypoints[:, 0]
    ys = waypoints[:, 1]
    # cumulative arc-length
    diffs = np.hypot(np.diff(xs), np.diff(ys))
    s = np.concatenate(([0.0], np.cumsum(diffs)))
    # avoid repeated s values
    for i in range(1, len(s)):
        if s[i] <= s[i-1]:
            s[i] = s[i-1] + 1e-6

    csx = CubicSpline(s, xs, bc_type='natural')
    csy = CubicSpline(s, ys, bc_type='natural')
    s_samples = np.linspace(0.0, s[-1], n_samples)
    xs_s = csx(s_samples)
    ys_s = csy(s_samples)
    path = np.vstack([xs_s, ys_s]).T
    return path, s_samples, csx, csy

