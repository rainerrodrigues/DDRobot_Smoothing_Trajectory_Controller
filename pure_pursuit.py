import numpy as np
import math

class PurePursuitController:

    def __init__(self, path, lookahead=0.5, v=0.2):
        self.path = path          # Nx2 numpy array
        self.lookahead = lookahead
        self.v = v

    def control(self, state):
        """
        state = (x, y, theta)
        returns (v, omega)
        """
        x, y, theta = state

        # Find nearest waypoint
        dists = np.linalg.norm(self.path - np.array([x, y]), axis=1)
        idx = np.argmin(dists)

        # Lookahead target
        target_idx = min(idx + 5, len(self.path) - 1)
        tx, ty = self.path[target_idx]

        # Transform into robot frame
        dx = tx - x
        dy = ty - y

        alpha = math.atan2(dy, dx) - theta
        L = self.lookahead

        omega = 2.0 * math.sin(alpha) / L

        return self.v, omega

