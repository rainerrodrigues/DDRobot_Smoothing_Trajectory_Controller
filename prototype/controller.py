# controller.py
import numpy as np

def wrap_to_pi(angle):
    a = (angle + np.pi) % (2*np.pi) - np.pi
    return a

class PurePursuitController:
    def __init__(self, path, lookahead=0.6, v_cmd=0.4, slowdown_gain=2.0):
        """
        path: Nx2 array of path points (x,y)
        lookahead: meters
        v_cmd: base commanded linear velocity
        slowdown_gain: reduces v with curvature-like effect (optional)
        """
        self.path = np.asarray(path)
        self.lookahead = float(lookahead)
        self.v_cmd = float(v_cmd)
        self.slowdown_gain = slowdown_gain

    def find_goal(self, pos):
        """
        choose a goal point along the path that is at least lookahead distance
        ahead of the closest point.
        """
        dists = np.hypot(self.path[:,0] - pos[0], self.path[:,1] - pos[1])
        idx = int(np.argmin(dists))
        # move forward until arc distance >= lookahead
        L = 0.0
        goal = self.path[-1]
        for j in range(idx, len(self.path)-1):
            L += np.hypot(self.path[j+1,0]-self.path[j,0], self.path[j+1,1]-self.path[j,1])
            if L >= self.lookahead:
                goal = self.path[j+1]
                break
        return goal, idx

    def compute_control(self, state):
        """
        state: (x,y,theta)
        returns v, omega
        """
        x,y,theta = state
        pos = (x,y)
        goal, idx = self.find_goal(pos)
        dx = goal[0] - x
        dy = goal[1] - y
        # rotate into robot frame
        xr =  np.cos(-theta)*dx - np.sin(-theta)*dy
        yr =  np.sin(-theta)*dx + np.cos(-theta)*dy
        # avoid divide by zero
        dist_goal = max(1e-6, np.hypot(xr, yr))
        curvature = 2.0 * yr / (dist_goal**2)
        # optionally reduce v when curvature high
        v = self.v_cmd / (1.0 + self.slowdown_gain * abs(curvature))
        omega = v * curvature
        return float(v), float(omega)

