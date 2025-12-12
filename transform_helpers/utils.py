from geometry_msgs.msg import Quaternion

import numpy as np
from numpy.typing import NDArray

def rotmat2q(T: NDArray) -> Quaternion:
    # Function that transforms a 3x3 rotation matrix to a ros quaternion representation
    

    if T.shape != (3, 3):
        raise ValueError

    q = Quaternion()
    trace = np.trace(T)

    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q.w = 0.25 * s
        q.x = (T[2, 1] - T[1, 2]) / s
        q.y = (T[0, 2] - T[2, 0]) / s
        q.z = (T[1, 0] - T[0, 1]) / s
    else:
        if T[0, 0] > T[1, 1] and T[0, 0] > T[2, 2]:
            s = np.sqrt(1.0 + T[0, 0] - T[1, 1] - T[2, 2]) * 2.0
            q.w = (T[2, 1] - T[1, 2]) / s
            q.x = 0.25 * s
            q.y = (T[0, 1] + T[1, 0]) / s
            q.z = (T[0, 2] + T[2, 0]) / s
        elif T[1, 1] > T[2, 2]:
            s = np.sqrt(1.0 + T[1, 1] - T[0, 0] - T[2, 2]) * 2.0
            q.w = (T[0, 2] - T[2, 0]) / s
            q.x = (T[0, 1] + T[1, 0]) / s
            q.y = 0.25 * s
            q.z = (T[1, 2] + T[2, 1]) / s
        else:
            s = np.sqrt(1.0 + T[2, 2] - T[0, 0] - T[1, 1]) * 2.0
            q.w = (T[1, 0] - T[0, 1]) / s
            q.x = (T[0, 2] + T[2, 0]) / s
            q.y = (T[1, 2] + T[2, 1]) / s
            q.z = 0.25 * s

    return q
