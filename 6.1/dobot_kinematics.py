import math
from dobot_client import DobotClient

# Link lengths (mm)
L0 = 56.0 
L1 = 82.0
L2 = 135.0
L3 = 147.0
L4 = 60.0
L5 = 70.0

# Create a single client instance
# dobot = DobotClient()


def forward_kinematics(q1, q2, q3, q4):
    """Returns (x, y, z, r) given joint angles in degrees."""
    # if not dobot.is_goal_valid(q1, q2, q3, q4):
    #     raise ValueError(f"Joint goal out of range: {(q1, q2, q3, q4)}")

    q1r, q2r, q23r = math.radians(q1), math.radians(q2), math.radians(q2 + q3)
    reach = 49 * math.cos(q23r) + 45 * math.sin(q2r) + 20
    x = 3 * math.cos(q1r) * reach
    y = 3 * math.sin(q1r) * reach
    z = 135 * math.cos(q2r) - 147 * math.sin(q23r) + 68
    r = q1 + q4

    print(f"FK({q1}, {q2}, {q3}, {q4}) → x={x:.4f}, y={y:.4f}, z={z:.4f}, r={r:.4f}")
    return x, y, z, r


def inverse_kinematics(x, y, z, r):
    """Returns (q1, q2, q3, q4) in degrees given end-effector pose."""
    q1 = math.degrees(math.atan2(y, x))
    d  = math.sqrt(x**2 + y**2) - L4
    h  = z + L5 - L0 - L1
    c  = math.sqrt(d**2 + h**2)
    alpha = math.atan2(d, h)
    beta  = math.acos((L2**2 + c**2 - L3**2) / (2 * L2 * c))
    q2 = math.degrees(alpha - beta)
    q3 = math.degrees(math.asin((L2**2 + L3**2 - c**2) / (2 * L2 * L3)))
    q4 = r - q1
    
    # if not dobot.is_goal_valid(q1, q2, q3, q4):
    #     raise ValueError(f"Computed joint goal is invalid: {(q1, q2, q3, q4)}")

    print(f"IK({x:.2f}, {y:.2f}, {z:.2f}, {r:.2f}) → q1={q1:.4f}, q2={q2:.4f}, q3={q3:.4f}, q4={q4:.4f}")
    return q1, q2, q3, q4