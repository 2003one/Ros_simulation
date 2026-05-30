import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np

# ── Real robot dimensions (meters) ────────────────────────────────────────────
L1 = 0.035   # Coxa
L2 = 0.070   # Femur
L3 = 0.075   # Tibia

# ── Standing position ──────────────────────────────────────────────────────────
HOME_PY = 0.08    # foot outward distance from coxa joint
HOME_Z  = -0.10   # foot height below coxa joint (negative = below)

# ── Gait parameters ────────────────────────────────────────────────────────────
STEP_LENGTH = 0.03    # forward/backward foot travel per step (meters)
STEP_HEIGHT = 0.015   # how high the foot lifts during swing (meters)
GAIT_HZ     = 0.5     # full gait cycles per second

# ── Leg order MUST match controllers.yaml exactly ─────────────────────────────
# fl, fr, rl, rr  (3 joints each = 12 total)
LEGS = [
    {'name': 'fl', 'outward_sign': +1, 'phase_offset': 0.0   },
    {'name': 'fr', 'outward_sign': -1, 'phase_offset': np.pi },
    {'name': 'rl', 'outward_sign': +1, 'phase_offset': np.pi },
    {'name': 'rr', 'outward_sign': -1, 'phase_offset': 0.0   },
]


def ik_solve(px, py, pz):
    """
    Analytical IK — foot position (in leg-local frame) → joint angles.

    px : forward(+) / backward(-) offset from home position
    py : outward distance from coxa joint (always positive)
    pz : height relative to coxa joint (negative = below body)
    """
    theta1  = np.arctan2(py, px)
    r       = np.sqrt(px**2 + py**2)
    r_reach = r - L1
    h       = pz
    d       = np.sqrt(r_reach**2 + h**2)

    if d > L2 + L3:
        raise ValueError(f"Unreachable: d={d:.4f} > {L2+L3:.4f}")
    if d < abs(L2 - L3):
        raise ValueError(f"Too close: d={d:.4f} < {abs(L2-L3):.4f}")

    cos_t3 = np.clip((d**2 - L2**2 - L3**2) / (2*L2*L3), -1.0, 1.0)
    theta3  = np.arccos(cos_t3)

    alpha  = np.arctan2(-h, r_reach)
    beta   = np.arctan2(L3*np.sin(theta3), L2 + L3*np.cos(theta3))
    theta2 = alpha - beta

    return theta1, theta2, theta3


def ik_to_urdf(theta1, theta2, theta3, outward_sign):
    """
    Map IK angles → URDF joint angles.

    The URDF coxa extends laterally (±Y), while the IK assumes the leg
    extends along +X, so we apply axis/sign corrections:
      coxa  = outward_sign * (theta1 - π/2)
      femur = π/2 - theta2      (URDF zero = vertical, IK zero = horizontal)
      tibia = -theta3            (URDF bends in opposite direction to IK)
    """
    coxa  = outward_sign * (theta1 - np.pi / 2)
    femur = outward_sign * (np.pi/2 - theta2)
    tibia = -outward_sign * theta3
    return coxa, femur, tibia


def foot_trajectory(t):
    """
    Foot offset at gait phase t ∈ [0, 2π).

    [0,  π) → SWING  : foot lifts and steps forward
    [π, 2π) → STANCE : foot on ground, pushes backward
    """
    if t < np.pi:
        t_n    = t / np.pi
        px_off = STEP_LENGTH * (t_n - 0.5)           # -step/2 → +step/2
        lift   = STEP_HEIGHT * np.sin(np.pi * t_n)   # smooth arc
    else:
        t_n    = (t - np.pi) / np.pi
        px_off = STEP_LENGTH * (0.5 - t_n)           # +step/2 → -step/2
        lift   = 0.0
    return px_off, lift


class WalkForwardNode(Node):

    def __init__(self):
        super().__init__('walk_forward')

        # Publishes to the ros2_control joint position controller
        self.pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        self.timer = self.create_timer(0.02, self.step)   # 50 Hz
        self.phase = 0.0
        self.get_logger().info('Walk forward node started — trot gait')

    def step(self):
        """Advance gait phase and send 12 joint position commands at 50 Hz."""
        self.phase = (self.phase + 2 * np.pi * GAIT_HZ * 0.02) % (2 * np.pi)

        positions = []

        for leg in LEGS:
            t            = (self.phase + leg['phase_offset']) % (2 * np.pi)
            px_off, lift = foot_trajectory(t)

            try:
                t1, t2, t3 = ik_solve(px_off, HOME_PY, HOME_Z + lift)
            except ValueError as e:
                self.get_logger().warn(f"{leg['name']}: {e}")
                return   # skip this timestep entirely if any leg fails

            coxa, femur, tibia = ik_to_urdf(t1, t2, t3, leg['outward_sign'])
            positions.extend([coxa, femur, tibia])

        msg      = Float64MultiArray()
        msg.data = positions
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WalkForwardNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
