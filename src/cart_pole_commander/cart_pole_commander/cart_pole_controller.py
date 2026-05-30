"""
Robust MRAC Cart-Pole Controller --- Rachit Srivastava
=================================================================

Goal:
    1. Cart position x -> 0
    2. Pole angle theta -> 0

Important:
    Cart-pole has only ONE actuator: cart force F.

Architecture:
    [x, x_dot, theta, theta_dot] ---> Multi-state MRAC ---> Force F

Cart-centering mechanism:
    Cart position error is converted into a small pole angle reference:

        theta_ref = -Kx(x - x_ref) - Kd*x_dot

    This is required because the cart can return to center only by slightly
    leaning the pole.

Control law:
    u = Theta^T Z + W^T Phi(X)

where:
    Z = [x_ref, theta_ref, x, x_dot, theta, theta_dot, int_ex, int_eth]
    X = [x, x_dot, theta, theta_dot]
    Phi(X) = RBF basis functions
    
    
    Nominal adaptive gain vector:
    Theta_nom =
    [
        0.0,       # Kr_x_ref
        -55.15,    # Kr_theta_ref

        0.7,       # Kx_x
        0.8,       # Kx_x_dot
        55.15,     # Kx_theta
        6.21,      # Kx_theta_dot

        2.0,       # Ki_cart
        -121.3     # Ki_pole
    ]

Therefore:
    Kr_nom = [0.0, -55.15]

    Kx_nom = [0.7, 0.8, 55.15, 6.21]

    Ki_nom = [2.0, -121.3]

Reference model:
    y_m = [x_m, theta_m]^T

    For cart:
        omega_x = 0.55
        zeta_x  = 2.0

        x_m_ddot + 2.2*x_m_dot + 0.3025*x_m
        =
        0.3025*x_ref

    For pole:
        omega_theta = 8.0
        zeta_theta  = 0.9

        theta_m_ddot + 14.4*theta_m_dot + 64*theta_m
        =
        64*theta_ref

Adaptive learning rates:
    gamma   = 0.45      # learning rate for Theta
    gamma_w = 0.005     # learning rate for RBF weights W

Sigma modification:
    sigma   = 0.8       # leakage for Theta
    sigma_w = 0.7       # leakage for RBF weights W

Adaptation law:
    Theta_dot = -gamma * e_s * Z - sigma * (Theta - Theta_nom)

    W_dot = -gamma_w * e_s * Phi(X) - sigma_w * W

Composite adaptation error:
    e_s = e_theta + 0.7*e_x + 0.7*int_ex

where:
    e_x     = x_m - x
    e_theta = theta_m - theta

RBF basis function:
    Phi(X) = [phi_1, phi_2, ..., phi_N]

    phi_k = exp(-||X - c_k||^2 / (2*rbf_width^2))

RBF parameters:
    X = [x, x_dot, theta, theta_dot]

    x centers:
        [-1.2, 0.0, 1.2]

    x_dot centers:
        [-1.0, 0.0, 1.0]

    theta centers:
        [-0.35, 0.0, 0.35]

    theta_dot centers:
        [-2.0, 0.0, 2.0]

    Number of RBF basis functions:
        3 x 3 x 3 x 3 = 81

    rbf_width = 0.7

    RBF output bound:
        0 < phi_k <= 1

Parameter bounds:
    gain_clamp = 40.0

    Theta is clipped as:
        Theta_nom - 40 <= Theta <= Theta_nom + 40

    w_clamp = 1.5

    W is clipped as:
        -1.5 <= W <= 1.5

Cart-centering mechanism:
    Cart position error is converted into a small pole angle reference:

        theta_ref = -KX_TO_THETA*(x - x_ref) - KXD_TO_THETA*x_dot

    Current values:
        KX_TO_THETA  = 0.01
        KXD_TO_THETA = 0.05
        THETA_REF_MAX = 0.025 rad

    This is required because the cart can return to center only by slightly
    leaning the pole.
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ═════════════════════════════════════════════════════════════════════════════
# Multi-State Robust MRAC Controller
# ═════════════════════════════════════════════════════════════════════════════

class MIMOMRACController:
    """
    Multi-state Robust MRAC controller.

    State:
        X = [x, x_dot, theta, theta_dot]

    Reference:
        R = [x_ref, theta_ref]

    Regressor:
        Z = [x_ref, theta_ref, x, x_dot, theta, theta_dot, int_ex, int_eth]

    Control:
        u = Theta^T Z + W^T Phi(X)
    """

    def __init__(
        self,
        dt: float,
        output_min: float = -60.0,
        output_max: float = 60.0,
        gamma: float = 0.45,
        gamma_w: float = 0.005,
        sigma: float = 0.8,
        sigma_w: float = 0.7,
        gain_clamp: float = 40.0,
        w_clamp: float = 1.5,
        rbf_width: float = 0.7,
        dead_zone: float = 0.001,
    ):
        self.dt = dt

        self.output_min = output_min
        self.output_max = output_max

        self.gamma = gamma
        self.gamma_w = gamma_w
        self.sigma = sigma
        self.sigma_w = sigma_w

        self.gain_clamp = gain_clamp
        self.w_clamp = w_clamp
        self.rbf_width = rbf_width
        self.dead_zone = dead_zone

        # Reference model states:
        # xm[0] = cart reference model
        # xm[1] = pole reference model
        self.xm = np.zeros(2)
        self.xm_dot = np.zeros(2)

        # Reference model tuning
        # cart model is slower, pole model is faster
        self.omega = np.array([0.55, 8.0])
        self.zeta = np.array([2.0, 0.9])

        # Integral errors: [cart error, pole error]
        self.e_int = np.zeros(2)
        self.e_int_max = np.array([1.0, 0.25])

        # Nominal adaptive gains.
        #
        # Z = [x_ref, theta_ref, x, x_dot, theta, theta_dot, int_ex, int_eth]
        self.Theta_nom = np.array([
            0.0,       # x_ref gain
            -55.15,    # theta_ref gain
            0.7,       # x feedback gain
            0.8,       # x_dot damping
            55.15,     # theta feedback gain
            6.21,      # theta_dot damping
            2.0,       # cart integral gain
            -121.3     # pole integral gain
        ])

        self.Theta = self.Theta_nom.copy()

        # RBF centers in 4D:
        # [x, x_dot, theta, theta_dot]
        x_centers = np.linspace(-1.2, 1.2, 3)
        xd_centers = np.linspace(-1.0, 1.0, 3)
        th_centers = np.linspace(-0.35, 0.35, 3)
        thd_centers = np.linspace(-2.0, 2.0, 3)

        grid = np.meshgrid(
            x_centers,
            xd_centers,
            th_centers,
            thd_centers,
            indexing="ij"
        )

        self.centers = np.column_stack([g.ravel() for g in grid])
        self.n_basis = self.centers.shape[0]

        # Adaptive RBF weights
        self.W = np.zeros(self.n_basis)

        self.prev_error = np.zeros(2)
        self.prev_scalar_error = 0.0

    def reset(self):
        self.xm[:] = 0.0
        self.xm_dot[:] = 0.0
        self.e_int[:] = 0.0
        self.Theta = self.Theta_nom.copy()
        self.W[:] = 0.0
        self.prev_error[:] = 0.0
        self.prev_scalar_error = 0.0

    def _rbf(self, X: np.ndarray) -> np.ndarray:
        """
        RBF basis function vector.

        phi_k = exp(-||X - c_k||^2 / (2 * width^2))

        Each phi_k is bounded:
            0 < phi_k <= 1
        """
        diff = self.centers - X
        sq_dist = np.sum(diff ** 2, axis=1)
        return np.exp(-sq_dist / (2.0 * self.rbf_width ** 2))

    def compute(
        self,
        x_ref: float,
        theta_ref: float,
        x: float,
        x_dot: float,
        theta: float,
        theta_dot: float
    ) -> float:
        dt = self.dt

        # Reference vector
        r = np.array([x_ref, theta_ref])

        # Output vector
        y = np.array([x, theta])

        # Reference model dynamics:
        # xm_ddot = omega^2(r - xm) - 2*zeta*omega*xm_dot
        xm_ddot = self.omega ** 2 * (r - self.xm) \
                  - 2.0 * self.zeta * self.omega * self.xm_dot

        self.xm_dot += xm_ddot * dt
        self.xm += self.xm_dot * dt

        # Vector tracking error
        e = self.xm - y

        # Integral error
        self.e_int += e * dt
        self.e_int = np.clip(self.e_int, -self.e_int_max, self.e_int_max)

        # Composite scalar error for adaptation.
        #
        # e[1]        -> pole angle error
        # e[0]        -> cart position error
        # e_int[0]    -> cart integral error
        #
        # Reduced from previous tuning to reduce oscillations:
        # old: e_s = e[1] + 0.8*e[0] + 0.9*e_int[0]
        # new: e_s = e[1] + 0.7*e[0] + 0.7*e_int[0]
        e_s = e[1] + 0.7 * e[0] + 0.7 * self.e_int[0]

        # Regressor vector
        Z = np.array([
            x_ref,
            theta_ref,
            x,
            x_dot,
            theta,
            theta_dot,
            self.e_int[0],
            self.e_int[1],
        ])

        # RBF state vector
        X = np.array([x, x_dot, theta, theta_dot])
        phi = self._rbf(X)

        # Adaptive control law
        u_linear = float(np.dot(self.Theta, Z))
        u_rbf = float(np.dot(self.W, phi))

        u_raw = u_linear + u_rbf
        u = float(np.clip(u_raw, self.output_min, self.output_max))

        # Adaptation law with sigma modification
        if abs(e_s) > self.dead_zone:
            self.Theta += (
                -self.gamma * e_s * Z
                - self.sigma * (self.Theta - self.Theta_nom)
            ) * dt

            self.W += (
                -self.gamma_w * e_s * phi
                - self.sigma_w * self.W
            ) * dt
        else:
            # Leakage even when error is very small
            self.Theta -= self.sigma * (self.Theta - self.Theta_nom) * dt
            self.W -= self.sigma_w * self.W * dt

        # Clamp adaptive parameters
        self.Theta = np.clip(
            self.Theta,
            self.Theta_nom - self.gain_clamp,
            self.Theta_nom + self.gain_clamp
        )

        np.clip(self.W, -self.w_clamp, self.w_clamp, out=self.W)

        self.prev_error = e.copy()
        self.prev_scalar_error = e_s

        return u

    def get_tracking_error(self):
        return self.prev_error

    def get_scalar_error(self):
        return self.prev_scalar_error

    def get_rbf_norm(self):
        return float(np.linalg.norm(self.W))

    def get_gains(self):
        return self.Theta.copy()

    def describe(self) -> str:
        return (
            f"[Multi-State MRAC]\n"
            f"  omega_cart={self.omega[0]:.2f}, omega_pole={self.omega[1]:.2f}\n"
            f"  zeta_cart={self.zeta[0]:.2f}, zeta_pole={self.zeta[1]:.2f}\n"
            f"  gamma={self.gamma:.4f}, gamma_w={self.gamma_w:.4f}\n"
            f"  sigma={self.sigma:.4f}, sigma_w={self.sigma_w:.4f}\n"
            f"  RBF basis={self.n_basis}, width={self.rbf_width:.3f}\n"
            f"  ||W||={self.get_rbf_norm():.4f}"
        )


# ═════════════════════════════════════════════════════════════════════════════
# ROS 2 Node
# ═════════════════════════════════════════════════════════════════════════════

class CartPoleMIMOMRACNode(Node):
    """
    Cart-pole multi-state MRAC node.

    Subscribes:
        /joint_states

    Publishes:
        /cart_effort_controller/commands

    Goal:
        x -> 0
        theta -> 0
    """

    M = 1.0
    m = 0.2
    l = 0.5
    g = 9.81

    def __init__(self):
        super().__init__("cart_pole_mimo_mrac_controller")

        # ROS interfaces
        self._js_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._js_cb,
            10
        )

        self._cmd_pub = self.create_publisher(
            Float64MultiArray,
            "/cart_effort_controller/commands",
            10
        )

        self.dt = 0.01
        self._timer = self.create_timer(self.dt, self._loop)

        # Joint index cache
        self._ci = None
        self._pi = None
        self._ready = False

        # States
        self.x = 0.0
        self.x_dot = 0.0
        self.theta = 0.0
        self.theta_dot = 0.0

        # Filtered velocities
        self._vfa = 0.35
        self._fxd = 0.0
        self._ftd = 0.0

        # Desired cart and pole center
        self.x_ref = 0.0
        self.theta_ref = 0.0

        # Safety limits
        self.ANGLE_LIMIT = 0.60
        self.POSITION_LIMIT = 1.85

        # Cart-centering to pole-reference conversion.
        #
        # Balanced tuning:
        # KX_TO_THETA gives enough centering authority.
        # KXD_TO_THETA adds damping.
        # THETA_REF_MAX reduced to avoid excessive pole lean and oscillation.
        self.KX_TO_THETA = 0.01
        self.KXD_TO_THETA = 0.05
        self.THETA_REF_MAX = 0.025

        # Multi-state MRAC controller
        self.mimo_mrac = MIMOMRACController(
            dt=self.dt,

            # Moderate force for simulation
            output_min=-60.0,
            output_max=60.0,

            # Reduced RBF adaptation to calm oscillation
            gamma=0.45,
            gamma_w=0.005,

            # Strong leakage prevents drift
            sigma=0.8,
            sigma_w=0.7,

            gain_clamp=40.0,
            w_clamp=1.5,
            rbf_width=0.7,
            dead_zone=0.001,
        )

        self._n = 0

        self.get_logger().info("=" * 70)
        self.get_logger().info("  Cart-Pole Multi-State Robust MRAC")
        self.get_logger().info("=" * 70)
        self.get_logger().info(
            f"  Plant: M={self.M} kg, m={self.m} kg, l={self.l} m, g={self.g}"
        )
        self.get_logger().info(
            "  Goal: cart x -> 0 and pole theta -> 0"
        )
        self.get_logger().info(
            f"  Cart-centering: theta_ref = "
            f"-{self.KX_TO_THETA:.3f}(x-x_ref) - {self.KXD_TO_THETA:.3f}x_dot, "
            f"clipped to ±{self.THETA_REF_MAX:.3f} rad"
        )
        self.get_logger().info(self.mimo_mrac.describe())

    def _js_cb(self, msg: JointState):
        # Resolve joint indices once
        if self._ci is None:
            try:
                self._ci = list(msg.name).index("cart_joint")
                self._pi = list(msg.name).index("pole_joint")
            except ValueError:
                return

        ci = self._ci
        pi = self._pi

        try:
            self.x = msg.position[ci]

            # Normalize pole angle to [-pi, pi]
            self.theta = math.atan2(
                math.sin(msg.position[pi]),
                math.cos(msg.position[pi])
            )

            raw_xd = 0.0
            raw_td = 0.0

            if len(msg.velocity) > max(ci, pi):
                raw_xd = msg.velocity[ci]
                raw_td = msg.velocity[pi]

            # Low-pass filter velocities
            a = self._vfa
            self._fxd = a * raw_xd + (1.0 - a) * self._fxd
            self._ftd = a * raw_td + (1.0 - a) * self._ftd

            self.x_dot = self._fxd
            self.theta_dot = self._ftd

            self._ready = True

        except IndexError:
            self._ready = False

    def _loop(self):
        if not self._ready:
            return

        # Safety stop
        if abs(self.theta) > self.ANGLE_LIMIT or abs(self.x) > self.POSITION_LIMIT:
            self.mimo_mrac.reset()
            self._pub(0.0)

            self._n += 1
            if self._n % 50 == 0:
                self.get_logger().warn(
                    f"SAFETY STOP | "
                    f"|theta|={abs(self.theta):.3f} rad, "
                    f"|x|={abs(self.x):.3f} m"
                )
            return

        # ═══════════════════════════════════════════════════════════════════
        # Cart-centering pole reference
        # ═══════════════════════════════════════════════════════════════════
        #
        # If cart moves away from zero, generate a small theta_ref.
        #
        # x > 0  -> theta_ref negative
        # x < 0  -> theta_ref positive
        #
        # This creates the required lean to move cart back to center.
        theta_ref_raw = (
            -self.KX_TO_THETA * (self.x - self.x_ref)
            -self.KXD_TO_THETA * self.x_dot
        )

        self.theta_ref = float(
            np.clip(theta_ref_raw, -self.THETA_REF_MAX, self.THETA_REF_MAX)
        )

        # Multi-state MRAC directly computes force
        u = self.mimo_mrac.compute(
            x_ref=self.x_ref,
            theta_ref=self.theta_ref,
            x=self.x,
            x_dot=self.x_dot,
            theta=self.theta,
            theta_dot=self.theta_dot
        )

        self._pub(u)

        self._n += 1

        if self._n % 50 == 0:
            e = self.mimo_mrac.get_tracking_error()
            e_s = self.mimo_mrac.get_scalar_error()

            self.get_logger().info(
                f"x={self.x:+.3f} m, "
                f"x_dot={self.x_dot:+.3f} | "
                f"theta={self.theta:+.4f} rad, "
                f"theta_dot={self.theta_dot:+.4f} | "
                f"theta_ref={self.theta_ref:+.4f} rad | "
                f"e_x={e[0]:+.4f}, "
                f"e_theta={e[1]:+.4f}, "
                f"e_s={e_s:+.4f}, "
                f"u={u:+.3f} N"
            )

            if self._n % 500 == 0:
                gains = self.mimo_mrac.get_gains()

                self.get_logger().info(
                    "  Adaptive gains Theta = "
                    + np.array2string(gains, precision=3, suppress_small=True)
                )

                self.get_logger().info(
                    f"  RBF ||W|| = {self.mimo_mrac.get_rbf_norm():.4f}"
                )

    def _pub(self, effort: float):
        msg = Float64MultiArray()
        msg.data = [float(effort)]
        self._cmd_pub.publish(msg)


# ═════════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)

    node = CartPoleMIMOMRACNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub(0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
