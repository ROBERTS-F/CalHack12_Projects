#!/usr/bin/env python3
import copy
import argparse
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp


class Robot:
    # Follow this convention per row: [theta, d, a, alpha]
    ROBOT_DH_TABLES = {
        "so100": [
            [0, 0.0542, 0.0304, np.pi / 2],
            [0, 0.0,    0.1160, 0.0],
            [0, 0.0,    0.1347, 0.0],
            [0, 0.0,    0.0,   -np.pi / 2],
            [0, 0.0609, 0.0,    0.0],  # extend link to include gripper if desired
        ]
    }

    # Mechanical joint limits (6 entries incl. gripper; IK uses 5 DoF)
    MECH_JOINT_LIMITS_LOW = {"so100": np.array([-2.2000, -3.1416, 0.0000, -2.0000, -3.1416, -0.2000])}
    MECH_JOINT_LIMITS_UP  = {"so100": np.array([ 2.2000,  0.2000, 3.1416,  1.8000,  3.1416,  2.0000])}

    # worldTbase (naming in original code: WORLD_T_TOOL)
    WORLD_T_TOOL = {
        "so100": np.array(
            [[ 0.0,  1.0, 0.0,  0.0   ],
             [-1.0,  0.0, 0.0, -0.0453],
             [ 0.0,  0.0, 1.0,  0.0647],
             [ 0.0,  0.0, 0.0,  1.0   ]]
        )
    }

    # nTtool (last DH frame to tool)
    N_T_TOOL = {
        "so100": np.array(
            [[ 0.0,  0.0, -1.0, 0.0],
             [ 1.0,  0.0,  0.0, 0.0],
             [ 0.0, -1.0,  0.0, 0.0],
             [ 0.0,  0.0,  0.0, 1.0]]
        )
    }

    def __init__(self, robot_type="so100"):
        if robot_type not in Robot.ROBOT_DH_TABLES:
            raise ValueError(f"Unknown robot type: {robot_type}. Available: {list(Robot.ROBOT_DH_TABLES.keys())}")
        self.robot_type = robot_type
        self.dh_table = Robot.ROBOT_DH_TABLES[robot_type]
        self.mech_joint_limits_low = Robot.MECH_JOINT_LIMITS_LOW[robot_type]
        self.mech_joint_limits_up  = Robot.MECH_JOINT_LIMITS_UP[robot_type]
        self.worldTbase = Robot.WORLD_T_TOOL[robot_type]
        self.nTtool     = Robot.N_T_TOOL[robot_type]

    def from_dh_to_mech(self, q_dh: np.ndarray) -> np.ndarray:
        """Convert joint positions from DH to mechanical coordinates."""
        beta = np.deg2rad(14.45)  # see dh2.png in README
        q_mech = np.zeros_like(q_dh)
        q_mech[0] =  q_dh[0]
        q_mech[1] = -q_dh[1] - beta
        q_mech[2] = -q_dh[2] + beta
        q_mech[3] = -q_dh[3] - np.pi/2
        q_mech[4] = -q_dh[4] - np.pi/2
        return q_mech

    def from_mech_to_dh(self, q_mech: np.ndarray) -> np.ndarray:
        """Convert joint positions from mechanical to DH coordinates."""
        beta = np.deg2rad(14.45)
        q_dh = np.zeros_like(q_mech)
        q_dh[0] =  q_mech[0]
        q_dh[1] = -q_mech[1] - beta
        q_dh[2] = -q_mech[2] + beta
        q_dh[3] = -q_mech[3] - np.pi/2
        q_dh[4] = -q_mech[4] - np.pi/2
        return q_dh[:-1]  # skip gripper

    def check_joint_limits(self, q_vec: np.ndarray):
        """Assert mechanical joint limits are not exceeded (expects 6 if you include gripper)."""
        for i, q in enumerate(q_vec):
            assert self.mech_joint_limits_low[i] <= q <= self.mech_joint_limits_up[i], (
                f"[ERROR] Joint limits out of bound. J{i+1} = {q}, "
                f"limits=({self.mech_joint_limits_low[i]}, {self.mech_joint_limits_up[i]})"
            )


class RobotUtils:
    @staticmethod
    def calc_distance(p1, p2):
        return np.linalg.norm(p2 - p1)

    @staticmethod
    def inv_homog_mat(T):
        Rm = T[:3, :3]
        t  = T[:3, 3]
        T_inv = np.eye(4)
        T_inv[:3, :3] = Rm.T
        T_inv[:3, 3]  = -Rm.T @ t
        return T_inv

    @staticmethod
    def calc_lin_err(T_current, T_desired):
        return T_desired[:3, 3] - T_current[:3, 3]

    @staticmethod
    def calc_ang_err(T_current, T_desired):
        R_current = T_current[:3, :3]
        R_desired = T_desired[:3, :3]
        return 0.5 * (
            np.cross(R_current[:, 0], R_desired[:, 0]) +
            np.cross(R_current[:, 1], R_desired[:, 1]) +
            np.cross(R_current[:, 2], R_desired[:, 2])
        )

    @staticmethod
    def calc_dh_matrix(dh, theta):
        _, d, a, alpha = dh
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array(
            [[ ct, -st*ca,  st*sa, a*ct],
             [ st,  ct*ca, -ct*sa, a*st],
             [  0,     sa,     ca,   d ],
             [  0,      0,      0,   1 ]]
        )

    @staticmethod
    def calc_an_jac_n(robot, q):
        DOF = len(q)
        J = np.zeros((6, DOF))
        U = np.eye(4)
        for j in reversed(range(DOF)):
            T_j = RobotUtils.calc_dh_matrix(robot.dh_table[j], q[j])
            U = T_j @ U
            px, py = U[0, 3], U[1, 3]
            d = np.array([
                -U[0, 0] * py + U[1, 0] * px,
                -U[0, 1] * py + U[1, 1] * px,
                -U[0, 2] * py + U[1, 2] * px
            ])
            delta = U[2, :3]
            J[:3, j] = d
            J[3:, j] = delta
        return J

    @staticmethod
    def calc_an_jac_0(robot, q, T_current):
        J_end = RobotUtils.calc_an_jac_n(robot, q)
        R_mat = T_current[:3, :3]
        J_transform = np.zeros((6, 6))
        J_transform[:3, :3] = R_mat
        J_transform[3:, 3:] = R_mat
        return J_transform @ J_end

    @staticmethod
    def dls_pseudoinv(J_an, lambda_val=0.001):
        JT = J_an.T
        JJT = J_an @ JT
        return JT @ np.linalg.inv(JJT + lambda_val * np.eye(JJT.shape[0]))


class RobotKinematics:
    def forward_kinematics(self, robot, q):
        baseTn = self._forward_kinematics_baseTn(robot, q)
        return robot.worldTbase @ baseTn @ robot.nTtool

    def _forward_kinematics_baseTn(self, robot, q):
        T = np.eye(4)
        for i in range(len(q)):
            dh = robot.dh_table[i]
            T = T @ RobotUtils.calc_dh_matrix(dh, q[i])
        return T

    def _inverse_kinematics_step_baseTn(self, robot, q_start, T_desired, use_orientation=True, k=0.8, n_iter=50):
        q = copy.deepcopy(q_start)
        for _ in range(n_iter):
            T_current = self._forward_kinematics_baseTn(robot, q)
            err_lin = RobotUtils.calc_lin_err(T_current, T_desired)
            if use_orientation:
                err_ang = RobotUtils.calc_ang_err(T_current, T_desired)
                error = np.concatenate((err_lin, err_ang))
                J_an = RobotUtils.calc_an_jac_0(robot, q, T_current)
            else:
                error = err_lin
                J_an = RobotUtils.calc_an_jac_0(robot, q, T_current)[:3, :]
            if np.linalg.norm(error) < 1e-5:
                break
            J_pinv = RobotUtils.dls_pseudoinv(J_an)
            q += k * (J_pinv @ error)
        return q

    def inverse_kinematics(self, robot, q_start, desired_worldTtool, use_orientation=True, k=0.8, n_iter=50):
        desired_baseTn = (
            RobotUtils.inv_homog_mat(robot.worldTbase)
            @ desired_worldTtool
            @ RobotUtils.inv_homog_mat(robot.nTtool)
        )
        q = copy.deepcopy(q_start)
        n_steps = self._interp_init(self._forward_kinematics_baseTn(robot, q), desired_baseTn)
        for i in range(0, n_steps + 1):
            T_desired_interp = self._interp_execute(i)
            q = self._inverse_kinematics_step_baseTn(robot, q, T_desired_interp, use_orientation, k, n_iter)
        current_worldTtool = self.forward_kinematics(robot, q)
        err_lin = RobotUtils.calc_lin_err(current_worldTtool, desired_worldTtool)
        lin_error_norm = np.linalg.norm(err_lin)
        assert lin_error_norm < 1e-2, (
            f"[ERROR] Large position error ({lin_error_norm:.4f}). "
            f"Check target reachability (position/orientation)"
        )
        return q

    def _interp_init(self, T_start, T_final, delta=0.01):
        assert delta > 0, f"[ERROR] Delta = ({delta:.4f}). Delta must be strictly greater than zero"
        self.t_start = T_start[:3, 3]
        self.t_final = T_final[:3, 3]
        R_start = T_start[:3, :3]
        R_final = T_final[:3, :3]
        times = [0, 1]
        rotations = R.from_matrix([R_start, R_final])
        self.slerp = Slerp(times, rotations)
        dist = RobotUtils.calc_distance(self.t_final, self.t_start)
        self.n_steps = int(np.ceil(dist / delta))
        return self.n_steps

    def _interp_execute(self, i):
        s = 1.0 if self.n_steps == 0 else i / self.n_steps
        t_interp = (1 - s) * self.t_start + s * self.t_final
        R_interp = self.slerp(s).as_matrix()
        T_interp = np.eye(4)
        T_interp[:3, :3] = R_interp
        T_interp[:3, 3] = t_interp
        return T_interp


def main():
    ap = argparse.ArgumentParser(description="Compute IK joint angles for SO-ARM100 (SO100).")
    ap.add_argument("--x", type=float, required=True, help="Target X [m] in WORLD frame")
    ap.add_argument("--y", type=float, required=True, help="Target Y [m] in WORLD frame")
    ap.add_argument("--z", type=float, required=True, help="Target Z [m] in WORLD frame")
    ap.add_argument("--roll",  type=float, default=0.0, help="End-effector roll  [deg] (optional)")
    ap.add_argument("--pitch", type=float, default=0.0, help="End-effector pitch [deg] (optional)")
    ap.add_argument("--yaw",   type=float, default=0.0, help="End-effector yaw   [deg] (optional)")
    ap.add_argument("--ori", action="store_true", help="Use orientation in IK (default: position-only)")
    ap.add_argument("--k", type=float, default=0.4, help="IK step size")
    ap.add_argument("--iters", type=int, default=200, help="Iterations per interpolation step")
    args = ap.parse_args()

    # Build goal transform in WORLD frame
    T_goal = np.eye(4)
    T_goal[:3, 3] = np.array([args.x, args.y, args.z])
    if args.ori:
        R_goal = R.from_euler("xyz", np.deg2rad([args.roll, args.pitch, args.yaw])).as_matrix()
        T_goal[:3, :3] = R_goal

    robot = Robot("so100")
    ik = RobotKinematics()

    # Initial guess (DH coordinates)
    q0_dh = np.zeros(5)

    # Solve IK
    q_dh = ik.inverse_kinematics(
        robot,
        q_start=q0_dh,
        desired_worldTtool=T_goal,
        use_orientation=bool(args.ori),
        k=args.k,
        n_iter=args.iters,
    )

    # Verify FK and compute linear error
    T_chk = ik.forward_kinematics(robot, q_dh)
    pos_err = np.linalg.norm(RobotUtils.calc_lin_err(T_chk, T_goal))

    # Convert to mechanical angles (for eventual commanding)
    q_mech = robot.from_dh_to_mech(q_dh)

    # Pretty prints
    def rd(x): return np.round(x, 6)
    def deg(x): return np.round(np.rad2deg(x), 3)

    print("\n=== SO100 Inverse Kinematics ===")
    print(f"Target position (m):  [{args.x:.3f}, {args.y:.3f}, {args.z:.3f}]")
    if args.ori:
        print(f"Target RPY (deg):    [{args.roll:.2f}, {args.pitch:.2f}, {args.yaw:.2f}] (used)")
    else:
        print("Target orientation:   not used (position-only solve)")
    print("DH angles (rad):      ", rd(q_dh))
    print("DH angles (deg):      ", deg(q_dh))
    print("Mechanical (rad):     ", rd(q_mech))
    print("Mechanical (deg):     ", deg(q_mech))
    print("Position error (m):   ", rd(pos_err))


if __name__ == "__main__":
    main()
