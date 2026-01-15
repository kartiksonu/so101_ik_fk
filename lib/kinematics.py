import numpy as np


class RobotKinematics:
    """Robot kinematics using placo library for forward and inverse kinematics."""

    def __init__(
        self,
        urdf_path: str,
        target_frame_name: str = "gripper_frame_link",
        joint_names: list[str] | None = None,
    ):
        """
        Initialize placo-based kinematics solver.

        Args:
            urdf_path (str): Path to the robot URDF file
            target_frame_name (str): Name of the end-effector frame in the URDF
            joint_names (list[str] | None): List of joint names to use for the kinematics solver
        """
        try:
            import placo  # type: ignore[import-not-found]
        except ImportError as e:
            raise ImportError(
                "placo is required for RobotKinematics. "
                "Please install the optional dependencies of `kinematics` in the package."
            ) from e

        self.robot = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)  # Fix the base

        self.target_frame_name = target_frame_name

        # Set joint names
        self.joint_names = list(self.robot.joint_names()) if joint_names is None else joint_names

        # Initialize frame task for IK
        self.tip_frame = self.solver.add_frame_task(self.target_frame_name, np.eye(4))

    def forward_kinematics(self, joint_pos_deg: np.ndarray) -> np.ndarray:
        """
        Compute forward kinematics for given joint configuration given the target frame name in the constructor.

        Args:
            joint_pos_deg: Joint positions in degrees (numpy array)

        Returns:
            4x4 transformation matrix of the end-effector pose
        """

        # Convert degrees to radians
        joint_pos_rad = np.deg2rad(joint_pos_deg[: len(self.joint_names)])

        # Update joint positions in placo robot
        for i, joint_name in enumerate(self.joint_names):
            self.robot.set_joint(joint_name, joint_pos_rad[i])

        # Update kinematics
        self.robot.update_kinematics()

        # Get the transformation matrix
        return self.robot.get_T_world_frame(self.target_frame_name)

    def inverse_kinematics(
        self,
        current_joint_pos: np.ndarray,
        desired_ee_pose: np.ndarray,
        position_weight: float = 1.0,
        orientation_weight: float = 0.01,
        max_iterations: int = 100,
        position_tolerance: float = 0.001,  # 1mm
    ) -> np.ndarray:
        """
        Compute inverse kinematics using placo solver with iterative refinement.

        Args:
            current_joint_pos: Current joint positions in degrees (used as initial guess)
            desired_ee_pose: Target end-effector pose as a 4x4 transformation matrix
            position_weight: Weight for position constraint in IK
            orientation_weight: Weight for orientation constraint in IK, set to 0.0 to only constrain position
            max_iterations: Maximum number of IK iterations for convergence
            position_tolerance: Position error tolerance in meters (default 1mm)

        Returns:
            Joint positions in degrees that achieve the desired end-effector pose
        """
        target_position = desired_ee_pose[:3, 3]
        joint_pos_deg = current_joint_pos[: len(self.joint_names)].copy()

        for _ in range(max_iterations):
            # Convert current joint positions to radians
            joint_pos_rad = np.deg2rad(joint_pos_deg)

            # Set current joint positions
            for i, joint_name in enumerate(self.joint_names):
                self.robot.set_joint(joint_name, joint_pos_rad[i])
            self.robot.update_kinematics()

            # Check current position error
            current_pose = self.robot.get_T_world_frame(self.target_frame_name)
            current_position = current_pose[:3, 3]
            position_error = np.linalg.norm(current_position - target_position)

            if position_error < position_tolerance:
                break

            # Build target pose: current orientation + target position
            target_pose = current_pose.copy()
            target_pose[:3, 3] = target_position

            # Update the target pose for the frame task
            self.tip_frame.T_world_frame = target_pose

            # Configure the task
            self.tip_frame.configure(self.target_frame_name, "soft", position_weight, orientation_weight)

            # Solve IK (one iteration)
            self.solver.solve(True)
            self.robot.update_kinematics()

            # Extract new joint positions
            joint_pos_rad = []
            for joint_name in self.joint_names:
                joint = self.robot.get_joint(joint_name)
                joint_pos_rad.append(joint)
            joint_pos_deg = np.rad2deg(joint_pos_rad)

        # Preserve gripper position if present in current_joint_pos
        if len(current_joint_pos) > len(self.joint_names):
            result = np.zeros_like(current_joint_pos)
            result[: len(self.joint_names)] = joint_pos_deg
            result[len(self.joint_names) :] = current_joint_pos[len(self.joint_names) :]
            return result
        else:
            return np.array(joint_pos_deg)
