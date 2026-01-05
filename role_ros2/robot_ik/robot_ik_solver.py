import numpy as np
from dm_control import mjcf
from dm_robotics.moma.effectors import arm_effector, cartesian_6d_velocity_effector

from role_ros2.robot_ik.arm import FrankaArm


class RobotIKSolver:
    def __init__(self):
        self.relative_max_joint_delta = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
        self.max_joint_delta = self.relative_max_joint_delta.max()
        self.max_gripper_delta = 0.25
        self.max_lin_delta = 0.075
        self.max_rot_delta = 0.15
        self.control_hz = 15

        self._arm = FrankaArm()
        self._physics = mjcf.Physics.from_mjcf_model(self._arm.mjcf_model)
        self._effector = arm_effector.ArmEffector(arm=self._arm, action_range_override=None, robot_name=self._arm.name)

        self._effector_model = cartesian_6d_velocity_effector.ModelParams(self._arm.wrist_site, self._arm.joints)

        self._effector_control = cartesian_6d_velocity_effector.ControlParams(
            control_timestep_seconds=1 / self.control_hz,
            max_lin_vel=self.max_lin_delta,
            max_rot_vel=self.max_rot_delta,
            joint_velocity_limits=self.relative_max_joint_delta,
            nullspace_joint_position_reference=[0] * 7,
            nullspace_gain=0.025,
            regularization_weight=1e-2,
            enable_joint_position_limits=True,
            minimum_distance_from_joint_position_limit=0.3,
            joint_position_limit_velocity_scale=0.95,
            max_cartesian_velocity_control_iterations=300,
            max_nullspace_control_iterations=300,
        )

        self._cart_effector_6d = cartesian_6d_velocity_effector.Cartesian6dVelocityEffector(
            self._arm.name, self._effector, self._effector_model, self._effector_control
        )
        # Fix for MuJoCo 2.3.2 and dm-robotics compatibility issue with eq_active
        # Try-except to handle version compatibility
        try:
            self._cart_effector_6d.after_compile(self._arm.mjcf_model, self._physics)
        except AttributeError as e:
            if 'eq_active' in str(e) or 'MjModel' in str(e):
                # Workaround: Recompile physics after creating effector
                # This is a known issue with MuJoCo 2.3.2 and dm-robotics 0.5.0
                import warnings
                warnings.warn(
                    f"MuJoCo/dm_control version compatibility issue detected: {e}\n"
                    "Attempting workaround by recompiling physics..."
                )
                # Recreate physics after effector creation
                self._physics = mjcf.Physics.from_mjcf_model(self._arm.mjcf_model)
                # Try after_compile again
                try:
                    self._cart_effector_6d.after_compile(self._arm.mjcf_model, self._physics)
                except Exception as e2:
                    raise RuntimeError(
                        f"Failed to initialize Cartesian6dVelocityEffector after workaround: {e2}\n"
                        "This may be due to incompatible MuJoCo/dm_control/dm_robotics versions.\n"
                        "Recommended versions: mujoco==2.3.2, dm-control==1.0.5, dm-robotics-moma==0.5.0"
                    ) from e2
            else:
                raise

    ### Inverse Kinematics ###
    def cartesian_velocity_to_joint_velocity(self, cartesian_velocity, robot_state):
        cartesian_delta = self.cartesian_velocity_to_delta(cartesian_velocity)
        qpos = np.array(robot_state["joint_positions"])
        qvel = np.array(robot_state["joint_velocities"])

        self._arm.update_state(self._physics, qpos, qvel)
        self._cart_effector_6d.set_control(self._physics, cartesian_delta)
        joint_delta = self._physics.bind(self._arm.actuators).ctrl.copy()
        np.any(joint_delta)

        joint_velocity = self.joint_delta_to_velocity(joint_delta)

        return joint_velocity

    ### Velocity To Delta ###
    def gripper_velocity_to_delta(self, gripper_velocity):
        gripper_vel_norm = np.linalg.norm(gripper_velocity)

        if gripper_vel_norm > 1:
            gripper_velocity = gripper_velocity / gripper_vel_norm

        gripper_delta = gripper_velocity * self.max_gripper_delta

        return gripper_delta

    def cartesian_velocity_to_delta(self, cartesian_velocity):
        if isinstance(cartesian_velocity, list):
            cartesian_velocity = np.array(cartesian_velocity)

        lin_vel, rot_vel = cartesian_velocity[:3], cartesian_velocity[3:6]

        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)

        if lin_vel_norm > 1:
            lin_vel = lin_vel / lin_vel_norm
        if rot_vel_norm > 1:
            rot_vel = rot_vel / rot_vel_norm

        lin_delta = lin_vel * self.max_lin_delta
        rot_delta = rot_vel * self.max_rot_delta

        # Debug: Log conversion for verification
        # Note: This is called frequently, so only log occasionally
        if hasattr(self, '_debug_count'):
            self._debug_count += 1
        else:
            self._debug_count = 0
        
        if self._debug_count % 15 == 0:  # Log every 15 calls (~1 second at 15 Hz)
            import logging
            logger = logging.getLogger(__name__)
            logger.debug(
                f"🔧 IK: cartesian_velocity_to_delta:\n"
                f"   Input velocity: {cartesian_velocity[:3]} (norm: {lin_vel_norm:.4f})\n"
                f"   Output delta: {lin_delta} (max_lin_delta: {self.max_lin_delta:.4f})"
            )

        return np.concatenate([lin_delta, rot_delta])

    def joint_velocity_to_delta(self, joint_velocity):
        if isinstance(joint_velocity, list):
            joint_velocity = np.array(joint_velocity)

        relative_max_joint_vel = self.joint_delta_to_velocity(self.relative_max_joint_delta)
        max_joint_vel_norm = (np.abs(joint_velocity) / relative_max_joint_vel).max()

        if max_joint_vel_norm > 1:
            joint_velocity = joint_velocity / max_joint_vel_norm

        joint_delta = joint_velocity * self.max_joint_delta

        return joint_delta

    ### Delta To Velocity ###
    def gripper_delta_to_velocity(self, gripper_delta):
        return gripper_delta / self.max_gripper_delta

    def cartesian_delta_to_velocity(self, cartesian_delta):
        if isinstance(cartesian_delta, list):
            cartesian_delta = np.array(cartesian_delta)

        cartesian_velocity = np.zeros_like(cartesian_delta)
        cartesian_velocity[:3] = cartesian_delta[:3] / self.max_lin_delta
        cartesian_velocity[3:6] = cartesian_delta[3:6] / self.max_rot_delta

        return cartesian_velocity

    def joint_delta_to_velocity(self, joint_delta):
        if isinstance(joint_delta, list):
            joint_delta = np.array(joint_delta)

        return joint_delta / self.max_joint_delta
    
    ### Collision Detection ###
    def check_collision(self, joint_positions, joint_velocities=None):
        """
        Check if a given joint configuration is in collision.
        
        Args:
            joint_positions: Joint positions (7 DOF) as list or numpy array
            joint_velocities: Optional joint velocities (7 DOF), defaults to zeros
        
        Returns:
            bool: True if collision detected, False otherwise
        """
        if joint_velocities is None:
            joint_velocities = np.zeros(7)
        
        # Update physics state with given joint configuration
        self._arm.update_state(self._physics, np.array(joint_positions), np.array(joint_velocities))
        
        # Forward kinematics to update all body positions
        self._physics.forward()
        
        # Check for collisions using MuJoCo's contact detection
        # ncon: number of contacts
        # contacts: array of contact information
        ncon = self._physics.data.ncon
        
        # If there are any contacts, there is a collision
        return ncon > 0
    
    def is_configuration_valid(self, joint_positions, joint_velocities=None, check_collision=True):
        """
        Check if a joint configuration is valid (within limits and optionally collision-free).
        
        Args:
            joint_positions: Joint positions (7 DOF) as list or numpy array
            joint_velocities: Optional joint velocities (7 DOF), defaults to zeros
            check_collision: If True, also check for collisions
        
        Returns:
            tuple: (is_valid, reason)
                - is_valid: True if configuration is valid
                - reason: String describing why configuration is invalid (if not valid)
        """
        joint_positions = np.array(joint_positions)
        
        # Check joint limits (using Franka joint limits)
        # These are approximate limits, should match the actual robot
        joint_limits_lower = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
        joint_limits_upper = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
        
        if np.any(joint_positions < joint_limits_lower):
            invalid_joint = np.where(joint_positions < joint_limits_lower)[0][0]
            return False, f"Joint {invalid_joint} below lower limit: {joint_positions[invalid_joint]:.4f} < {joint_limits_lower[invalid_joint]:.4f}"
        
        if np.any(joint_positions > joint_limits_upper):
            invalid_joint = np.where(joint_positions > joint_limits_upper)[0][0]
            return False, f"Joint {invalid_joint} above upper limit: {joint_positions[invalid_joint]:.4f} > {joint_limits_upper[invalid_joint]:.4f}"
        
        # Check for collisions if requested
        if check_collision:
            if self.check_collision(joint_positions, joint_velocities):
                return False, "Collision detected"
        
        return True, "Configuration is valid"