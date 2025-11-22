#!/usr/bin/env python3
"""
================================================================
Trajectory Tracking Controller v2 - With Settling Detection
================================================================
FIXES:
1. Waits for pendulum to settle at hanging position (α≈0, β≈0)
2. Validates initial state before starting trajectory
3. Active damping during settling phase
4. Proper angle wrapping for continuous joints
================================================================
"""

import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String, Float64
from geometry_msgs.msg import Vector3Stamped, PointStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import os
from ament_index_python.packages import get_package_share_directory


class TrajectoryTrackingController(Node):
    
    # Controller states
    STATE_WAITING = 0      # Waiting for settling
    STATE_SETTLING = 1     # Active damping to settle pendulum
    STATE_READY = 2        # Ready to start trajectory
    STATE_TRACKING = 3     # Following trajectory
    STATE_HOLDING = 4      # Holding at end position
    
    def __init__(self):
        super().__init__('trajectory_tracking_controller')
        
        self._declare_parameters()
        self._get_parameters()
        self._load_trajectory()
        self._load_lqr_gains()
        
        # State vector: [theta, alpha, beta, theta_dot, alpha_dot, beta_dot, int_theta_error]
        self.current_state = np.zeros(7)
        self.state_received = False
        
        # Controller state machine
        self.ctrl_state = self.STATE_WAITING
        self.trajectory_time = 0.0
        self.theta_integral = 0.0
        
        # Settling detection
        self.settle_start_time = None
        self.settling_samples = []
        
        # Restart logic
        self.stall_start_time = None
        self.ref_stable_start_time = None
        self.prev_ref_alpha = 0.0
        self.prev_ref_beta = 0.0
        
        # Publishers/Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, self.joint_states_topic, self.joint_state_callback, 10)
        self.effort_pub = self.create_publisher(
            Float64MultiArray, self.effort_command_topic, 10)
        self.mode_pub = self.create_publisher(String, '/controller_mode', 10)
        
        self.ref_alpha_pub = self.create_publisher(Float64, '/reference/alpha', 10)
        self.ref_beta_pub = self.create_publisher(Float64, '/reference/beta', 10)
        self.ref_theta_pub = self.create_publisher(Float64, '/reference/theta', 10)
        self.error_pub = self.create_publisher(PointStamped, '/tracking/error', 10)
        
        if self.publish_debug_info:
            self.debug_pub = self.create_publisher(Vector3Stamped, self.debug_topic, 10)
        
        self.control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(self.control_period, self.control_loop)
        
        self._log_controller_info()

    def _declare_parameters(self):
        self.declare_parameter('control_frequency', 1000.0)
        self.declare_parameter('Q_diag', [1.0, 500.0, 800.0, 1.0, 1.0, 1.0, 1.0])
        self.declare_parameter('R_weight', 0.1)
        self.declare_parameter('trajectory_states_file', 'rdip_trajectory_states.csv')
        self.declare_parameter('trajectory_control_file', 'rdip_trajectory_control.csv')
        self.declare_parameter('lqr_gains_file', 'rdip_lqr_gains_timevarying.csv')
        self.declare_parameter('max_torque', 20.0)
        self.declare_parameter('enable_saturation', True)
        self.declare_parameter('joint_names',
            ['revolute_joint', 'first_pendulum_joint', 'second_pendulum_joint'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY))
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('effort_command_topic', '/effort_controller/commands')
        self.declare_parameter('publish_debug_info', True)
        self.declare_parameter('debug_topic', '/trajectory_tracking_debug')
        
        # Settling parameters
        self.declare_parameter('settling_position_threshold', 0.15)   # rad - must be within this of 0
        self.declare_parameter('settling_velocity_threshold', 0.1)    # rad/s
        self.declare_parameter('settling_duration', 0.5)              # seconds to stay settled
        self.declare_parameter('max_settling_time', 10.0)             # max time to wait for settling
        self.declare_parameter('use_active_damping', True)            # apply damping during settling
        self.declare_parameter('damping_gain', 0.5)                   # damping coefficient
        
        # Restart params
        self.declare_parameter('auto_restart_enabled', True)
        self.declare_parameter('stall_velocity_threshold', 0.1)
        self.declare_parameter('stall_error_threshold', 0.5)
        self.declare_parameter('stall_duration', 2.0)
        self.declare_parameter('hold_duration', 10.0)
    
    def _get_parameters(self):
        self.control_frequency = self.get_parameter('control_frequency').value
        self.Q_diag = self.get_parameter('Q_diag').value
        self.R_weight = self.get_parameter('R_weight').value
        self.traj_states_file = self.get_parameter('trajectory_states_file').value
        self.traj_control_file = self.get_parameter('trajectory_control_file').value
        self.lqr_gains_file = self.get_parameter('lqr_gains_file').value
        self.max_torque = self.get_parameter('max_torque').value
        self.enable_saturation = self.get_parameter('enable_saturation').value
        self.joint_names = self.get_parameter('joint_names').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.effort_command_topic = self.get_parameter('effort_command_topic').value
        self.publish_debug_info = self.get_parameter('publish_debug_info').value
        self.debug_topic = self.get_parameter('debug_topic').value
        
        self.settle_pos_thresh = self.get_parameter('settling_position_threshold').value
        self.settle_vel_thresh = self.get_parameter('settling_velocity_threshold').value
        self.settle_duration = self.get_parameter('settling_duration').value
        self.max_settle_time = self.get_parameter('max_settling_time').value
        self.use_active_damping = self.get_parameter('use_active_damping').value
        self.damping_gain = self.get_parameter('damping_gain').value
        
        self.auto_restart = self.get_parameter('auto_restart_enabled').value
        self.stall_vel_thresh = self.get_parameter('stall_velocity_threshold').value
        self.stall_err_thresh = self.get_parameter('stall_error_threshold').value
        self.stall_duration = self.get_parameter('stall_duration').value
        self.hold_duration = self.get_parameter('hold_duration').value
    
    def _load_trajectory(self):
        try:
            pkg_share = get_package_share_directory('double_inverted_pendulum')
            traj_dir = os.path.join(pkg_share, 'trajectories')
            states_path = os.path.join(traj_dir, self.traj_states_file)
            control_path = os.path.join(traj_dir, self.traj_control_file)
            
            if not os.path.exists(states_path):
                states_path = self.traj_states_file
                control_path = self.traj_control_file
            
            states_df = pd.read_csv(states_path)
            control_df = pd.read_csv(control_path)
            
            self.traj_time = states_df['time'].values
            self.traj_states = states_df[['theta', 'alpha', 'beta', 
                                          'theta_dot', 'alpha_dot', 
                                          'beta_dot', 'int_theta']].values
            self.traj_control = control_df['u'].values
            self.T_total = self.traj_time[-1]
            
            self.traj_initial = self.traj_states[0, :3].copy()
            
            self.get_logger().info(f'Loaded trajectory: {self.T_total:.2f}s')
            self.get_logger().info(f'  Expected start: α={self.traj_initial[1]:.3f}, β={self.traj_initial[2]:.3f}')
        except Exception as e:
            self.get_logger().error(f'Failed to load trajectory: {e}')
            raise
    
    def _load_lqr_gains(self):
        try:
            pkg_share = get_package_share_directory('double_inverted_pendulum')
            gains_path = os.path.join(pkg_share, 'trajectories', self.lqr_gains_file)
            if not os.path.exists(gains_path):
                gains_path = self.lqr_gains_file
            if os.path.exists(gains_path):
                gains_df = pd.read_csv(gains_path)
                self.lqr_gains = gains_df[['K1', 'K2', 'K3', 'K4', 'K5', 'K6', 'K7']].values
                self.use_time_varying_lqr = True
                self.get_logger().info('Loaded time-varying LQR gains')
            else:
                self.use_time_varying_lqr = False
                self.constant_K = np.array(self.Q_diag) / self.R_weight
        except Exception as e:
            self.use_time_varying_lqr = False
            self.constant_K = np.array(self.Q_diag) / self.R_weight
    
    def joint_state_callback(self, msg: JointState):
        try:
            idx_arm = msg.name.index(self.joint_names[0])
            idx_pend1 = msg.name.index(self.joint_names[1])
            idx_pend2 = msg.name.index(self.joint_names[2])
            
            theta = msg.position[idx_arm]
            alpha = msg.position[idx_pend1]
            beta = msg.position[idx_pend2]
            theta_dot = msg.velocity[idx_arm] if len(msg.velocity) > idx_arm else 0.0
            alpha_dot = msg.velocity[idx_pend1] if len(msg.velocity) > idx_pend1 else 0.0
            beta_dot = msg.velocity[idx_pend2] if len(msg.velocity) > idx_pend2 else 0.0
            
            self.current_state = np.array([theta, alpha, beta, 
                                          theta_dot, alpha_dot, beta_dot, 
                                          self.theta_integral])
            self.state_received = True
        except (ValueError, IndexError):
            pass

    def wrap_to_pi(self, angle):
        """Wrap angle to [-π, π]"""
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def compute_angle_error(self, ref, actual):
        """Compute shortest angular error: ref - actual"""
        return self.wrap_to_pi(ref - actual)
    
    def is_pendulum_settled(self):
        """Check if pendulum is at expected starting position"""
        alpha = self.wrap_to_pi(self.current_state[1])
        beta = self.wrap_to_pi(self.current_state[2])
        alpha_dot = self.current_state[4]
        beta_dot = self.current_state[5]
        
        # Expected start is trajectory initial (usually 0, 0)
        alpha_target = self.traj_initial[1]
        beta_target = self.traj_initial[2]
        
        alpha_error = abs(self.compute_angle_error(alpha_target, alpha))
        beta_error = abs(self.compute_angle_error(beta_target, beta))
        
        pos_ok = (alpha_error < self.settle_pos_thresh) and (beta_error < self.settle_pos_thresh)
        vel_ok = (abs(alpha_dot) < self.settle_vel_thresh) and (abs(beta_dot) < self.settle_vel_thresh)
        
        return pos_ok and vel_ok, alpha_error, beta_error

    def get_reference_trajectory(self, t):
        if t >= self.T_total: 
            return self.traj_states[-1, :].copy(), 0.0
        elif t <= 0.0: 
            return self.traj_states[0, :].copy(), self.traj_control[0]
        
        idx = np.searchsorted(self.traj_time, t)
        if idx == 0: 
            return self.traj_states[0, :].copy(), self.traj_control[0]
        if idx >= len(self.traj_states): 
            idx = len(self.traj_states) - 1
        
        t0, t1 = self.traj_time[idx-1], self.traj_time[idx]
        ratio = (t - t0) / (t1 - t0)
        
        x_ref = (1 - ratio) * self.traj_states[idx-1, :] + ratio * self.traj_states[idx, :]
        
        idx_ctrl = min(idx, len(self.traj_control) - 1)
        idx_ctrl_prev = min(idx - 1, len(self.traj_control) - 1)
        u_ref = (1 - ratio) * self.traj_control[idx_ctrl_prev] + ratio * self.traj_control[idx_ctrl]
        
        return x_ref, u_ref
    
    def get_lqr_gain(self, t):
        if not self.use_time_varying_lqr: 
            return self.constant_K
        idx = np.searchsorted(self.traj_time, t)
        if idx == 0: 
            return self.lqr_gains[0, :]
        if idx >= len(self.lqr_gains): 
            return self.lqr_gains[-1, :]
        
        t0, t1 = self.traj_time[idx-1], self.traj_time[idx]
        ratio = (t - t0) / (t1 - t0)
        return (1 - ratio) * self.lqr_gains[idx-1, :] + ratio * self.lqr_gains[idx, :]

    def check_for_restart(self, x_ref, x_error):
        if not self.auto_restart:
            return False

        now = self.get_clock().now().nanoseconds / 1e9

        # Stall detection
        is_still = (abs(self.current_state[4]) < self.stall_vel_thresh) and \
                   (abs(self.current_state[5]) < self.stall_vel_thresh)
        is_failed = (abs(x_error[1]) > self.stall_err_thresh) or \
                    (abs(x_error[2]) > self.stall_err_thresh)

        if is_still and is_failed:
            if self.stall_start_time is None:
                self.stall_start_time = now
            elif (now - self.stall_start_time) > self.stall_duration:
                self.get_logger().warn('STALL DETECTED: Restarting...')
                return True
        else:
            self.stall_start_time = None

        # Demo loop - constant reference
        if abs(x_ref[1] - self.prev_ref_alpha) < 1e-6 and \
           abs(x_ref[2] - self.prev_ref_beta) < 1e-6:
            if self.ref_stable_start_time is None:
                self.ref_stable_start_time = now
            elif (now - self.ref_stable_start_time) > self.hold_duration:
                self.get_logger().info('DEMO LOOP: Restarting...')
                return True
        else:
            self.ref_stable_start_time = None
            
        self.prev_ref_alpha = x_ref[1]
        self.prev_ref_beta = x_ref[2]
        
        return False

    def reset_controller(self):
        self.ctrl_state = self.STATE_WAITING
        self.stall_start_time = None
        self.ref_stable_start_time = None
        self.settle_start_time = None
        self.settling_samples = []
        self.theta_integral = 0.0
        self._publish_effort(0.0)
        self.get_logger().info('Controller reset - waiting for settling...')

    def control_loop(self):
        if not self.state_received: 
            return
        
        now = self.get_clock().now().nanoseconds / 1e9
        
        # ============== STATE MACHINE ==============
        
        if self.ctrl_state == self.STATE_WAITING:
            # Initialize settling
            self.settle_start_time = now
            self.settling_samples = []
            self.ctrl_state = self.STATE_SETTLING
            self.get_logger().info('Waiting for pendulum to settle...')
            self._publish_effort(0.0)
            return
        
        elif self.ctrl_state == self.STATE_SETTLING:
            # Check if settled
            is_settled, alpha_err, beta_err = self.is_pendulum_settled()
            
            if is_settled:
                self.settling_samples.append(now)
                # Check if settled for required duration
                if len(self.settling_samples) > 0:
                    settle_time = now - self.settling_samples[0]
                    if settle_time >= self.settle_duration:
                        self.ctrl_state = self.STATE_READY
                        self.get_logger().info(f'Pendulum SETTLED! α_err={alpha_err:.3f}, β_err={beta_err:.3f}')
            else:
                self.settling_samples = []  # Reset if not settled
            
            # Check timeout
            if (now - self.settle_start_time) > self.max_settle_time:
                self.get_logger().warn(f'Settling timeout! α_err={alpha_err:.3f}, β_err={beta_err:.3f}')
                self.get_logger().warn('Starting anyway - tracking may be poor')
                self.ctrl_state = self.STATE_READY
            
            # Apply active damping or zero torque
            if self.use_active_damping:
                tau = -self.damping_gain * self.current_state[3]  # Damp arm velocity
                tau = np.clip(tau, -5.0, 5.0)  # Limit damping torque
            else:
                tau = 0.0
            
            self._publish_effort(tau)
            self._publish_mode()
            return
        
        elif self.ctrl_state == self.STATE_READY:
            # Start trajectory
            self.trajectory_start_time = now
            self.theta_integral = 0.0
            self.ctrl_state = self.STATE_TRACKING
            
            alpha = self.wrap_to_pi(self.current_state[1])
            beta = self.wrap_to_pi(self.current_state[2])
            self.get_logger().info(f'TRAJECTORY STARTED! α={alpha:.3f}, β={beta:.3f}')
        
        # ============== TRACKING / HOLDING ==============
        
        if self.ctrl_state == self.STATE_TRACKING:
            self.trajectory_time = now - self.trajectory_start_time
            
            if self.trajectory_time >= self.T_total:
                self.ctrl_state = self.STATE_HOLDING
                self.get_logger().info('TRAJECTORY COMPLETE - Holding')
        
        # Get reference
        if self.ctrl_state == self.STATE_HOLDING:
            x_ref = self.traj_states[-1, :].copy()
            K = self.lqr_gains[-1, :] if self.use_time_varying_lqr else self.constant_K
            u_ff = 0.0
        else:
            x_ref, u_ff = self.get_reference_trajectory(self.trajectory_time)
            K = self.get_lqr_gain(self.trajectory_time)
        
        # Compute error with proper angle wrapping
        x_error = np.zeros(7)
        x_error[0] = self.compute_angle_error(x_ref[0], self.current_state[0])
        x_error[1] = self.compute_angle_error(x_ref[1], self.current_state[1])
        x_error[2] = self.compute_angle_error(x_ref[2], self.current_state[2])
        x_error[3] = x_ref[3] - self.current_state[3]
        x_error[4] = x_ref[4] - self.current_state[4]
        x_error[5] = x_ref[5] - self.current_state[5]
        
        # Integral of theta error
        self.theta_integral += x_error[0] * self.control_period
        x_error[6] = self.theta_integral
        
        # Check restart
        if self.check_for_restart(x_ref, x_error):
            self.reset_controller()
            return
        
        # Control calculation
        u_fb = np.dot(K, x_error)
        tau = u_ff + u_fb
        
        if self.enable_saturation:
            tau = np.clip(tau, -self.max_torque, self.max_torque)
        
        self._publish_effort(tau)
        self._publish_mode()
        self._publish_reference(x_ref, x_error)
        if self.publish_debug_info: 
            self._publish_debug(tau, x_error)
    
    def _publish_effort(self, tau):
        msg = Float64MultiArray()
        msg.data = [float(tau), 0.0, 0.0]
        self.effort_pub.publish(msg)
    
    def _publish_mode(self):
        msg = String()
        state_names = ['WAITING', 'SETTLING', 'READY', 'TRACKING', 'HOLDING']
        if self.ctrl_state == self.STATE_TRACKING:
            msg.data = f'TRACKING_{self.trajectory_time:.2f}s'
        else:
            msg.data = state_names[self.ctrl_state]
        self.mode_pub.publish(msg)
    
    def _publish_debug(self, tau, x_error):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = float(tau)
        msg.vector.y = float(x_error[1])
        msg.vector.z = float(x_error[2])
        self.debug_pub.publish(msg)
    
    def _publish_reference(self, x_ref, x_error):
        self.ref_theta_pub.publish(Float64(data=float(x_ref[0])))
        self.ref_alpha_pub.publish(Float64(data=float(x_ref[1])))
        self.ref_beta_pub.publish(Float64(data=float(x_ref[2])))
        
        error_msg = PointStamped()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.point.x = float(x_error[0])
        error_msg.point.y = float(x_error[1])
        error_msg.point.z = float(x_error[2])
        self.error_pub.publish(error_msg)

    def _log_controller_info(self):
        self.get_logger().info('='*50)
        self.get_logger().info('TRAJECTORY TRACKING v2 (With Settling)')
        self.get_logger().info(f'  Settle threshold: pos={self.settle_pos_thresh:.2f} rad')
        self.get_logger().info(f'  Settle duration: {self.settle_duration:.1f}s')
        self.get_logger().info(f'  Max settle wait: {self.max_settle_time:.1f}s')
        self.get_logger().info(f'  Active damping: {self.use_active_damping}')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()