import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus
import math
import time

class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        # QoS for PX4: Best Effort
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher_ = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_publisher_ = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_publisher_ = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscriber for vehicle status
        self.vehicle_status_subscription = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        self.vehicle_status = VehicleStatus()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        # Timer: 50Hz
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.cmdloop_callback)

        self.offboard_setpoint_counter_ = 0
        self.start_time = time.time()
        self.radius = 5.0
        self.omega = 0.5 # rad/s
        self.takeoff_height = -5.0 # NED frame (negative is up)

        self.get_logger().info("Offboard Control Node Initialized")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_publisher_.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Switching to Offboard mode")

    def cmdloop_callback(self):
        # 1. Publish OffboardControlMode
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(offboard_msg)

        # 2. Logic to Arm and Switch Mode
        if self.offboard_setpoint_counter_ == 250:
            self.engage_offboard_mode()
            self.arm()

        # 3. Trajectory Generation
        trajectory_msg = TrajectorySetpoint()
        trajectory_msg.position = [0.0, 0.0, 0.0]
        trajectory_msg.yaw = 0.0

        if self.offboard_setpoint_counter_ < 250:
            # Phase 1: Wait and arming check (0 - 5s)
            trajectory_msg.position = [0.0, 0.0, 0.0] 
        
        elif self.offboard_setpoint_counter_ < 500:
            # Phase 2: Takeoff (5s - 10s)
            # Hover at 5 meters
            trajectory_msg.position = [0.0, 0.0, self.takeoff_height]
        
        elif self.offboard_setpoint_counter_ < 750:
            # Phase 3: Move to start of circle (10s - 15s)
            # Linearly interpolate to (radius, 0, height) to avoid jerk
            # Logic: We want to go from (0,0) to (r,0)
            # Duration: 250 steps (5s).
            progress = (self.offboard_setpoint_counter_ - 500) / 250.0
            trajectory_msg.position = [
                progress * self.radius,
                0.0,
                self.takeoff_height
            ]
            trajectory_msg.yaw = 0.0 # Facing North

        else:
            # Phase 4: Circle (15s+)
            # Calculate angle based on counter to avoid system time jitter
            # theta = omega * t
            # t starts from 0 relative to phase start
            delta_steps = self.offboard_setpoint_counter_ - 750
            theta = self.omega * (delta_steps * self.timer_period)

            trajectory_msg.position[0] = self.radius * math.cos(theta)
            trajectory_msg.position[1] = self.radius * math.sin(theta)
            trajectory_msg.position[2] = self.takeoff_height
            
            # Yaw to face direction of travel (tangent to circle)
            # Velocity vector is (-r*sin(theta), r*cos(theta))
            # Yaw = atan2(vy, vx)
            trajectory_msg.yaw = math.atan2(
                math.cos(theta), -math.sin(theta))

        trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(trajectory_msg)

        self.offboard_setpoint_counter_ += 1

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Land command sent")

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()

    try:
        rclpy.spin(offboard_control)
    except KeyboardInterrupt:
        # offboard_control.get_logger().info("KeyboardInterrupt, shutting down...")
        # Attempt to land for safety
        try:
            offboard_control.land()
            time.sleep(0.1)
        except Exception:
            pass # Context might be invalid, just exit
    finally:
        offboard_control.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
