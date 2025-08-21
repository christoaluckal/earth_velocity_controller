import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from deltacan.msg import DeltaCan
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import time

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_control')

        self.declare_parameter('int_number', 5)

        self.publisher_ = self.create_publisher(DeltaCan, '/deltacan', 10)
        self.jointstate_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.goalstate_subscriber = self.create_subscription(Float32MultiArray, '/goal_state', self.goal_callback,10)

        self.goalstate_timer = self.create_timer(0.05, self.goalpub_callback)
        self.goalstate_publisher = self.create_publisher(Float32MultiArray, '/goal_state_out',10)

        self.curr_goal = None
        self.boom_idx = 2
        self.arm_idx = 4
        self.bucket_idx = 5

        self.boom_error = 0
        self.arm_error = 0
        self.bucket_error = 0

        self.boom_sign = -1
        self.arm_sign = 1
        self.bucket_sign = 1

        self.boom_gain = 4
        self.arm_gain = 4
        self.bucket_gain = [1.15, 0.01, 0]

        self.prev_bucket_speed = 0
        self.prev_bucket_vel = 0

        self.curr_error = 0
        self.prev_error = 0
        self.error_sum = 0
        self.dt = 1/20

    def goalpub_callback(self):
        if self.curr_goal is not None:
            msg = Float32MultiArray()
            msg.data = self.curr_goal
            self.goalstate_publisher.publish(msg)


    def joint_callback(self, msg: JointState):
        if self.curr_goal is None:
            return
        # curr_boom_vel = msg.velocity[self.boom_idx]
        # curr_arm_vel = msg.velocity[self.arm_idx]
        curr_bucket_vel = msg.velocity[self.bucket_idx]

        # desired_boom_vel = self.curr_goal[0]
        # desired_arm_vel = self.curr_goal[1]
        desired_bucket_vel = self.curr_goal[2]

        # err_boom = desired_boom_vel - curr_boom_vel
        # err_arm = desired_arm_vel - curr_arm_vel
        self.curr_error = desired_bucket_vel - curr_bucket_vel


        dc_msg = DeltaCan()
        # boom_speed = 0.0
        # arm_speed = 0.0
        bucket_speed = 0.0
        dt = self.dt

        self.error_sum += self.curr_error * dt
        self.error_sum = max(min(self.error_sum, 2), -2)

        if abs(self.curr_error) > 0.05:
            P = self.bucket_gain[0] * self.curr_error
            # Derivative term
            D = -self.bucket_gain[1] * (curr_bucket_vel - self.prev_bucket_vel) / dt

            I = self.bucket_gain[2] * self.error_sum

            bucket_speed = P + I + D
            self.get_logger().info(f'Cur:{self.curr_error}|Sp:{bucket_speed}')
            # if self.curr_error > 0:
            #     if bucket_speed > 1.0:
            #         bucket_speed = 1.0
            #     elif bucket_speed < 0.5:
            #         bucket_speed = 0.5
            # else:
            #     if bucket_speed > -0.5:
            #         bucket_speed = -0.5
            #     elif bucket_speed < -1.0:
            #         bucket_speed = -1.0
        else:
            bucket_speed = self.prev_bucket_speed

        # dc_msg.mboomcmd = boom_speed
        # dc_msg.marmcmd = arm_speed
        dc_msg.mbucketcmd = bucket_speed
        self.prev_error = self.curr_error
        self.prev_bucket_speed = bucket_speed
        self.prev_bucket_vel = curr_bucket_vel
        self.get_logger().info("*"*10)
        self.publisher_.publish(dc_msg)
        time.sleep(0.05)

    def goal_callback(self, msg:Float32MultiArray):
        self.curr_goal = msg.data
        pass

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = VelocityController()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()