import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from deltacan.msg import DeltaCan
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import time
import numpy as np
from scipy.interpolate import CubicSpline, interp1d
from scipy.optimize import minimize


def generate_sine(min_range=0, max_range=1, duration=10, control_freq=10, wave_freq=1, phase=0, noise=0.02):

    dt = 1 / control_freq
    t = np.arange(0, duration, dt)
    y = np.sin(2 * np.pi * wave_freq * t + phase)
    amplitude = (max_range - min_range) / 2
    offset = (max_range + min_range) / 2
    y = y * amplitude + offset
    if noise > 0:
        y += np.random.uniform(-noise, noise, size=y.shape)
        y = np.clip(y, -1, 1)

    return t, y

class VelocityController(Node):

    def __init__(self):
        super().__init__('velocity_control')

        self.declare_parameter('int_number', 5)

        self.publisher_ = self.create_publisher(DeltaCan, '/deltacan', 10)
        self.jointstate_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        # self.goalstate_subscriber = self.create_subscription(Float32MultiArray, '/goal_state', self.goal_callback,10)

        self.goalstate_idxer = self.create_timer(0.05, self.goal_callback)

        self.goalstate_timer = self.create_timer(0.05, self.goalpub_callback)
        self.goalstate_publisher = self.create_publisher(Float32MultiArray, '/goal_state_out',10)

        self.curr_goal = None
        self.boom_idx = 2
        self.arm_idx = 4
        self.bucket_idx = 5

        self.current_state = None

        self.boom_gain = 4
        self.arm_gain = 4
        self.bucket_gain = [2.5,0.005, 0]

        self.prev_bucket_speed = 0
        self.prev_bucket_vel = 0

        self.curr_error = 0
        self.prev_error = 0
        self.integral_error = 0
        self.dt = 1/20

        signal = np.arange(-1.0, 1.0, 0.1)
        power =  np.array([self.bucket_cmd(x) for x in signal])
        if not np.all(np.diff(power) >= 0):
            self.cs = interp1d(power, signal, kind='linear', fill_value='extrapolate')
        else:
            self.cs = CubicSpline(power, signal)

        self.signal_idx = 0
        freq = 1/10
        self.signal_t, self.signal_y = generate_sine(min_range=-1, max_range=1, duration=60, control_freq=20, wave_freq=freq, phase=0, noise=0.0)

    def bucket_cmd(self,x):
        if x < -0.9: return -0.7464773197463939
        elif x < -0.8: return -0.7556077862660108
        elif x < -0.7: return -0.7475886335802124
        elif x < -0.6: return -0.6943159340316355
        elif x < -0.5: return -0.3478137666597351
        elif x < -0.45: return -0.11074819014084174
        elif x >= -0.45 and x <= 0.45: return x*0.005
        elif x < 0.45: return -0.03170411937007261
        elif x < 0.5: return 0.07074540207098638
        elif x < 0.6: return 0.1531096691607231
        elif x < 0.7: return 0.494843938147395
        elif x < 0.8: return 0.9654822056154688
        elif x < 0.9: return 1.1323096288265486
        elif x < 1.0: return 1.124019310302267
        else: return 1.1221103605940879


    def objective(self,x,w):
        error = (w-self.cs(x[0]))**2
        return error

    def optimize_cmd(self,w):
        x_guesses = np.linspace(-1.0, 1.0, 20)
        best_x = None
        best_error = float('inf')
        for x0 in x_guesses:
            result = minimize(self.objective, [x0], args=(w,), bounds=[(-1.0, 1.0)])
            if result.fun < best_error:
                best_error = result.fun
                best_x = result.x[0]
        return best_x


    def goalpub_callback(self):
        if self.curr_goal is not None:
            msg = Float32MultiArray()
            msg.data = self.curr_goal
            self.goalstate_publisher.publish(msg)

        if self.current_state is None:
            return

        dc_msg = DeltaCan()
        # boom_speed = 0.0
        # arm_speed = 0.0
        newbucket_speed = 0.0
        dt = self.dt

        self.curr_error = self.curr_goal[2] - self.current_state[2]

        self.integral_error += self.curr_error * dt
        self.integral_error = max(min(self.integral_error, 2), -2)

        if abs(self.curr_error) > 0.05:
            P = self.bucket_gain[0] * self.curr_error
            # Derivative term
            D = self.bucket_gain[1] * (self.curr_error - self.prev_error) / dt

            I = self.bucket_gain[2] * self.integral_error

            bucket_speed = self.curr_goal[2]

            # newbucket_speed = self.optimize_cmd(bucket_speed)
            # newbucket_speed = self.cs(bucket_speed)

            if type(self.cs) == CubicSpline:
                newbucket_speed = self.optimize_cmd(bucket_speed)
            else:
                newbucket_speed = self.cs(bucket_speed)

            newbucket_speed = max(min(newbucket_speed, 1.0), -1.0)
        else:
            newbucket_speed = self.prev_bucket_speed

        alpha = 1

        dc_msg.mbucketcmd = float(newbucket_speed*alpha + (1-alpha)* self.prev_bucket_speed)
        self.prev_bucket_speed = newbucket_speed
        self.prev_error = self.curr_error
        self.get_logger().info("*"*10)
        self.publisher_.publish(dc_msg)

        

    def joint_callback(self, msg: JointState):
        if self.curr_goal is None:
            return
        # curr_boom_vel = msg.velocity[self.boom_idx]
        # curr_arm_vel = msg.velocity[self.arm_idx]
        curr_bucket_vel = msg.velocity[self.bucket_idx]

        # desired_boom_vel = self.curr_goal[0]
        # desired_arm_vel = self.curr_goal[1]
        desired_bucket_vel = self.curr_goal[2]

        self.current_state = [None, None, curr_bucket_vel]


        

    # def goal_callback(self, msg:Float32MultiArray):
    #     self.curr_goal = msg.data
    #     pass

    def goal_callback(self):
        if self.signal_idx >= len(self.signal_y):
            self.signal_idx = 0
        self.curr_goal = [0.0, 0.0, self.signal_y[self.signal_idx]]
        self.signal_idx += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = VelocityController()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()