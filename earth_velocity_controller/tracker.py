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
import pandas as pd
import matplotlib.pyplot as plt

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
        super().__init__('velocity_tracker')

        self.declare_parameter('dt', 1/55)
        self.dt = self.get_parameter('dt').value

        self.publisher_ = self.create_publisher(DeltaCan, '/twist_deltacan', 10)
        self.jointstate_subscriber = self.create_subscription(JointState, '/takeuchi_excavator_base_node/joint_state', self.joint_callback, 10)
        # self.goalstate_subscriber = self.create_subscription(Float32MultiArray, '/goal_state', self.goal_callback,10)

        self.goalstate_idxer = self.create_timer(self.dt, self.goal_callback)

        self.goalstate_timer = self.create_timer(self.dt, self.goalpub_callback)
        self.goalstate_publisher = self.create_publisher(DeltaCan, '/goal_state_out',10)

        self.curr_goal = None

        self.current_state = None


        self.prev_bucket_speed = 0
        self.prev_arm_speed = 0
        self.prev_boom_speed = 0

        self.curr_error = 0
        self.curr_error_bucket = 0
        self.curr_error_arm = 0
        self.curr_error_boom = 0

        self.prev_error_bucket = 0
        self.prev_error_arm = 0
        self.prev_error_boom = 0

        self.boom_idx = 1
        self.arm_idx = 2
        self.bucket_idx = 3

        self.bucket_min = 0.05
        self.arm_min = 0.05
        self.boom_min = 0.05

        
        signal = np.arange(-1.0, 1.0, 0.1)
        bucket_speed =  np.array([self.bucket_cmd(x) for x in signal])
        if not np.all(np.diff(bucket_speed) >= 0):
            self.bucket_cs = interp1d(bucket_speed, signal, kind='linear', fill_value='extrapolate')
        else:
            self.bucket_cs = CubicSpline(bucket_speed, signal)
        
        arm_speed =  np.array([self.arm_cmd(x) for x in signal])
        if not np.all(np.diff(arm_speed) >= 0):
            self.arm_cs = interp1d(arm_speed, signal, kind='linear', fill_value='extrapolate')
        else:
            self.arm_cs = CubicSpline(arm_speed, signal)

        boom_speed =  np.array([self.boom_cmd(x) for x in signal])
        if not np.all(np.diff(boom_speed) >= 0):
            self.boom_cs = interp1d(boom_speed, signal, kind='linear', fill_value='extrapolate')
        else:
            self.boom_cs = CubicSpline(boom_speed, signal)

        self.signal_idx = 0
        freq = 1/10
        duration = 10

        _, self.signal_bucket = generate_sine(min_range=-1, max_range=1, duration=duration, control_freq=int(1/self.dt), wave_freq=freq, phase=np.pi, noise=0.0)
        self.signal_t, self.signal_arm = generate_sine(min_range=-1, max_range=1, duration=duration, control_freq=int(1/self.dt), wave_freq=freq, phase=0, noise=0.0)
        _, self.signal_boom = generate_sine(min_range=-1, max_range=1, duration=duration, control_freq=int(1/self.dt), wave_freq=freq, phase=np.pi/2, noise=0.0)

        bucket_mask = abs(self.signal_bucket) > self.bucket_min
        self.signal_bucket[bucket_mask == False] = 0.0


        self.signal_bucket *= 1.0
        self.signal_arm *= 0.0
        self.signal_boom *= 0.0

        self.kp_bucket = 1.0
        self.ki_bucket = 0.0
        self.kd_bucket = 0.1

        self.kp_arm = 1.0
        self.ki_arm = 0.0
        self.kd_arm = 0.1

        self.kp_boom = 1.0
        self.ki_boom = 0.0
        self.kd_boom = 0.1

        self.int_error_bucket = 0
        self.int_error_arm = 0
        self.int_error_boom = 0

    def parse_csv(self):
        df = pd.read_csv(self.csv_path)
        boom_vels = df['swing_to_boom_vel']
        boom_vels += 0.4*np.sign(boom_vels[len(boom_vels)//2])
        arm_vels = df['boom_to_arm_vel']
        arm_vels += 0.4*np.sign(arm_vels[len(boom_vels)//2])
        bucket_vels = df['arm_to_bucket_vel']
        bucket_vels += 0.4*np.sign(bucket_vels[len(boom_vels)//2])
        t = df['time_from_start_s']
        return [t,boom_vels,arm_vels,bucket_vels]
        
    def bucket_cmd(self,excav_type='green',x=0.0):
        if excav_type == 'white':
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
        else:
            if x < -0.9:
                return -0.73289284
            elif x < -0.8:
                return -0.7272489
            elif x < -0.7:
                return -0.7269514514
            elif x < -0.6:
                return -0.638291842088111
            elif x < -0.5:
                return -0.328184874623581
            elif x < -0.45:
                return -0.104481405978331
            elif -0.45 <= x <= 0.45:
                return x * 0.005
            elif x < 0.5:
                return 0.058998878447
            elif x < 0.6:
                return 0.16739111
            elif x < 0.7:
                return 0.473202
            elif x < 0.8:
                return 0.97915879
            elif x < 0.9:
                return 1.020914
            else:
                return 1.036916476


    def arm_cmd(self,excav_type='green',x=0.0):
        if excav_type == 'white':
            if x < -0.9: return -0.7297529607089939
            elif x < -0.8: return -0.7417820435603295
            elif x < -0.7: return -0.7455697543943114
            elif x < -0.6: return -0.6743000194405915
            elif x < -0.5: return -0.2602819510725163
            elif x < -0.45: return -0.07527410444846894
            elif x >= -0.45 and x <= 0.45: return 0.0
            elif x < 0.45: return -0.028214468531235375
            elif x < 0.5: return 0.023708225612123934
            elif x < 0.6: return 0.09856746947233368
            elif x < 0.7: return 0.38919978426931034
            elif x < 0.8: return 0.7416978037933635
            elif x < 0.9: return 0.7555933051518472
            elif x < 1.0: return 0.752780260055926
            else: return 0.744536923787804
        else:
            if x < -0.9:
                return -0.7548945735
            elif x < -0.8:
                return -0.7438634824
            elif x < -0.7:
                return -0.7477281
            elif x < -0.6:
                return -0.704628205
            elif x < -0.5:
                return -0.3006559276
            elif x < -0.45:
                return -0.08843463
            elif -0.45 <= x <= 0.45:
                return x * 0.005
            elif x < 0.5:
                return -0.038891
            elif x < 0.6:
                return 0.120695427
            elif x < 0.7:
                return 0.39692405
            elif x < 0.8:
                return 0.68158669023
            elif x < 0.9:
                return 0.6900818
            elif x < 1.0:
                return 0.689677765
            else:
                return 0.68544316

    def boom_cmd(self,excav_type='green',x=0.0):
        if excav_type == 'white':
            if x < -0.9: return 0.4463869246347824
            elif x < -0.8: return 0.4503544485398761
            elif x < -0.7: return 0.43847760993112805
            elif x < -0.6: return 0.3985477917802957
            elif x < -0.5: return 0.22969102031072908
            elif x < -0.45: return 0.10025171220496362
            elif x >= -0.45 and x <= 0.45: return 0.0
            elif x < 0.45: return 0.04753058433724343
            elif x < 0.5: return -0.06316787902224205
            elif x < 0.6: return -0.13641786070618744
            elif x < 0.7: return -0.2727715499858695
            elif x < 0.8: return -0.45252302846893266
            elif x < 0.9: return -0.5158417589198473
            elif x < 1.0: return -0.5363396786385148
            else: return -0.5367303013977662
        else:
            if x < -0.9:
                return 0.42449316323
            elif x < -0.8:
                return 0.41108322
            elif x < -0.7:
                return 0.4267039109
            elif x < -0.6:
                return 0.38538409
            elif x < -0.5:
                return 0.233282959
            elif x < -0.45:
                return 0.08755922
            elif -0.45 <= x <= 0.45:
                return x * 0.005
            elif x < 0.5:
                return 0.038531499
            elif x < 0.6:
                return -0.09277926
            elif x < 0.7:
                return -0.1690113387
            elif x < 0.8:
                return -0.291168518879239
            elif x < 0.9:
                return -0.45902159549
            elif x < 1.0:
                return -0.5142994435
            else:
                return -0.517271958

    def _pid_update(self, error, prev_error, int_error, kp, ki, kd, dt):
        # integrate error
        int_error += error * dt

        # derivative term
        deriv = (error - prev_error) / dt if dt > 0 else 0.0

        # PID output
        output = kp * error + ki * int_error + kd * deriv
        return output, int_error

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
            msg = DeltaCan()
            msg.header.stamp = self.get_clock().now().to_msg()
            goals = self.curr_goal.tolist()
            msg.mboomcmd = float(goals[0])
            msg.marmcmd = float(goals[1])
            msg.mbucketcmd = float(goals[2])
            self.goalstate_publisher.publish(msg)

        if self.current_state is None:
            return

        dc_msg = DeltaCan()
        newboom_speed = 0.0
        newarm_speed = 0.0
        newbucket_speed = 0.0

        self.curr_error = self.curr_goal - self.current_state

        if np.any(abs(self.curr_error) > 0.05):
            if abs(self.curr_error[2]) > 0.05:
                bucket_speed = self.curr_goal[2]
                if type(self.bucket_cs) == CubicSpline:
                    newbucket_speed = self.optimize_cmd(bucket_speed)
                else:
                    newbucket_speed = self.bucket_cs(bucket_speed)

                pid_correction, self.int_error_bucket = self._pid_update(
                    self.curr_error[2],
                    self.prev_error_bucket,
                    self.int_error_bucket,
                    self.kp_bucket, self.ki_bucket, self.kd_bucket,
                    self.dt
                )
                newbucket_speed = max(min(newbucket_speed + pid_correction, 1.0), -1.0)
                
            else:
                newbucket_speed = self.prev_bucket_speed

            if abs(self.curr_error[1]) > 0.05:
                arm_speed = self.curr_goal[1]
                if type(self.arm_cs) == CubicSpline:
                    newarm_speed = self.optimize_cmd(arm_speed)
                else:
                    newarm_speed = self.arm_cs(arm_speed)

                pid_correction, self.int_error_arm = self._pid_update(
                    self.curr_error[1],
                    self.prev_error_arm,
                    self.int_error_arm,
                    self.kp_arm, self.ki_arm, self.kd_arm,
                    self.dt
                )

                newarm_speed = max(min(newarm_speed + pid_correction, 1.0), -1.0)
            else:
                newarm_speed = self.prev_arm_speed

            if abs(self.curr_error[0]) > 0.05:
                boom_sped = self.curr_goal[0]
                if type(self.boom_cs) == CubicSpline:
                    newboom_speed = self.optimize_cmd(boom_sped)
                else:
                    newboom_speed = self.boom_cs(boom_sped)

                pid_correction, self.int_error_boom = self._pid_update(
                    self.curr_error[0],
                    self.prev_error_boom,
                    self.int_error_boom,
                    self.kp_boom, self.ki_boom, self.kd_boom,
                    self.dt
                )

                newboom_speed = max(min(newboom_speed, 1.0), -1.0)

        else:
            newbucket_speed = self.prev_bucket_speed
            newarm_speed = self.prev_arm_speed
            newboom_speed = self.prev_boom_speed

        dc_msg.mbucketcmd = float(newbucket_speed)
        dc_msg.marmcmd = float(newarm_speed)
        dc_msg.mboomcmd = float(newboom_speed)
        self.prev_bucket_speed = float(newbucket_speed)
        self.prev_arm_speed = float(newarm_speed)
        self.prev_boom_speed = float(newboom_speed)

        self.prev_error_bucket = self.curr_error[2]
        self.prev_error_arm = self.curr_error[1]
        self.prev_error_boom = self.curr_error[0]

        self.publisher_.publish(dc_msg)

        

    def joint_callback(self, msg: JointState):
        if self.curr_goal is None:
            return
        curr_boom_vel = msg.velocity[self.boom_idx]
        curr_arm_vel = msg.velocity[self.arm_idx]
        curr_bucket_vel = msg.velocity[self.bucket_idx]


        self.current_state = np.array([curr_boom_vel, curr_arm_vel, curr_bucket_vel])



    def goal_callback(self):
        if self.signal_idx >= len(self.signal_arm):
            self.get_logger().info("Completed all signals, resetting index to 0")
            self.signal_idx = 0
        self.curr_goal = np.array([
            self.signal_boom[self.signal_idx],
            self.signal_arm[self.signal_idx],
            self.signal_bucket[self.signal_idx]
        ])
        self.signal_idx += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = VelocityController()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()