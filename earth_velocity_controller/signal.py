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
import random

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

        # self.goalstate_subscriber = self.create_subscription(Float32MultiArray, '/goal_state', self.goal_callback,10)

        self.goalstate_timer = self.create_timer(self.dt, self.goalpub_callback)
        self.goalstate_publisher = self.create_publisher(DeltaCan, '/frequencies',10)
        self.amplitude_publisher = self.create_publisher(DeltaCan, '/amplitudes',10)

        self.curr_goal = None

        self.current_state = None


        self.prev_bucket_speed = 0
        self.prev_arm_speed = 0
        self.prev_boom_speed = 0

        self.curr_error = 0

        self.boom_idx = 1
        self.arm_idx = 2
        self.bucket_idx = 3

        self.bucket_min = 0.05
        self.arm_min = 0.05
        self.boom_min = 0.05



        self.signal_idx = 0
        self.freqs = [1/20,1/15,1/10,1/5,1/2]
        self.amps = np.arange(0.4,1.1,0.1)
        # add 0.0 amplitude to make it possible to stop
        self.amps = np.append(self.amps, 0.0)
        self.amps = np.append(self.amps, 0.0)
        self.amps = np.append(self.amps, 0.0)
        self.duration = 10

        self.bucket_freq = random.choice(self.freqs)
        self.arm_freq = random.choice(self.freqs)
        self.boom_freq = random.choice(self.freqs)

        self.boom_amp = random.choice(self.amps)
        self.arm_amp = random.choice(self.amps)
        self.bucket_amp = random.choice(self.amps)

        self.boom_gain = 0.0
        self.arm_gain = 1.0
        self.bucket_gain = 1.0

        self.get_logger().info(f'Initial Frequencies: Boom {self.boom_freq}, Arm {self.arm_freq}, Bucket {self.bucket_freq}')
        self.get_logger().info(f'Initial Amplitudes: Boom {self.boom_amp}, Arm {self.arm_amp}, Bucket {self.bucket_amp}')



        # _, self.signal_bucket = generate_sine(min_range=-1, max_range=1, duration=duration, control_freq=int(1/self.dt), wave_freq=self.bucket_freq, phase=np.pi, noise=0.0)
        # self.signal_t, self.signal_arm = generate_sine(min_range=-1, max_range=1, duration=duration, control_freq=int(1/self.dt), wave_freq=self.arm_freq, phase=0, noise=0.0)
        # _, self.signal_boom = generate_sine(min_range=-1, max_range=1, duration=duration, control_freq=int(1/self.dt), wave_freq=self.boom_freq, phase=np.pi/2, noise=0.0)

        _, self.signal_bucket = generate_sine(min_range=-self.bucket_amp, max_range=self.bucket_amp, duration=self.duration, control_freq=int(1/self.dt), wave_freq=self.bucket_freq, phase=np.pi, noise=0.0)
        self.signal_t, self.signal_arm = generate_sine(min_range=-self.arm_amp, max_range=self.arm_amp, duration=self.duration, control_freq=int(1/self.dt), wave_freq=self.arm_freq, phase=0, noise=0.0)
        _, self.signal_boom = generate_sine(min_range=-self.boom_amp, max_range=self.boom_amp, duration=self.duration, control_freq=int(1/self.dt), wave_freq=self.boom_freq, phase=np.pi/2, noise=0.0)


        self.signal_bucket *= self.bucket_gain
        self.signal_arm *= self.arm_gain
        self.signal_boom *= self.boom_gain


    def goalpub_callback(self):

        msg = DeltaCan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mboomcmd = float(self.boom_freq)
        msg.marmcmd = float(self.arm_freq)
        msg.mbucketcmd = float(self.bucket_freq)
        self.goalstate_publisher.publish(msg)

        amp_msg = DeltaCan()
        amp_msg.header.stamp = self.get_clock().now().to_msg()
        amp_msg.mboomcmd = float(self.boom_amp)
        amp_msg.marmcmd = float(self.arm_amp)
        amp_msg.mbucketcmd = float(self.bucket_amp)
        self.amplitude_publisher.publish(amp_msg)


        dc_msg = DeltaCan()
        newboom_speed = 0.0
        newarm_speed = 0.0
        newbucket_speed = 0.0


        newbucket_speed = self.signal_bucket[self.signal_idx]
        newarm_speed = self.signal_arm[self.signal_idx]
        newboom_speed = self.signal_boom[self.signal_idx]
        self.signal_idx += 1

        if self.signal_idx >= len(self.signal_t)-1:
            self.get_logger().info('Changing Frequencies and Amplitudes')
            self.signal_idx = 0
            self.bucket_freq = random.choice(self.freqs)
            self.arm_freq = random.choice(self.freqs)
            self.boom_freq = random.choice(self.freqs)
            self.boom_amp = random.choice(self.amps)
            self.arm_amp = random.choice(self.amps)
            self.bucket_amp = random.choice(self.amps)
            _, self.signal_bucket = generate_sine(min_range=-self.bucket_amp, max_range=self.bucket_amp, duration=self.duration, control_freq=int(1/self.dt), wave_freq=self.bucket_freq, phase=np.pi, noise=0.0)
            self.signal_t, self.signal_arm = generate_sine(min_range=-self.arm_amp, max_range=self.arm_amp, duration=self.duration, control_freq=int(1/self.dt), wave_freq=self.arm_freq, phase=0, noise=0.0)
            _, self.signal_boom = generate_sine(min_range=-self.boom_amp, max_range=self.boom_amp, duration=self.duration, control_freq=int(1/self.dt), wave_freq=self.boom_freq, phase=np.pi/2, noise=0.0)
            self.signal_bucket *= self.bucket_gain
            self.signal_arm *= self.arm_gain
            self.signal_boom *= self.boom_gain
            self.get_logger().info(f'New Frequencies: Boom {self.boom_freq}, Arm {self.arm_freq}, Bucket {self.bucket_freq}')
            self.get_logger().info(f'New Amplitudes: Boom {self.boom_amp}, Arm {self.arm_amp}, Bucket {self.bucket_amp}')
            return

        dc_msg.mbucketcmd = float(newbucket_speed)
        dc_msg.marmcmd = float(newarm_speed)
        dc_msg.mboomcmd = float(newboom_speed)
        self.prev_bucket_speed = float(newbucket_speed)
        self.prev_arm_speed = float(newarm_speed)
        self.prev_boom_speed = float(newboom_speed)
        self.publisher_.publish(dc_msg)

        

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = VelocityController()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()