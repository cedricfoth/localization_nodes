#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from hippo_msgs.msg import RangeMeasurement, RangeMeasurementArray
from sensor_msgs.msg import FluidPressure
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64

from extended_kalman_filter import ExtendedKalmanFilter
from get_h_H import get_h_H
from get_f_F import get_f_F

class PositionEstimator(Node):
    def __init__(self):
        super().__init__(node_name='position_estimator')

        self.init_params()

        self.create_pub_sub_timer()

        self.add_on_set_parameters_callback(callback=self.on_params_changed)

        self.prev_t = self.get_clock().now().nanoseconds * 1e-9 - 1
        self.dt()



    def init_params(self):
        self.declare_parameters(namespace='',
                                parameters=[('x0', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                            ('initial_state_covariance', rclpy.Parameter.Type.DOUBLE),
                                            ('process_noise_position_stddev', rclpy.Parameter.Type.DOUBLE),
                                            ('range_noise_stddev',rclpy.Parameter.Type.DOUBLE),
                                            ('r_tag0_K', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                            ('r_tag1_K', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                            ('r_tag2_K', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                            ('r_tag3_K', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                            ('r_Cam_R', rclpy.Parameter.Type.DOUBLE_ARRAY),
                                            ('pressure_noise_stddev', rclpy.Parameter.Type.DOUBLE),
                                            ('atmospheric_pressure', rclpy.Parameter.Type.INTEGER), # Pa
                                            ('rho', rclpy.Parameter.Type.INTEGER), # kg/m³
                                            ('g', rclpy.Parameter.Type.DOUBLE), # N/kg
                                            ('r_pressure_R', rclpy.Parameter.Type.DOUBLE_ARRAY), # m
                                            ('x_limits', rclpy.Parameter.Type.DOUBLE_ARRAY)
                                             ])
        


        x0 = np.array(self.get_parameter('x0').get_parameter_value().double_array_value)
        self.get_logger().info(f'x0 = {x0}')
        
        P0 = self.get_parameter('initial_state_covariance').get_parameter_value().double_value * np.eye(len(x0))
        self.get_logger().info(f'P0 = {P0}')
        
        self.process_noise_position_stddev = self.get_parameter('process_noise_position_stddev').get_parameter_value().double_value
        Q = (self.process_noise_position_stddev**2) * np.eye(len(x0))
        self.get_logger().info(f'Q = {Q}')
        
        f, F = get_f_F()

        self.range_noise_stddev = self.get_parameter('range_noise_stddev').get_parameter_value().double_value
        self.get_logger().info(f'range_noise_stddev = {self.range_noise_stddev}')
        
        self.r_tag_K = np.array([self.get_parameter('r_tag0_K').get_parameter_value().double_array_value, 
                                 self.get_parameter('r_tag1_K').get_parameter_value().double_array_value, 
                                 self.get_parameter('r_tag2_K').get_parameter_value().double_array_value, 
                                 self.get_parameter('r_tag3_K').get_parameter_value().double_array_value])
        
        self.get_logger().info(f'r_tag_K = {self.r_tag_K}')

        self.r_Cam_R = np.array(self.get_parameter('r_Cam_R').get_parameter_value().double_array_value)
        self.get_logger().info(f'r_Cam_R = {self.r_Cam_R}')
        
        self.r_Cam_K = np.zeros(3)

        self.atmospheric_pressure = self.get_parameter('atmospheric_pressure').get_parameter_value().integer_value
        self.get_logger().info(f'atmospheric_pressure = {self.atmospheric_pressure}')
        
        self.rho = self.get_parameter('rho').get_parameter_value().integer_value
        self.get_logger().info(f'rho = {self.rho}')

        self.g = self.get_parameter('g').get_parameter_value().double_value
        self.get_logger().info(f'g = {self.g}')
        
        self.r_pressure_R = np.array(self.get_parameter('r_pressure_R').get_parameter_value().double_array_value)
        self.get_logger().info(f'r_pressure_R = {self.r_pressure_R}')

        self.r_pressure_K = np.zeros(3)

        self.x_limits = np.array(self.get_parameter('x_limits').get_parameter_value().double_array_value)

        self.EKF = ExtendedKalmanFilter(x0, P0, Q, f, F, x_limits = self.x_limits)

    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            self.get_logger().info(f'Try to set [{param.name}] = {param.value}')
            if param.name == 'process_noise_position_stddev':
                self.process_noise_position_stddev = param.value
                self.Q = (self.process_noise_position_stddev**2) * np.eye(len(self.EKF.get_x()))
                self.EKF.change_params(self.Q)
            elif param.name == 'range_noise_stddev':
                self.range_noise_stddev = param.value 
            elif param.name == 'r_tag0_K':
                self.r_tag_K[0] = param.value
            elif param.name == 'r_tag1_K':
                self.r_tag_K[1] = param.value
            elif param.name == 'r_tag2_K':
                self.r_tag_K[2] = param.value
            elif param.name == 'r_tag3_K':
                self.r_tag_K[3] = param.value
            elif param.name == 'r_Cam_R':
                self.r_Cam_R = np.array(param.value)
            elif param.name == 'pressure_noise_stddev':
                self.R[4][4] = param.value
            elif param.name == 'atmospheric_pressure':
                self.atmospheric_pressure = param.value
            elif param.name == 'pressure_sensor_z_offset':
                self.pressure_sensor_z_offset = param.value
            elif param.name == 'rho':
                self.rho = param.value
            elif param.name == 'g':
                self.g = param.value
            elif param.name == 'r_pressure_R':
                self.r_pressure_R = np.array(param.value)
            elif param.name == 'x_limits':
                self.x_limits = np.array(param.value)
            else:
                continue
        return SetParametersResult(successful=True, reason='Parameter set')

    def create_pub_sub_timer(self):
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.position_pub = self.create_publisher(
            msg_type=PoseStamped,
            topic='position_estimate',
            qos_profile=1)

        self.ranges_sub = self.create_subscription(
            msg_type=RangeMeasurementArray,
            topic='ranges',
            callback=self.on_ranges,
            qos_profile=qos)
        
        self.vision_pose_sub = self.create_subscription(
            msg_type=PoseWithCovarianceStamped,
            topic='vision_pose_cov',
            callback=self.on_vision_pose,
            qos_profile=qos)
        
        self.pressure_sub = self.create_subscription(
            msg_type=FluidPressure,
            topic='pressure',
            callback=self.on_pressure,
            qos_profile=qos)
        
        self.range_noise_stddev_pub = self.create_publisher(
            msg_type=Float64,
            topic='range_noise_stddev',
            qos_profile=1)
        
        self.process_noise_position_stddev_pub = self.create_publisher(
            msg_type=Float64,
            topic='process_noise_position_stddev',
            qos_profile=1
        )
        
        # do prediction step with 50 Hz
        self.process_update_timer = self.create_timer(
            1.0 / 50, self.on_prediction_step_timer)
        
    def on_ranges(self, msg: RangeMeasurementArray):
        n = len(msg._measurements)

        if n < 3:
            return
        
        z = np.full(4, np.nan)
        R = np.zeros((4, 4))

        measurement: RangeMeasurement
        for i, measurement in enumerate(msg.measurements):
            id = measurement.id
            d = measurement.range
            self.get_logger().info(f'The {i}. element contains the measurement of tag {id} with the distance of: {d}m')
            z[id] = d
            R[id][id] = self.range_noise_stddev

        self.EKF.predict(self.dt())

        h, H = get_h_H(x = self.EKF.get_x(), r_tag_K = self.r_tag_K, r_Cam_K = self.r_Cam_K, r_pressure_K = self.r_pressure_K, rho = self.rho, g = self.g, atmospheric_pressure = self.atmospheric_pressure)
        self.get_logger().info(f'z = {z}')
        x = self.EKF.update(z, h, H, R)
        self.get_logger().info(f'x[:-3] = {x}')

        self.publish_position_msg(self.EKF.get_x(), self.now())

    def on_vision_pose(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation

        (alpha, beta, gamma) = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        R_z = np.array([[np.cos(gamma), -np.sin(gamma), 0], 
                        [np.sin(gamma),  np.cos(gamma), 0], 
                        [       0,             0,       1]])

        self.r_Cam_K = R_z @ self.r_Cam_R

        self.get_logger().info(f'r_Cam_K = {self.r_Cam_K}')

        self.r_pressure_K = R_z @ self.r_pressure_R

    def on_pressure(self, msg: FluidPressure):
        pass

    def on_prediction_step_timer(self):
        self.EKF.predict(self.dt())
        self.publish_position_msg(self.EKF.get_x(), self.now())
        self.publish_stddevs()

    def publish_position_msg(self, x: np.ndarray, now: rclpy.time.Time):
        msg = PoseStamped()
        
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = x[0]
        msg.pose.position.y = x[1]
        msg.pose.position.z = x[2]

        self.position_pub.publish(msg)

    def publish_stddevs(self):
        range_msg = Float64()
        process_msg = Float64()

        range_msg.data = self.range_noise_stddev
        process_msg.data = self.process_noise_position_stddev

        self.range_noise_stddev_pub.publish(range_msg)
        self.process_noise_position_stddev_pub.publish(process_msg)


    def dt(self) -> float:
        t = self.get_clock().now().nanoseconds * 1e-9
        dt = t - self.prev_t
        self.prev_t = t
        return dt 
    
    def now(self):
        return self.get_clock().now()
        

def main():
    rclpy.init()
    node = PositionEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()