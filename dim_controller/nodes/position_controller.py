#!/usr/bin/env python3

import math
from typing import Callable, List
import numpy as np

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from hippo_msgs.msg import ActuatorSetpoint, Float64Stamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rcl_interfaces.msg import SetParametersResult



class positionController(Node):

    def __init__(self):
        super().__init__(node_name='position_controller')
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        # Setpoint Definition
        self.setpoint = [0.0,0.0,-0.5]

        self.init_params()

        #Controller Defintion -> in x,y,z
        self.Kp = [0.0,0.0,0.0]
        self.Kd= [0.0,0.0,0.0]
        self.Ki = [0.0,0.0,0.0]
        
        # Derivative part
        self.lastError = np.array([0.0,0.0,0.0])
        self.last_t = self.get_clock().now().nanoseconds *1e-09

        # Integral part
        self.errorIntegral =np.array([0.0,0.0,0.0])
        
        #Subscriber
        self.position_sub = self.create_subscription(msg_type=PoseStamped,
                                                     topic='position_estimate',
                                                     callback=self.on_pose,
                                                     qos_profile=1)


        #Publisher Necessary
        self.actuator_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                  topic='thrust_setpoint',
                                                  qos_profile=1)

        #Publisher for Debugging
        self.debugger_Kp_value = self.create_publisher(msg_type=Float32MultiArray,
                                                 topic='kp_value',
                                                 qos_profile=1)        

        self.debugger_Kd_value = self.create_publisher(msg_type=Float32MultiArray,
                                                 topic='kd_value',
                                                 qos_profile=1)        
        self.debugger_Ki_value = self.create_publisher(msg_type=Float32MultiArray,
                                                 topic='ki_value',
                                                 qos_profile=1)        
        self.debugger_error = self.create_publisher(msg_type=Float32MultiArray,
                                                 topic='error',
                                                 qos_profile=1)        

        self.debugger_dError = self.create_publisher(msg_type=Float32MultiArray,
                                                 topic='dError',
                                                 qos_profile=1)        
        self.debugger_errorIntegral = self.create_publisher(msg_type=Float32MultiArray,
                                                 topic='errorIntegral',
                                                 qos_profile=1)        


    def init_params(self):
        self.declare_parameters(namespace='',
                                parameters=[
                                    ('setpoint',rclpy.Parameter.Type.DOUBLE_ARRAY),
                                    ('proportionalGain',rclpy.Parameter.Type.DOUBLE_ARRAY),
                                    ('integralGain',rclpy.Parameter.Type.DOUBLE_ARRAY),
                                    ('derivateGain',rclpy.Parameter.Type.DOUBLE_ARRAY)
                                ])
        
        self.add_on_set_parameters_callback(self.on_params_changed)
    

    def on_params_changed(self, params):
        self.setpoint = self.get_parameter('setpoint').get_parameter_value().double_array_value
        self.Kp = self.get_parameter('proportionalGain').get_parameter_value().double_array_value
        self.Ki = self.get_parameter('integralGain').get_parameter_value().double_array_value
        self.Kd = self.get_parameter('derivateGain').get_parameter_value().double_array_value
        return SetParametersResult(successful=True)

    def on_pose(self, posemsg: PoseStamped())  -> None:
        
        msg = posemsg
        position = np.array([0.0,0.0,0.0])

        position[0] = msg.pose.position.x
        position[1] = msg.pose.position.y
        position[2] = msg.pose.position.z



        # Current Time

        currentTime = self.get_clock().now().nanoseconds*1e-9
        dt = currentTime -self.last_t
        self.last_t = currentTime
        # Error Defintion
        error = self.setpoint-position

        # Delta Error 
        dError = self.lastError - error

        self.lastError = error
        # Updating errorIntegral
        if (error[0] < 0.30 and error[0] > -0.30 and self.Kp[0]>0):
            self.errorIntegral[0] = self.errorIntegral[0] + error[0]*dt
        else:
            self.errorIntegral[0] = 0.0
        if (error[1] < 0.30 and error[1] > -0.30 and self.Kp[1]>0):
            self.errorIntegral[1] = self.errorIntegral[1] + error[1]*dt
        else:
            self.errorIntegral[1] = 0.0
        if (error[2] < 0.30 and error[2] > -0.30 and self.Kp[2]>0):
            self.errorIntegral[2] = self.errorIntegral[2] + error[2]*dt
        else:
            self.errorIntegral[2] = 0.0
        actuatorCommands = self.Kp * error +  self.Kd * (dError/dt) + self.Ki *self.errorIntegral

        
        # actuatorCommandsTransformed = S@actuatorCommands 


        self.publish_actuatorCommands(actuatorCommands = actuatorCommands, timestamp = self.get_clock().now())

        self.publish_debugging(error,self.errorIntegral,dError)
        



    def publish_actuatorCommands(self, actuatorCommands, timestamp: rclpy.time.Time) -> None:

        msg = ActuatorSetpoint()

        msg.x = actuatorCommands[1]
        msg.y = - actuatorCommands[0]
        msg.z = actuatorCommands[2]

        msg.header.stamp = timestamp.to_msg()

        self.actuator_pub.publish(msg)

    def publish_debugging(self, error, errorIntegral, dError) -> None:
        # Publisch Gain
        msgProportional = Float32MultiArray()
        msgProportional.data = [0.0]*3
        msgDerivative = Float32MultiArray()
        msgDerivative.data = [0.0]*3
        msgIntegral = Float32MultiArray()
        msgIntegral.data = [0.0]*3

        for index, value in enumerate(self.Kp):
            msgProportional.data[index] = value

        for index, value in enumerate(self.Kd):
            msgDerivative.data[index] = value

        for index, value in enumerate(self.Ki):
            msgIntegral.data[index] = value

        self.debugger_Kp_value.publish(msgProportional)
        self.debugger_Kd_value.publish(msgDerivative)
        self.debugger_Ki_value.publish(msgIntegral)

        # Publish Error

        msgError = Float32MultiArray()
        msgError.data = [0.0]*3
        msgdError = Float32MultiArray()
        msgdError.data = [0.0]*3
        msgerrorIntegral = Float32MultiArray()
        msgerrorIntegral.data = [0.0]*3

        for index, value in enumerate(error):
            msgError.data[index] = value

        for index, value in enumerate(dError):
            msgdError.data[index] = value

        for index, value in enumerate(errorIntegral):
            msgerrorIntegral.data[index] = value

        self.debugger_error.publish(msgError)
        self.debugger_dError.publish(msgdError)
        self.debugger_errorIntegral.publish(msgerrorIntegral)
        


def main():
    rclpy.init()
    node = positionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()





