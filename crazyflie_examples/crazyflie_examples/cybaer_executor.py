import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor

from std_msgs.msg import String
from crazyflie_interfaces.msg import Status, LogDataGeneric #, FullState
from geometry_msgs.msg import Pose

import numpy as np
import rowan

class CybaerTalker(Node):

    def __init__(self):
        # Calls Node.__init__('cybaer_talker')
        super().__init__('cybaer_talker')
        cfname = 'cf4tmp'
        prefix = '/' + cfname
        self.prefix = prefix
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter', 10)
        timer_period = 2.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        # self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)

class CybaerListener(Node):

    def __init__(self):
        super().__init__('cybaer_listener')
        cfname = 'cf4'
        prefix = '/' + cfname
        self.prefix = prefix
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.statusSimpleSubscriber = self.create_subscription(
            LogDataGeneric, f'{self.prefix}/status_simple', self.status_simple_topic_callback, 10)
        self.status = {}
        self.get_logger().info(f'subscribed topic: {self.prefix}/status_simple')

        self.poseSimpleSubscriber = self.create_subscription(
            LogDataGeneric, f'{self.prefix}/pose_simple', self.pose_simple_topic_callback, 10)
        self.pose = {}
        self.get_logger().info(f'subscribed topic: {self.prefix}/pose_simple')

        self.twistSimpleSubscriber = self.create_subscription(
            LogDataGeneric, f'{self.prefix}/twist_simple', self.twist_simple_topic_callback, 10)
        self.twist = {}
        self.get_logger().info(f'subscribed topic: {self.prefix}/twist_simple')


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    
    def status_simple_topic_callback(self, msg):
        """
        Call back for topic /cfXXX/status_simple.
        vars: ["pm.vbatMV","supervisor.info"]
        """
        self.status_simple = {'id': msg.header.frame_id,
                       'timestamp_sec': msg.header.stamp.sec,
                       'timestamp_nsec': msg.header.stamp.nanosec,
                       'supervisor': msg.values[1],
                       'battery': msg.values[0]}
        
        self.get_logger().info(f'status_simple_topic_callback was called {self.status_simple}')


    def pose_simple_topic_callback(self, msg):
        """
        Call back for topic /cfXXX/pose_simple.
        vars: ["stateEstimateZ.x", "stateEstimateZ.y", "stateEstimateZ.z","stateEstimate.qx","stateEstimate.qy","stateEstimate.qz","stateEstimate.qw"] 
        """

        pos = np.array([msg.values[0], msg.values[1], msg.values[2]])/1000.0 # conversion from mm to m
        quat = np.array([msg.values[3], msg.values[4], msg.values[5], msg.values[6]]) # conversion from deg to rad
        # quatZ = msg.values[3]
        lbd = rowan.to_euler(quat)
        # lbd = np.array([msg.values[3], msg.values[4], msg.values[5]])*np.pi/180.0 # conversion from deg to rad

        self.pose = {   'id': msg.header.frame_id,
                        'timestamp_sec': msg.header.stamp.sec,
                        'timestamp_nsec': msg.header.stamp.nanosec,
                        'pos': pos,
                        'quat': quat,
                        'lbd': lbd }
        
        self.get_logger().info(f'pose_simple_topic_callback was called {self.pose}')


    def twist_simple_topic_callback(self, msg):
        """
        Call back for topic /cfXXX/twist_simple.
        vars: ["stateEstimateZ.vx", "stateEstimateZ.vy", "stateEstimateZ.vz","stateEstimateZ.rateRoll","stateEstimateZ.ratePitch","stateEstimateZ.rateYaw"]
        """

        vel = np.array([msg.values[0], msg.values[1], msg.values[2]])/1000.0 # conversion from mm/ to m/s
        om = np.array([msg.values[3], msg.values[4], msg.values[5]])*np.pi/180.0 # conversion from deg/s to rad/s

        self.twist = {   'id': msg.header.frame_id,
                        'timestamp_sec': msg.header.stamp.sec,
                        'timestamp_nsec': msg.header.stamp.nanosec,
                        'vel': vel,
                        'om': om }
        
        self.get_logger().info(f'twist_simple_topic_callback was called {self.twist}')


def main(args=None):
    rclpy.init(args=args)
    try:
        talker = CybaerTalker()
        listener = CybaerListener()

        # Runs all callbacks in the main thread
        executor = SingleThreadedExecutor()
        # Add imported nodes to this executor
        executor.add_node(talker)
        executor.add_node(listener)

        # Execute callbacks for both nodes as they become ready
        executor.spin()
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)


if __name__ == '__main__':
    main()