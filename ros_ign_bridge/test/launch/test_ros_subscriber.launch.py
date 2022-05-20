# Copyright 2020 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit, OnExecutionComplete

from launch_ros.actions import Node

import launch_testing
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest

from builtin_interfaces.msg import Time
from std_msgs.msg import Bool, ColorRGBA, Empty, Float32, Float64, UInt32, Header, String
from geometry_msgs.msg import Quaternion, Vector3, Point, Pose, PoseWithCovariance, PoseStamped, Transform, TransformStamped, Twist, TwistWithCovariance, Wrench
from tf2_msgs.msg import TFMessage
from ros_ign_interfaces.msg import JointWrench, Entity, Contact, Contacts, Light, GuiCamera, StringVec, TrackVisual, VideoRecord
from sensor_msgs.msg import Image, CameraInfo, FluidPressure, Imu, LaserScan, MagneticField, PointCloud2, JointState, BatteryState
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory
from rosgraph_msgs.msg import Clock

topics_list = [
    ("bool", Bool), ("time", Time), ("color", ColorRGBA), ("empty", Empty),
    ("float", Float32), ("double", Float64), ("uint32", UInt32), ("header", Header),
    ("string", String), ("quaternion", Quaternion), ("vector3", Vector3), ("point", Point),
    ("pose", Pose), ("pose_with_covariance", PoseWithCovariance), ("pose_stamped", PoseStamped),
    ("transform", Transform), ("tf2_message", TFMessage), ("tranform_stamped", TransformStamped),
    ("twist", Twist), ("twist_with_covariance", TwistWithCovariance), ("wrench", Wrench),
    ("joint_wrench", JointWrench), ("entity", Entity), ("contact", Contact), ("contacts", Contacts),
    ("light", Light), ("gui_camera", GuiCamera), ("stringmsg_v", StringVec), ("track_visual", TrackVisual),
    ("video_record", VideoRecord), ("image", Image), ("camera_info", CameraInfo), ("fluid_pressure", FluidPressure),
    ("imu", Imu), ("laserscan", LaserScan), ("magnetic", MagneticField), ("odometry", Odometry),
    ("odometry_with_covariance", Odometry), ("pointcloud2", PointCloud2), ("joint_states", JointState), ("joint_trajectory", JointTrajectory),
    ("clock", Clock)
]

expected_topics = {
    "bool", "time", "color", "empty", "float", "double", "uint32", "header",
    "string", "quaternion", "vector3", "point", "pose", "pose_with_covariance",
    "pose_stamped", "transform", "tf2_message", "transform_stamped", "twist",
    "twist_with_covariance", "wrench", "joint_wrench", "entity", "contact",
    "contacts", "light", "gui_camera", "stringmsg_v", "track_visual", "video_record",
    "image", "camera_info", "fluid_pressure", "imu", "laserscan", "magnetic",
    "odometry", "odometry_with_covariance", "pointcloud2", "joint_states", "joint_trajectory",
    "clock"
}

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():

    publisher = Node(
        package='ros_ign_bridge',
        executable='test_ign_publisher',
        output='screen'
    )
    # process_under_test = Node(
    #     package='ros_ign_bridge',
    #     executable='test_ros_subscriber',
    #     output='screen'
    # )

    # Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
          '/time@builtin_interfaces/msg/Time@ignition.msgs.Time',
          '/bool@std_msgs/msg/Bool@ignition.msgs.Boolean',
          '/color@std_msgs/msg/ColorRGBA@ignition.msgs.Color',
          '/empty@std_msgs/msg/Empty@ignition.msgs.Empty',
          '/float@std_msgs/msg/Float32@ignition.msgs.Float',
          '/double@std_msgs/msg/Float64@ignition.msgs.Double',
          '/uint32@std_msgs/msg/UInt32@ignition.msgs.UInt32',
          '/header@std_msgs/msg/Header@ignition.msgs.Header',
          '/string@std_msgs/msg/String@ignition.msgs.StringMsg',
          '/quaternion@geometry_msgs/msg/Quaternion@ignition.msgs.Quaternion',
          '/vector3@geometry_msgs/msg/Vector3@ignition.msgs.Vector3d',
          '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
          '/point@geometry_msgs/msg/Point@ignition.msgs.Vector3d',
          '/pose@geometry_msgs/msg/Pose@ignition.msgs.Pose',
          '/pose_with_covariance@geometry_msgs/msg/PoseWithCovariance@'
          'ignition.msgs.PoseWithCovariance',
          '/pose_stamped@geometry_msgs/msg/PoseStamped@ignition.msgs.Pose',
          '/transform@geometry_msgs/msg/Transform@ignition.msgs.Pose',
          '/tf2_message@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
          '/transform_stamped@geometry_msgs/msg/TransformStamped@ignition.msgs.Pose',
          '/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist',
          '/twist_with_covariance@geometry_msgs/msg/TwistWithCovariance@'
          'ignition.msgs.TwistWithCovariance',
          '/wrench@geometry_msgs/msg/Wrench@ignition.msgs.Wrench',
          '/joint_wrench@ros_ign_interfaces/msg/JointWrench@ignition.msgs.JointWrench',
          '/entity@ros_ign_interfaces/msg/Entity@ignition.msgs.Entity',
          '/contact@ros_ign_interfaces/msg/Contact@ignition.msgs.Contact',
          '/contacts@ros_ign_interfaces/msg/Contacts@ignition.msgs.Contacts',
          '/light@ros_ign_interfaces/msg/Light@ignition.msgs.Light',
          '/gui_camera@ros_ign_interfaces/msg/GuiCamera@ignition.msgs.GUICamera',
          '/stringmsg_v@ros_ign_interfaces/msg/StringVec@ignition.msgs.StringMsg_V',
          '/track_visual@ros_ign_interfaces/msg/TrackVisual@ignition.msgs.TrackVisual',
          '/video_record@ros_ign_interfaces/msg/VideoRecord@ignition.msgs.VideoRecord',
          '/image@sensor_msgs/msg/Image@ignition.msgs.Image',
          '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
          '/fluid_pressure@sensor_msgs/msg/FluidPressure@ignition.msgs.FluidPressure',
          '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
          '/laserscan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
          '/magnetic@sensor_msgs/msg/MagneticField@ignition.msgs.Magnetometer',
          # '/actuators@mav_msgs/msg/Actuators@ignition.msgs.Actuators',
          '/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
          '/odometry_with_covariance@nav_msgs/msg/Odometry@'
          'ignition.msgs.OdometryWithCovariance',
          '/pointcloud2@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
          '/joint_states@sensor_msgs/msg/JointState@ignition.msgs.Model',
          '/battery_state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState',
          '/joint_trajectory@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory'
        ],
        output='screen'
    )
    return LaunchDescription([
        bridge,
        publisher,
        launch_testing.actions.ReadyToTest(),
    ]), locals()

class ROSSubscriberTest(unittest.TestCase):
    def test_topics(self):
        wait_for_node_object = WaitForTopics(topics_list, timeout=30.0)
        assert wait_for_node_object.wait()
        assert wait_for_node_object.topics_received() == expected_topics
        assert wait_for_node_object.topics_not_received() == set()
        wait_for_node_object.shutdown()
# class ROSSubscriberTest(unittest.TestCase):

#     def test_termination(self, process_under_test, proc_info):
#         proc_info.assertWaitForShutdown(process=process_under_test, timeout=10)


# @launch_testing.post_shutdown_test()
# class ROSSubscriberTestAfterShutdown(unittest.TestCase):

#     def test_exit_code(self, process_under_test, proc_info):
#         launch_testing.asserts.assertExitCodes(
#             proc_info,
#             [launch_testing.asserts.EXIT_OK],
#             process_under_test
#         )
