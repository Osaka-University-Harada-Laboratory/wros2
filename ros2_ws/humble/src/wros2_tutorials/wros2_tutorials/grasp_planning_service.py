import os
import math
import numpy as np
from transforms3d.quaternions import mat2quat

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import modeling.geometric_model as gm
import modeling.collision_model as cm
import visualization.panda.world as wd
import grasping.planning.antipodal as gpa
import robot_sim.end_effectors.gripper.robotiqhe.robotiqhe as he


g_node = None
pose_dict = {}
base = None
gripper = None
object_tube = None
gripper_stl_path = \
    'wrs/robot_sim/end_effectors/gripper/robotiqhe/meshes/base_cvt.stl'
finger1_stl_path = \
    'wrs/robot_sim/end_effectors/gripper/robotiqhe/meshes/finger1_cvt.stl'
finger2_stl_path = \
    'wrs/robot_sim/end_effectors/gripper/robotiqhe/meshes/finger2_cvt.stl'
markers = None


class GraspPlanner(Node):

    def __init__(self):
        super().__init__('grasp_tf_publisher')
        self.br = StaticTransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.update_tfs)

    def update_tfs(self):
        global pose_dict

        if pose_dict is not {}:
            for name, data in pose_dict.items():
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = data['parent']
                t.child_frame_id = name
                t.transform.translation.x = data['pose'].position.x
                t.transform.translation.y = data['pose'].position.y
                t.transform.translation.z = data['pose'].position.z
                t.transform.rotation.x = data['pose'].orientation.x
                t.transform.rotation.y = data['pose'].orientation.y
                t.transform.rotation.z = data['pose'].orientation.z
                t.transform.rotation.w = data['pose'].orientation.w

                self.br.sendTransform(t)


def gen_marker(frame_name, name, id_int, pose, stl_path):
    """ Generates a marker.

        Attributes:
            frame_name (str): Frame name
            name (str): Unique marker name
            id_int (int): Unique id number
            pose (geometry_msgs/Pose): Pose of the marker
            stl_path (str):
    """

    marker = Marker()
    marker.header.frame_id = frame_name
    t = Clock().now()
    marker.header.stamp = t.to_msg()
    marker.ns = name
    marker.id = id_int
    marker.type = marker.MESH_RESOURCE
    marker.action = marker.ADD
    marker.pose = pose
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.mesh_resource = 'package://wros2_tutorials/'+stl_path
    marker.mesh_use_embedded_materials = True

    return marker


def plan_grasps(req, res):
    """ Plans grasps. """
    global g_node
    global pose_dict
    global gripper
    global object_tube
    global base
    global markers

    grasp_info_list = gpa.plan_grasps(
        gripper,
        object_tube,
        angle_between_contact_normals=math.radians(90),
        openning_direction='loc_x',
        max_samples=4,
        min_dist_between_sampled_contact_points=.016,
        contact_offset=.016)

    for i, grasp_info in enumerate(grasp_info_list):
        jaw_width, jaw_center_pos, jaw_center_rotmat, hnd_pos, hnd_rotmat = \
            grasp_info
        gripper.grip_at_with_jcpose(
            jaw_center_pos, jaw_center_rotmat, jaw_width)
        gripper.gen_meshmodel().attach_to(base)

        pose_b = Pose()
        pose_b.position.x = hnd_pos[0]
        pose_b.position.y = hnd_pos[1]
        pose_b.position.z = hnd_pos[2]
        q = mat2quat(hnd_rotmat)
        pose_b.orientation.x = q[1]
        pose_b.orientation.y = q[2]
        pose_b.orientation.z = q[3]
        pose_b.orientation.w = q[0]
        parent_frame = 'object'
        markers.markers.append(
            gen_marker(
                parent_frame,
                'hande_b_'+str(i),
                0,
                pose_b,
                gripper_stl_path))
        pose_dict['hande_b_'+str(i)] = \
            {'parent': parent_frame, 'pose': pose_b}
        g_node.update_tfs()

        parent_frame = 'hande_b_'+str(i)
        pose_f1 = Pose()
        pose_f1.position.x = -0.025
        pose_f1.position.y = 0.0
        pose_f1.position.z = 0.11
        pose_f1.orientation.x = 0.
        pose_f1.orientation.y = 0.
        pose_f1.orientation.z = 0.
        pose_f1.orientation.w = 1.
        markers.markers.append(
            gen_marker(
                parent_frame,
                'hande_f1_'+str(i),
                0,
                pose_f1,
                finger1_stl_path))
        pose_f2 = Pose()
        pose_f2.position.x = 0.025
        pose_f2.position.y = 0.0
        pose_f2.position.z = 0.11
        pose_f2.orientation.x = 0.
        pose_f2.orientation.y = 0.
        pose_f2.orientation.z = 0.
        pose_f2.orientation.w = 1.
        markers.markers.append(
            gen_marker(
                parent_frame,
                'hande_f2_'+str(i),
                0,
                pose_f2,
                finger2_stl_path))
        pose_dict['hande_f1_'+str(i)] = \
            {'parent': parent_frame, 'pose': pose_f1}
        pose_dict['hande_f2_'+str(i)] = \
            {'parent': parent_frame, 'pose': pose_f2}

    return res


def main(args=None):
    rclpy.init(args=args)
    global g_node
    global gripper
    global object_tube
    global base
    global markers

    g_node = GraspPlanner()
    pkg_path = '/ros2_ws/src'
    base = wd.World(cam_pos=[1, 1, 1], lookat_pos=[0, 0, 0])
    base.taskMgr.step()
    gripper = he.RobotiqHE()
    gm.gen_frame().attach_to(base)
    base.taskMgr.step()
    object_stl_path = os.path.join(
        pkg_path, 'wrs/0000_examples/objects/tubebig.stl')
    object_tube = cm.CollisionModel(object_stl_path)
    object_tube.set_rgba([.9, .75, .35, .3])
    object_tube.attach_to(base)
    base.taskMgr.step()

    markers = MarkerArray()
    pose = Pose()
    pose.position.x = 0.
    pose.position.y = 0.
    pose.position.z = 0.
    pose.orientation.x = 0.
    pose.orientation.y = 0.
    pose.orientation.z = 0.
    pose.orientation.w = 1.
    markers.markers.append(
        gen_marker(
            'base_link',
            'object',
            0,
            pose,
            'wrs/0000_examples/objects/tubebig.stl'))

    planning_service = g_node.create_service(
        Empty, 'plan_grasp', plan_grasps)
    pub = g_node.create_publisher(
        MarkerArray, 'grasp_pub', 1)

    while rclpy.ok():
        pub.publish(markers)
        base.taskMgr.step()
        rclpy.spin_once(g_node)

    g_node.destroy_service(planning_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
