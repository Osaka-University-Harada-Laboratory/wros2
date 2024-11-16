
import numpy as np
from panda3d.core import Vec3, Mat4
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


class GraspPlanner(Node):

    def __init__(self):
        super().__init__('grasp_planning_server')

        self.declare_parameter('gripper_name', 'robotiqhe')
        gripper_name = self.get_parameter(
            'gripper_name').get_parameter_value().string_value
        if gripper_name in ['robotiqhe', 'robotiq85', 'robotiq140']:
            self.base = wd.World(cam_pos=[1, 1, 1], lookat_pos=[0, 0, 0])

            self.declare_parameter(
                'antipodal_grasp.angle_between_contact_normals', 90)
            self.angle_between_contact_normals = self.get_parameter(
                'antipodal_grasp.angle_between_contact_normals').\
                    get_parameter_value().integer_value
            self.declare_parameter(
                'antipodal_grasp.openning_direction', 'loc_x')
            self.openning_direction = self.get_parameter(
                'antipodal_grasp.openning_direction').\
                    get_parameter_value().string_value
            self.declare_parameter(
                'antipodal_grasp.max_samples', 4)
            self.max_samples = self.get_parameter(
                'antipodal_grasp.max_samples').\
                    get_parameter_value().integer_value
            self.declare_parameter(
                'antipodal_grasp.min_dist_between_sampled_contact_points', .016)
            self.min_dist_between_sampled_contact_points = self.get_parameter(
                'antipodal_grasp.min_dist_between_sampled_contact_points').\
                    get_parameter_value().double_value
            self.declare_parameter(
                'antipodal_grasp.contact_offset', .016)
            self.contact_offset = self.get_parameter(
                'antipodal_grasp.contact_offset').\
                    get_parameter_value().double_value
        else:
            self.get_logger().error(
                "The specified gripper is not implemented.",
                throttle_duration_sec=1)
        self.base.taskMgr.step()

        if gripper_name == 'robotiqhe':
            import robot_sim.end_effectors.gripper.robotiqhe.robotiqhe as gr
            self.gripper = gr.RobotiqHE()
            self.body_mesh_path = self.gripper.lft.lnks[0]['mesh_file']
            self.fingers_dict = {
                'gripper.lft.lnks.1': self.gripper.lft.lnks[1],
                'gripper.rgt.lnks.1': self.gripper.rgt.lnks[1]}
        elif gripper_name == 'robotiq85':
            import robot_sim.end_effectors.gripper.robotiq85.robotiq85 as gr
            self.gripper = gr.Robotiq85()
            self.body_mesh_path = self.gripper.lft_outer.lnks[0]['mesh_file']
            self.fingers_dict = {
                'gripper.lft_outer.lnks.1': self.gripper.lft_outer.lnks[1],
                'gripper.rgt_outer.lnks.1': self.gripper.rgt_outer.lnks[1],
                'gripper.lft_outer.lnks.2': self.gripper.lft_outer.lnks[2],
                'gripper.rgt_outer.lnks.2': self.gripper.rgt_outer.lnks[2],
                'gripper.lft_outer.lnks.3': self.gripper.lft_outer.lnks[3],
                'gripper.rgt_outer.lnks.3': self.gripper.rgt_outer.lnks[3],
                'gripper.lft_outer.lnks.4': self.gripper.lft_outer.lnks[4],
                'gripper.rgt_outer.lnks.4': self.gripper.rgt_outer.lnks[4],
                'gripper.lft_inner.lnks.1': self.gripper.lft_inner.lnks[1],
                'gripper.rgt_inner.lnks.1': self.gripper.rgt_inner.lnks[1]}
        elif gripper_name == 'robotiq140':
            import robot_sim.end_effectors.gripper.robotiq140.robotiq140 as gr
            self.gripper = gr.Robotiq140()
            self.body_mesh_path = self.gripper.lft_outer.lnks[0]['mesh_file']
            self.fingers_dict = {
                'gripper.lft_outer.lnks.1': self.gripper.lft_outer.lnks[1],
                'gripper.rgt_outer.lnks.1': self.gripper.rgt_outer.lnks[1],
                'gripper.lft_outer.lnks.2': self.gripper.lft_outer.lnks[2],
                'gripper.rgt_outer.lnks.2': self.gripper.rgt_outer.lnks[2],
                'gripper.lft_outer.lnks.3': self.gripper.lft_outer.lnks[3],
                'gripper.rgt_outer.lnks.3': self.gripper.rgt_outer.lnks[3],
                'gripper.lft_outer.lnks.4': self.gripper.lft_outer.lnks[4],
                'gripper.rgt_outer.lnks.4': self.gripper.rgt_outer.lnks[4],
                'gripper.lft_inner.lnks.1': self.gripper.lft_inner.lnks[1],
                'gripper.rgt_inner.lnks.1': self.gripper.rgt_inner.lnks[1]}
        else:
            self.get_logger().error(
                "The specified gripper is not implemented.",
                throttle_duration_sec=1)
        gm.gen_frame().attach_to(self.base)
        self.base.taskMgr.step()

        self.declare_parameter(
            'object_mesh_path',
            '/ros2_ws/src/wrs/0000_examples/objects/tubebig.stl')
        self.object_mesh_path = self.get_parameter(
            'object_mesh_path').get_parameter_value().string_value
        self.object_tube = cm.CollisionModel(self.object_mesh_path)
        self.object_tube.set_rgba([.9, .75, .35, .3])
        self.object_tube.attach_to(self.base)
        self.base.taskMgr.step()

        self.markers = MarkerArray()
        pose = Pose()
        pose.position.x = 0.
        pose.position.y = 0.
        pose.position.z = 0.
        pose.orientation.x = 0.
        pose.orientation.y = 0.
        pose.orientation.z = 0.
        pose.orientation.w = 1.
        scale = [1., 1., 1.]
        if gripper_name in ['robotiqhe', 'robotiq85', 'robotiq140']:
            pass
        else:
            self.get_logger().error(
                "The specified gripper is not implemented.",
                throttle_duration_sec=1)
        self.markers.markers.append(
            self.gen_marker(
                'base_link',
                'object',
                0,
                pose,
                self.object_mesh_path,
                scale=scale,
                color=[1.0, 0.5, 0.5, 0.5]))

        self.pose_dict = {}
        self.br = StaticTransformBroadcaster(self)
        if gripper_name in ['robotiqhe', 'robotiq85', 'robotiq140']:
            self.planning_service = self.create_service(
                Empty, 'plan_grasp', self.plan_grasps)
        else:
            self.get_logger().error(
                "The specified gripper is not implemented.",
                throttle_duration_sec=1)
        self.marker_pub = self.create_publisher(
            MarkerArray, 'grasp_pub', 1)
        self.timer = self.create_timer(0.1, self.update_tfs)

    def update_tfs(self):
        """ Sends tfs. """

        if self.pose_dict is not {}:
            for name, data in self.pose_dict.items():
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

    def gen_marker(
            self,
            frame_name,
            name, id_int,
            pose,
            mesh_path,
            scale=[1., 1., 1.],
            color=[0.8, 0.8, 0.8, 0.8]):
        """ Generates a marker.

            Attributes:
                frame_name (str): Frame name
                name (str): Unique marker name
                id_int (int): Unique id number
                pose (geometry_msgs/Pose): Pose of the marker
                mesh_path (str): Mesh file path
                scale (list(float)): Object scale to be displayed
                color (list(float)): Object color to be displayed
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
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.a = float(color[0])
        marker.color.r = float(color[1])
        marker.color.g = float(color[2])
        marker.color.b = float(color[3])
        marker.mesh_resource = 'file://' + mesh_path
        marker.mesh_use_embedded_materials = True

        return marker

    def plan_grasps(self, req, res):
        """ Plans grasps. """

        self.get_logger().info(
            f'self.openning_direction: {self.openning_direction}')
        grasp_info_list = gpa.plan_grasps(
            self.gripper,
            self.object_tube,
            angle_between_contact_normals=\
                np.radians(self.angle_between_contact_normals),
            openning_direction=\
                self.openning_direction,
            max_samples=\
                self.max_samples,
            min_dist_between_sampled_contact_points=\
                self.min_dist_between_sampled_contact_points,
            contact_offset=\
                self.contact_offset)
        self.get_logger().info(
            f'Number of generated grasps: {len(grasp_info_list)}')

        for i, grasp_info in enumerate(grasp_info_list):
            jaw_width, jaw_pos, jaw_rotmat, hnd_pos, hnd_rotmat = \
                grasp_info
            self.gripper.grip_at_with_jcpose(
                jaw_pos, jaw_rotmat, jaw_width)
            self.gripper.gen_meshmodel().attach_to(self.base)

            parent_frame = 'object'
            pose_b = Pose()
            pose_b.position.x = hnd_pos[0]
            pose_b.position.y = hnd_pos[1]
            pose_b.position.z = hnd_pos[2]
            q = mat2quat(hnd_rotmat)
            pose_b.orientation.x = q[1]
            pose_b.orientation.y = q[2]
            pose_b.orientation.z = q[3]
            pose_b.orientation.w = q[0]
            self.markers.markers.append(
                self.gen_marker(
                    parent_frame,
                    'hande_b_'+str(i),
                    0,
                    pose_b,
                    self.body_mesh_path))
            self.pose_dict['hande_b_'+str(i)] = \
                {'parent': parent_frame, 'pose': pose_b}
            self.update_tfs()

            for k, v in self.fingers_dict.items():
                pose = Pose()
                pose.position.x = v['gl_pos'][0]
                pose.position.y = v['gl_pos'][1]
                pose.position.z = v['gl_pos'][2]
                q = mat2quat(v['gl_rotmat'])
                pose.orientation.x = q[1]
                pose.orientation.y = q[2]
                pose.orientation.z = q[3]
                pose.orientation.w = q[0]
                scale = [1., 1., 1.]
                if v['scale'] is not None:
                    scale = v['scale']
                self.markers.markers.append(
                    self.gen_marker(
                        parent_frame,
                        k+'_'+str(i),
                        0,
                        pose,
                        v['mesh_file'],
                        scale))
                self.pose_dict[k+'_'+str(i)] = \
                    {'parent': parent_frame, 'pose': pose}

        return res


def main(args=None):
    rclpy.init(args=args)
    g_node = GraspPlanner()

    while rclpy.ok():
        g_node.marker_pub.publish(g_node.markers)
        g_node.base.taskMgr.step()
        rclpy.spin_once(g_node)

    g_node.destroy_service(g_node.planning_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
