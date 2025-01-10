#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose, Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from control_msgs.action import GripperCommand, FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, Header
from enum import Enum
import numpy as np
import tf2_ros
import time

class TaskState(Enum):
    IDLE = 0
    PLANNING = 1
    MOVING_TO_PICK = 2
    PICKING = 3
    MOVING_TO_PLACE = 4
    PLACING = 5
    ERROR = 6
    FINISHED = 7

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        # Create callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Initialize action clients
        self.move_group_client = ActionClient(
            self, 
            MoveGroup, 
            'move_group',
            callback_group=self.callback_group
        )
        self.gripper_client = ActionClient(
            self, 
            GripperCommand, 
            'gripper_command',
            callback_group=self.callback_group
        )
        self.trajectory_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            'follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Initialize publishers
        self.scene_pub = self.create_publisher(
            PlanningScene,
            'planning_scene',
            10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            'visualization_marker_array',
            10
        )
        
        # Initialize subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # TF2 Buffer for coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Define poses (will be updated with actual robot coordinates)
        self.pick_pose = Pose(position=Point(x=0.5, y=0.0, z=0.1))
        self.place_pose = Pose(position=Point(x=0.5, y=0.5, z=0.1))
        
        # Initialize state and status variables
        self.state = TaskState.IDLE
        self.joint_states = None
        self.planning_attempts = 0
        self.max_planning_attempts = 3
        
        # Create timers
        self.state_timer = self.create_timer(
            0.1, 
            self.state_machine_callback,
            callback_group=self.callback_group
        )
        self.visualization_timer = self.create_timer(
            0.5,
            self.publish_visualization,
            callback_group=self.callback_group
        )
        
        # Initialize planning scene
        self.setup_planning_scene()

    def setup_planning_scene(self):
        """Set up the planning scene with collision objects"""
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        
        # Add table as collision object
        table = CollisionObject()
        table.header.frame_id = 'base_link'
        table.id = 'table'
        
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [1.5, 1.0, 0.02]  # Length, width, height
        
        table.primitives.append(box)
        table_pose = Pose()
        table_pose.position.x = 0.5
        table_pose.position.y = 0.0
        table_pose.position.z = -0.01  # Half height of table
        table.primitive_poses.append(table_pose)
        
        scene_msg.world.collision_objects.append(table)
        self.scene_pub.publish(scene_msg)

    def joint_state_callback(self, msg):
        """Store latest joint states"""
        self.joint_states = msg

    async def state_machine_callback(self):
        """Main state machine for pick and place operation"""
        try:
            if self.state == TaskState.IDLE:
                self.get_logger().info('Starting pick and place operation')
                self.state = TaskState.PLANNING
                self.planning_attempts = 0
                
            elif self.state == TaskState.PLANNING:
                success = await self.plan_movement(self.pick_pose)
                if success:
                    self.state = TaskState.MOVING_TO_PICK
                elif self.planning_attempts >= self.max_planning_attempts:
                    self.state = TaskState.ERROR
                    self.get_logger().error('Failed to plan movement after maximum attempts')
                else:
                    self.planning_attempts += 1
                    
            elif self.state == TaskState.MOVING_TO_PICK:
                if await self.check_movement_complete():
                    self.state = TaskState.PICKING
                    await self.actuate_gripper(close=True)
                    
            elif self.state == TaskState.PICKING:
                if await self.check_gripper_complete():
                    success = await self.plan_movement(self.place_pose)
                    if success:
                        self.state = TaskState.MOVING_TO_PLACE
                    else:
                        self.state = TaskState.ERROR
                        
            elif self.state == TaskState.MOVING_TO_PLACE:
                if await self.check_movement_complete():
                    self.state = TaskState.PLACING
                    await self.actuate_gripper(close=False)
                    
            elif self.state == TaskState.PLACING:
                if await self.check_gripper_complete():
                    self.state = TaskState.FINISHED
                    self.get_logger().info('Pick and place operation completed successfully')
                    
            elif self.state == TaskState.ERROR:
                # Implement error recovery here
                pass
                
        except Exception as e:
            self.get_logger().error(f'Error in state machine: {str(e)}')
            self.state = TaskState.ERROR

    async def plan_movement(self, target_pose: Pose) -> bool:
        """Plan and execute movement to target pose"""
        goal_msg = MoveGroup.Goal()
        
        # Set up the motion plan request
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        
        # Set the target pose
        constraint = moveit_msgs.msg.Constraints()
        constraint.position_constraints.append(
            self.create_position_constraint(target_pose)
        )
        constraint.orientation_constraints.append(
            self.create_orientation_constraint(target_pose)
        )
        goal_msg.request.goal_constraints.append(constraint)
        
        # Send the goal and wait for result
        goal_handle = await self.move_group_client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return False
            
        result = await goal_handle.get_result_async()
        return result.result.error_code.val == 1  # SUCCESS

    async def actuate_gripper(self, close: bool):
        """Control gripper with force feedback"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.0 if close else 0.08
        goal_msg.command.max_effort = 50.0
        
        self.get_logger().info(f'{"Closing" if close else "Opening"} gripper')
        
        goal_handle = await self.gripper_client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            self.get_logger().error('Gripper command rejected')
            self.state = TaskState.ERROR
            return
            
        self.current_gripper_goal = goal_handle

    async def check_movement_complete(self):
        """Check if robot has reached target pose with tolerance"""
        try:
            current_pose = await self.get_current_pose()
            target_pose = self.pick_pose if self.state == TaskState.MOVING_TO_PICK else self.place_pose
            
            position_tolerance = 0.01  # 1cm
            orientation_tolerance = 0.1  # ~5.7 degrees
            
            position_error = np.linalg.norm([
                current_pose.position.x - target_pose.position.x,
                current_pose.position.y - target_pose.position.y,
                current_pose.position.z - target_pose.position.z
            ])
            
            # Check if position error is within tolerance
            return position_error < position_tolerance
            
        except Exception as e:
            self.get_logger().error(f'Error checking movement completion: {str(e)}')
            return False

    async def check_gripper_complete(self):
        """Check if gripper has reached target position with force feedback"""
        if not hasattr(self, 'current_gripper_goal'):
            return False
            
        result = await self.current_gripper_goal.get_result_async()
        return result.result.success

    def publish_visualization(self):
        """Publish visualization markers for motion planning"""
        marker_array = MarkerArray()
        
        # Add markers for pick and place poses
        marker_array.markers.append(self.create_pose_marker(
            self.pick_pose, 'pick_pose', [0.0, 1.0, 0.0, 0.8]
        ))
        marker_array.markers.append(self.create_pose_marker(
            self.place_pose, 'place_pose', [1.0, 0.0, 0.0, 0.8]
        ))
        
        # Add planned trajectory visualization if available
        if hasattr(self, 'planned_trajectory'):
            marker_array.markers.append(self.create_trajectory_marker())
        
        self.marker_pub.publish(marker_array)

    def create_pose_marker(self, pose: Pose, marker_id: str, color: list):
        """Create a visualization marker for a pose"""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pick_and_place'
        marker.id = hash(marker_id)
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = pose
        marker.scale = Vector3(x=0.1, y=0.01, z=0.01)
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        return marker

    def create_trajectory_marker(self):
        """Create a visualization marker for the planned trajectory"""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'pick_and_place'
        marker.id = hash('trajectory')
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.005, y=0.0, z=0.0)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)
        
        # Add trajectory points
        for point in self.planned_trajectory.points:
            p = Point()
            p.x = point.positions[0]
            p.y = point.positions[1]
            p.z = point.positions[2]
            marker.points.append(p)
            
        return marker

def main():
    rclpy.init()
    node = PickAndPlaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()