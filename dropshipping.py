import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup
from std_msgs.msg import Bool
from enum import Enum

class TaskState(Enum):
    IDLE = 0
    MOVING_TO_PICK = 1
    PICKING = 2
    MOVING_TO_PLACE = 3
    PLACING = 4
    FINISHED = 5

class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        
        # Initialize action clients
        self.move_group_client = ActionClient(self, MoveGroup, 'move_group')
        self.gripper_client = ActionClient(self, GripperCommand, 'gripper_command')
        
        # Define poses
        self.pick_pose = Pose(position=Point(x=0.5, y=0.0, z=0.1))  # can coordinates
        self.place_pose = Pose(position=Point(x=0.5, y=0.5, z=0.1))  
        
        # Initialize state
        self.state = TaskState.IDLE
        
        # Create a timer to check task progress
        self.timer = self.create_timer(0.1, self.state_machine_callback)
        
    async def state_machine_callback(self):
        if self.state == TaskState.IDLE:
            self.get_logger().info('Starting pick and place operation')
            self.state = TaskState.MOVING_TO_PICK
            await self.move_to_pose(self.pick_pose)
            
        elif self.state == TaskState.MOVING_TO_PICK:
            if self.check_movement_complete():
                self.state = TaskState.PICKING
                await self.actuate_gripper(close=True)
                
        elif self.state == TaskState.PICKING:
            if self.check_gripper_complete():
                self.state = TaskState.MOVING_TO_PLACE
                await self.move_to_pose(self.place_pose)
                
        elif self.state == TaskState.MOVING_TO_PLACE:
            if self.check_movement_complete():
                self.state = TaskState.PLACING
                await self.actuate_gripper(close=False)
                
        elif self.state == TaskState.PLACING:
            if self.check_gripper_complete():
                self.state = TaskState.FINISHED
                self.get_logger().info('Pick and place operation completed')
    
    async def move_to_pose(self, target_pose):
        """Send movement command to MoveGroup action server"""
        goal_msg = MoveGroup.Goal()
        # Set up the motion plan request
        goal_msg.request.workspace_parameters.header.frame_id = 'base_link'
        goal_msg.request.group_name = 'arm'
        goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitive_poses[0] = target_pose
        
        self.get_logger().info(f'Sending move goal to pose: {target_pose}')
        await self.move_group_client.send_goal_async(goal_msg)
    
    async def actuate_gripper(self, close: bool):
        """Send command to gripper"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.0 if close else 0.08  # Example gripper positions
        goal_msg.command.max_effort = 50.0
        
        self.get_logger().info(f'{"Closing" if close else "Opening"} gripper')
        await self.gripper_client.send_goal_async(goal_msg)
    
    def check_movement_complete(self):
        """Check if the robot has reached its target pose"""
        # In a real implementation, check the action result
        # This is a simplified version
        return True
    
    def check_gripper_complete(self):
        """Check if the gripper has completed its motion"""
        # In a real implementation, you would check the action result
        # This is a simplified version
        return True

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