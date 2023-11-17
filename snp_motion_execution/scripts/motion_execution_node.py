#!/usr/bin/env python3
import threading
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.waitable import Waitable
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from snp_msgs.srv import ExecuteMotionPlan


class MotionExecServer():
    def __init__(self):
        self.node = Node('motion_execution_node')
        self.executor = MultiThreadedExecutor()
        # Add node to executor
        self.executor.add_node(self.node) 

        self.MOTION_EXEC_SERVICE = "execute_motion_plan"
        self.ENABLE_SERVICE = "robot_enable"
        self.FJT_ACTION_PARAM = "follow_joint_trajectory_action"
        self.JOINT_STATES_TOPIC = "joint_states"
        self.JOINT_STATE_TIME_THRESHOLD = 0.1  # seconds

        self.mutex = threading.Lock()
        self.latest_joint_state = JointState()
        self.cb_group = ReentrantCallbackGroup()
        fjt_action = self.node.declare_parameter(self.FJT_ACTION_PARAM, 
                                                 "joint_trajectory_position_controller/follow_joint_trajectory").value

        # Start subscribers, clients, and server
        self.joint_state_sub = self.node.create_subscription(JointState, self.JOINT_STATES_TOPIC, self.joint_state_cb, 10, callback_group=self.cb_group)
        self.fjt_client = ActionClient(self.node, FollowJointTrajectory, fjt_action, callback_group=self.cb_group)
        self.enable_client = self.node.create_client(Trigger, self.ENABLE_SERVICE, callback_group=self.cb_group)
        self.server = self.node.create_service(ExecuteMotionPlan, self.MOTION_EXEC_SERVICE, callback=self.execute_motion_plan, callback_group=self.cb_group)

        self.node.get_logger().info("Motion execution node started")

    '''Callback to update latest joint state'''
    def joint_state_cb(self, state: JointState):
        if self.mutex.acquire(blocking=False):
            self.latest_joint_state = state
            self.mutex.release()

    '''
    Calculate age of latest joint stage
    '''
    def get_joint_state_age(self):
        if self.mutex.acquire(blocking=False):
            current_time = self.node.get_clock().now().seconds_nanoseconds()
            latest_js_time = self.latest_joint_state.header.stamp
            self.mutex.release()
            time_diff = current_time[0] + current_time[1]*1e-9 - (latest_js_time.sec + latest_js_time.nanosec*1e-9)

            return time_diff

    '''Call service to enable robot'''
    def enable_robot(self):
        self.node.get_logger().info("Enabling robot")
        if not self.enable_client.service_is_ready():
            raise RuntimeError("Robot enable server is not available")
        
        request = Trigger.Request()
        future = self.enable_client.call_async(request)
        self.executor.spin_until_future_complete(future)
        
        response = future.result()
        if not response.success:
            raise RuntimeError("Failed to enable robot: '" + response.message + "'")
    
        self.node.get_logger().info("Robot enabled")

    '''
    Replace start state of joint trajectory with latest joint state
    '''
    def replace_start_state(self, fjt_goal: FollowJointTrajectory.Goal) -> FollowJointTrajectory.Goal:
        start_point= JointTrajectoryPoint()
        start_point.positions = len(fjt_goal.trajectory.joint_names)*[0.0]
        start_point.velocities = len(start_point.positions)*[0.0]
        start_point.accelerations = len(start_point.positions)*[0.0]
        start_point.effort = len(start_point.positions)*[0.0]
        start_point.time_from_start = Duration(sec=0, nanosec=0)

        # Find the index of the trajectory joint in the latest joint state message
        if self.mutex.acquire(blocking=False):
            for i in range(len(fjt_goal.trajectory.joint_names)):
                name = fjt_goal.trajectory.joint_names[i]
                idx = np.where(np.array(self.latest_joint_state.name) == name)[0]
                if len(idx) == 0:
                    raise RuntimeError("Failed to find joint '" + name + "' in latest joint state message")
                else:
                    idx = idx[0]    # hopefully there will be only one matching joint name
                
                start_point.positions[i] = self.latest_joint_state.position[idx]

            self.mutex.release()

        fjt_goal.trajectory.points[0] = start_point
        return fjt_goal

    '''Execute motion plan request'''
    def execute_motion_plan(self, request: ExecuteMotionPlan.Request, 
                            result: ExecuteMotionPlan.Request) -> ExecuteMotionPlan.Request:
        try:
            # Check the last received joint state
            js_age = self.get_joint_state_age()
            if js_age > self.JOINT_STATE_TIME_THRESHOLD:
                raise RuntimeError("Last joint state was not received within threshold (" + 
                                   str(js_age) + " > " + str(self.JOINT_STATE_TIME_THRESHOLD) + ")") 

            # Enable robot
            self.enable_robot()

            # Check that the server exists
            if not self.fjt_client.server_is_ready():
                raise RuntimeError("Action server not available")
            
            # Sleep to ensure that robot_enable actually did everything it had to. 
            # This is probably only necessary the first time that servos are enabled.
            time.sleep(1)
            
            # Send motion trajectory
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = request.motion_plan

            # Replace the start state of the trajectory with the current joint state
            goal_msg = self.replace_start_state(goal_msg)

            self.node.get_logger().info("Sending joint trajectory")
            goal_handle_future = self.fjt_client.send_goal_async(goal_msg)
            self.executor.spin_until_future_complete(goal_handle_future)
            
            goal_handle = goal_handle_future.result()

            if (goal_handle.status != GoalStatus.STATUS_ACCEPTED and 
                goal_handle.status != GoalStatus.STATUS_SUCCEEDED and 
                goal_handle.status != GoalStatus.STATUS_EXECUTING):
                raise RuntimeError("Follow joint trajectory action goal was not accepted (code " +
                                   str(goal_handle.status) + ")")
    
            # Wait for the trajectory to complete
            fjt_future = goal_handle.get_result_async()
            timeout = float(goal_msg.trajectory.points[-1].time_from_start.sec) * 1.5
            self.executor.spin_until_future_complete(fjt_future, timeout_sec=timeout)
            if not fjt_future.done():
                raise RuntimeError(f"Timed out waiting for trajectory to finish, > {timeout} sec")
            
            # Handle the action result code
            fjt_wrapper = fjt_future.result()
            if fjt_wrapper.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError("Follow joint trajectory action call did not succeed")

            # Handle the FJT error code
            if fjt_wrapper.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError("Follow joint trajectory action did not succeed: '" +
                                        fjt_wrapper.result.error_string + "'")

            # Communicate success
            result.success = True
            return result

        except Exception as e:
            result.success = False
            self.node.get_logger().error(f'{e}')
            return result

if __name__ == "__main__":
    rclpy.init()

    server = MotionExecServer()
    server.executor.spin()

    rclpy.shutdown()