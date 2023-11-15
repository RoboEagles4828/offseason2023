import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
import torch
import torch.nn as nn
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
from rl_games.algos_torch.models import ModelA2CContinuous
from rl_games.algos_torch.torch_ext import load_checkpoint
from rl_games.algos_torch.network_builder import A2CBuilder
from rl_games.common.a2c_common import ContinuousA2CBase
import yaml

class Reader(Node):
    def __init__(self):
        super().__init__("reinforcement_learning_runner")
        # self.robot_ip = robot_ip
        # self.policy = torch.load("/workspaces/offseason2023/isaac/Eaglegym/eaglegym/runs/EdnaK/nn/EdnaK.pth")
        
        self.policy = self.load_checkpoint("/workspaces/offseason2023/isaac/Eaglegym/eaglegym/runs/EdnaK/nn/EdnaK.pth")
        
        self.joint_action_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # self.joint_trajectory_action_pub = self.create_publisher(Twist, "joint_trajectory_message", 10)
        self.odom_sub = self.create_subscription(Odometry, "/saranga/zed/odom", self.odom_callback, 10)
        self.target_sub = self.create_subscription(String, '/real/obj_det_pose', self.target_callback, 10)
        # self.joint_state_sub = self.create_subscription(Float32, "joint_state", self.joint_state_callback, 10)
        self.odom_msg = Odometry()
        # self.joint_state_msg = JointState()
        self.twist_msg = Twist()
        # self.cmds = JointTrajectory()
        # self.position_cmds = JointTrajectoryPoint()
        self.episode_reward = 0
        self.step = 0
        
        self.i = 0
        # self.joints = [
        #     'arm_roller_bar_joint',
        #     'elevator_center_joint',
        #     'elevator_outer_1_joint',
        #     'elevator_outer_2_joint',
        #     'top_gripper_right_arm_joint',
        #     'top_gripper_left_arm_joint',
        #     'top_slider_joint',
        #     'bottom_intake_joint',
        # ]
        
        self.target_pos = []

        self.get_logger().info("\033[92m" + "Policy Runner Started" + "\033[0m")
        
    def load_checkpoint(self, filepath):
        config = yaml.load(open("/workspaces/offseason2023/isaac/Eaglegym/eaglegym/cfg/train/EdnaKPPO.yaml", "r"), Loader=yaml.FullLoader)
        config = config["params"]
        state = load_checkpoint(filepath)
        state_dict = {k.replace('a2c_network.', ''): v for k, v in state['model'].items()}
        builder = A2CBuilder()
        builder.load(config["network"])
        network = builder.build("network", actions_num=10, input_shape=(13,))
        
        model_state_dict = network.state_dict()
        state_dict = {k: v for k, v in state_dict.items() if k in model_state_dict}
        
        network.load_state_dict(state_dict)
        network.eval()

        return network
        # model = checkpoint['model']
        # model.load_state_dict(checkpoint)
        # for parameter in model.parameters():
        #     parameter.requires_grad = False
        # model.eval()
        # return model

    def get_action(self, msg):
        '''
        Gym obs type for EdnaK:
            [0:3] = ([target_pos] - [robot_pos]) / 3 # x, y, z
            [3:7] = [robot_rotation_quaternion] # x, y, z, w
            [7:10] = [robot_linear_velocities] / 2 # x, y, z
            [10:13] = [robot_angular_velocities] / M_PI # x, y, z
            
        Input type String:
            0,1,2,3,4,5,6,7,8,9,10,11,12,13
        '''
                
        # convert string to list
        # input_obs = msg.data.split(",")
        
        # obs = np.array(input_obs, dtype=np.float32)
        
        real_obs = np.zeros((1, 13))

        print(self.policy[real_obs])
        
        # if self.i == 0:
        #     for k, v in self.policy.items():
        #         print(f"Key: {k} Type: {type(v)}")
        #     self.i = 1
        # action = self.runner.get_action(torch.tensor(obs).float())
        # print(action)
        # self.twist_msg.linear.x = action[0].detach().numpy()
        # self.twist_msg.linear.y = action[1].detach().numpy()
        # self.twist_msg.angular.z = action[2].detach().numpy()
        # self.position_cmds.positions = [
        #     action[3].detach().numpy(),
        #     action[4].detach().numpy(),
        #     action[5].detach().numpy(),
        #     action[4].detach().numpy(),
        #     action[6].detach().numpy(),
        #     action[6].detach().numpy(),
        #     action[7].detach().numpy(),
        #     action[6].detach().numpy(),
        #     action[8].detach().numpy(),
        #     action[8].detach().numpy(),
        # ]
        # self.cmds.joint_names = self.joints
        # self.cmds.points = [self.position_cmds]
        
        # self.publisher_.publish(self.cmds)
        # self.get_logger().info("Action: " + "\033[93m" + str(self.twist_msg) + "\033[0m")
        
        self.joint_action_pub.publish(self.twist_msg)
        self.step += 1

    def odom_callback(self, msg: Odometry):
        if(msg != None and len(self.target_pos) > 0):
            self.odom_msg = msg
            obs_string = String()
            robot_pos = [
                float(self.odom_msg.pose.pose.position.x),
                float(self.odom_msg.pose.pose.position.y),
                float(self.odom_msg.pose.pose.position.z)
            ]
            robot_rot_quat = [
                float(self.odom_msg.pose.pose.orientation.x),
                float(self.odom_msg.pose.pose.orientation.y),
                float(self.odom_msg.pose.pose.orientation.z),
                float(self.odom_msg.pose.pose.orientation.w)
            ]
            robot_linear_vel = [
                float(self.odom_msg.twist.twist.linear.x),
                float(self.odom_msg.twist.twist.linear.y),
                float(self.odom_msg.twist.twist.linear.z)
            ]
            robot_angular_vel = [
                float(self.odom_msg.twist.twist.angular.x),
                float(self.odom_msg.twist.twist.angular.y),
                float(self.odom_msg.twist.twist.angular.z)
            ]
            
            obs_input = [
                (self.target_pos[0] - robot_pos[0]) / 3,
                (self.target_pos[1] - robot_pos[1]) / 3,
                (self.target_pos[2] - robot_pos[2]) / 3,
                robot_rot_quat[0],
                robot_rot_quat[1],
                robot_rot_quat[2],
                robot_rot_quat[3],
                robot_linear_vel[0],
                robot_linear_vel[1],
                robot_linear_vel[2],
                robot_angular_vel[0],
                robot_angular_vel[1],
                robot_angular_vel[2]
            ]
            
            obs_string.data = ",".join([str(i) for i in obs_input])
            
            # self.get_logger().info(f"Observation: \033[92m {obs_string} \033[0m")
            
            self.get_action(obs_string)
        return
    
    def target_callback(self, msg):
        if(msg != None):
            self.target_pos = msg.data.split("|")
            self.target_pos = [float(i) for i in self.target_pos]
            self.get_action(",")
            # self.get_logger().info(f"Target Pos: {self.target_pos} {self.odom_sub.topic_name}")         
        return

    def get_reward():
        return

def main(args=None):
    # env = gym.create_env("RealRobot", ip=self.robot_ip)
    rclpy.init(args=args)
    reader = Reader()
    rclpy.spin(reader)
    # env.disconnect()
if __name__ == '__main__':
    main()