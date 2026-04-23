import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from Triceratops_Config import RobotConfiguration, RobotDynamixel

from DXL_motor_control import DXL_Communication
from Gait_controller import GaitController
from Swing_controller import SwingController
from Stance_controller import StanceController

from Triceratops_IK import InverseKinematics

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from math import pi
import threading
import concurrent.futures
import traceback
import atexit
import time

DEGREE_TO_SERVO = 4095/360

class CmdVelSubscriber(Node):
    def __init__(self, mode_callback=None):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.listener_callback, 10
        )
        if mode_callback is not None:
            self.create_subscription(
                String, '/robot_mode', mode_callback, 10
            )

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0

        self.angular_x = 0.0
        self.angular_y = 0.0
        self.angular_z = 0.0

        self.get_logger().info('Vel Subscriber has been started')

    def listener_callback(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.linear_z = msg.linear.z

        self.angular_x = msg.angular.x
        self.angular_y = msg.angular.y
        self.angular_z = msg.angular.z

class RobotState:
    def __init__(self):
        self.config = RobotConfiguration()
        self.ticks = 0
        self.height = -0.10
        self.foot_locations = self.config.default_stance
        self.joint_angles = np.zeros((3, 4))
        self.last_goal = np.zeros(4)

class Command:
    def __init__(self):
        self.horizontal_velocity = np.array([0.0, 0.0])
        self.yaw_rate = 0
        self.height = -0.10

class RobotControl:
    def __init__(self):
        self.cmd_vel = CmdVelSubscriber(mode_callback=self._on_robot_mode)
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.cmd_vel)

        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.config = RobotConfiguration()
        self.gait_controller = GaitController(self.config)
        self.swing_controller = SwingController(self.config)
        self.stance_controller = StanceController(self.config)
        self.inverse_kinematics = InverseKinematics(self.config)

        self.state = RobotState()
        self.command = Command()
        self.control_cmd = ControlCmd()
        self.is_walking = False

    def _on_robot_mode(self, msg):
        mode = msg.data
        print(f"[robot_mode] 收到指令: {mode}")
        actions = {
            "start_gait":       self.start_gait,
            "stop":             self.stop_gait,
            "reset":            self.control_cmd.reset_to_original,
            "handshake":        self._handshake_thread,
            "sit":              self.sit,
            "shake_head_left":  lambda: self._shake_head_thread("left"),
            "shake_head_right": lambda: self._shake_head_thread("right"),
            "shake_head_center":lambda: self._shake_head_thread("center"),
        }
        if mode in actions:
            actions[mode]()
        else:
            print(f"[robot_mode] 未知指令: {mode}")

    def get_vel_data(self):
        self.command.horizontal_velocity = np.array([self.cmd_vel.linear_x, self.cmd_vel.linear_y])
        self.command.yaw_rate = self.cmd_vel.angular_z

    def __del__(self):
        pass

    def cleanup(self):
        self.executor.shutdown()

    def step_gait(self, state, command):
        contact_modes = self.gait_controller.contacts(state.ticks)
        new_foot_locations = np.zeros((3, 4))

        for leg_index in range(4):
            contact_mode = contact_modes[leg_index]
            if contact_mode == 1:
                new_location = self.stance_controller.next_foot_location(leg_index, state, command)
            else:
                swing_proportion = (
                    self.gait_controller.subphase_ticks(state.ticks) / self.config.swing_ticks
                )
                new_location = self.swing_controller.next_foot_location(
                    swing_proportion, leg_index, state, command
                )
            new_foot_locations[:, leg_index] = new_location

        return new_foot_locations, contact_modes

    def puppy_move(self):
        while self.is_walking:
            self.get_vel_data()

            with concurrent.futures.ThreadPoolExecutor() as executor:
                future_foot_locations = executor.submit(self.step_gait, self.state, self.command)

            self.state.joint_angles = self.inverse_kinematics.four_legs_inverse_kinematics(
                self.state.foot_locations
            )

            self.state.foot_locations, _ = future_foot_locations.result()

            goal = (
                self.state.joint_angles * 180 / 3.14 * DEGREE_TO_SERVO
                + self.config.leg_center_position
            )

            if np.linalg.norm(goal - self.state.last_goal) > self.config.goal_change_threshold:
                self.control_cmd.motor_position_control(goal)
                self.state.last_goal = goal

            self.state.ticks += 1

    def start_gait(self):
        self.stop_gait()
        self.is_walking = True
        self.puppy_move_thread = threading.Thread(target=self.puppy_move, daemon=True)
        self.puppy_move_thread.start()
        print("[start_gait] 步態啟動")

    def stop_gait(self):
        self.is_walking = False
        if hasattr(self, 'puppy_move_thread'):
            self.puppy_move_thread.join(timeout=3.0)
        print("[stop_gait] 步態停止")

    def _get_handshake_positions(self):
        hip_pos   = [2100, 2100, 2100, 2100]
        higher_FR = 2800
        higher_FL = 500
        higher_RR = 1500
        higher_RL = 2650
        lower_FR  = 2100
        lower_FL  = 1450
        lower_RR  = 1900
        lower_RL  = 1149

        default_pos = [[2048, 2048, 2048, 2048],
                       [2500, 1549, 1549, 2500],
                       [2100, 1949, 1949, 2100]]
        stand_pos   = [hip_pos,
                       [higher_FR, higher_FL, higher_RR, higher_RL],
                       [lower_FR,  lower_FL,  lower_RR,  lower_RL]]
        hip_pos   = [2100, 1600, 2100, 2100]
        fl_target_pos = [hip_pos,
                         [higher_FR, 600,  higher_RR, higher_RL],
                         [lower_FR,  1400, lower_RR,  lower_RL]]
        return default_pos, stand_pos, fl_target_pos

    def sit(self):
        """移動到握手支撐站姿，供 debug 用"""
        print("[sit] 開始")
        self.stop_gait()
        time.sleep(0.5)
        self.control_cmd.reset_to_original()
        time.sleep(0.5)
        default_pos, stand_pos, _ = self._get_handshake_positions()
        self._ramp_to_position(default_pos, stand_pos, steps=5)
        print("[sit] 完成")

    def handshake(self):
        """停步態 → 站好 → 漸進抬 FL 腿做握手 → 回站姿 → 重啟步態"""
        print("[handshake] 開始握手動作")
        self.stop_gait()
        time.sleep(0.5)
        self.control_cmd.reset_to_original()
        time.sleep(0.5)
        default_pos, stand_pos, fl_target_pos = self._get_handshake_positions()
        self._ramp_to_position(default_pos, stand_pos, steps=5)
        print("[handshake] 站好，準備抬腿")
        time.sleep(1)

        self._ramp_to_position(stand_pos, fl_target_pos)
        print("[handshake] 握手姿勢保持中")
        time.sleep(2)
        self._ramp_to_position(fl_target_pos, stand_pos, steps=10)
        time.sleep(1)
        self.control_cmd.reset_to_original()
        time.sleep(1)
        self.start_gait()
        print("[handshake] 完成")

    def _ramp_to_position(self, start_position, target_position, steps=20, delay=0.05):
        for i in range(1, steps + 1):
            progress = i / steps
            pos = [
                [int(start_position[joint][leg] + progress * (target_position[joint][leg] - start_position[joint][leg]))
                 for leg in range(4)]
                for joint in range(3)
            ]
            self.control_cmd.motor_position_control(pos)
            time.sleep(delay)

    def _handshake_thread(self):
        threading.Thread(target=self.handshake, daemon=True).start()

    def sway(self):
        """腰部左右晃動"""
        print("[sway] 開始左右晃動")
        self.control_cmd.mid_motor_position_control(mid_position=1250, step=20, delay=0.01)
        print("[sway] 左側")
        time.sleep(1)
        self.control_cmd.mid_motor_position_control(mid_position=2846, step=10, delay=0.01)
        print("[sway] 右側")
        time.sleep(1)
        self.control_cmd.mid_motor_position_control(mid_position=2048, step=10, delay=0.01)
        print("[sway] 回中心完成")

    def _sway_thread(self):
        threading.Thread(target=self.sway, daemon=True).start()

    HEAD_PAN = {"left": 1800, "right": 2250, "center": 2048}

    def shake_head(self, mode):
        target = self.HEAD_PAN.get(mode)
        if target is None:
            print(f"[shake_head] 未知 mode: {mode}，可用: left / right / center")
            return
        print(f"[shake_head] {mode}")
        self.control_cmd.head_motor_control(target)

    def _shake_head_thread(self, mode):
        threading.Thread(target=self.shake_head, args=(mode,), daemon=True).start()


class ControlCmd:
    """Low-level control of Dynamixel motors."""
    def __init__(self):
        self.setup_dynamixel()
        self.setup_motors()
        self.initialize_motor_states()
        self.walking_freq = 2000  # 2000Hz

    def setup_dynamixel(self):
        self.robot_dynamixel = RobotDynamixel()
        self.dynamixel = DXL_Communication(self.robot_dynamixel.DEVICE_NAME, self.robot_dynamixel.B_RATE)
        self.dynamixel.activateDXLConnection()

    def setup_motors(self):
        motor_names = ['FR_higher', 'FR_lower', 'FR_hip',
                       'FL_higher', 'FL_lower', 'FL_hip',
                       'RR_higher', 'RR_lower', 'RR_hip',
                       'RL_higher', 'RL_lower', 'RL_hip',
                       'waist_axis1', 'waist_axis2',
                       'head_tilt', 'head_pan']

        self.motor_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16]
        self.motors = {name: self.dynamixel.createMotor(name, motor_number=id_)
                       for name, id_ in zip(motor_names, self.motor_ids)}

        self.leg_motor_list = [
            [self.motors['FR_hip'], self.motors['FL_hip'], self.motors['RR_hip'], self.motors['RL_hip']],
            [self.motors['FR_higher'], self.motors['FL_higher'], self.motors['RR_higher'], self.motors['RL_higher']],
            [self.motors['FR_lower'], self.motors['FL_lower'], self.motors['RR_lower'], self.motors['RL_lower']]
        ]

    def initialize_motor_states(self):
        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()
        self.enable_all_motor()
        self.joint_position = np.zeros(12)
        self.head_pan_position = 2048
        self.waist_position = 2048
        self.last_commanded_position = [[2048]*4, [2048]*4, [2048]*4]

    def __del__(self):
        self.cleanup()

    def cleanup(self):
        self.disable_all_motor()
        self.dynamixel.closeHandler()

    def enable_all_motor(self):
        for motor in self.motors.values():
            motor.enableMotor()

    def disable_all_motor(self):
        for motor in self.motors.values():
            motor.disableMotor()

    def read_all_motor_data(self):
        pos = self.last_commanded_position
        print(f"[last commanded]")
        print(f"  hip    : {pos[0]}")
        print(f"  higher : {pos[1]}")
        print(f"  lower  : {pos[2]}")
        print(f"  waist  : {self.waist_position}  head_pan: {self.head_pan_position}")

    def test_bulk_read(self, n=50):
        success, fail = 0, 0
        for _ in range(n):
            before = self.dynamixel._DXL_Communication__communicate_error_count
            self.dynamixel.updateMotorData()
            after = self.dynamixel._DXL_Communication__communicate_error_count
            if after > before:
                fail += 1
            else:
                success += 1
            time.sleep(0.02)
        print(f"[bulk_read_test] {n} 次: 成功 {success}, 失敗 {fail}, 失敗率 {fail/n*100:.1f}%")

    def update_joint_state(self):
        self.dynamixel.updateMotorData()
        self.joint_position = np.zeros((3, 4))
        self.joint_position[0] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[0]]
        self.joint_position[1] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[1]]
        self.joint_position[2] = [motor.PRESENT_POSITION_value for motor in self.leg_motor_list[2]]

    def reset_to_original(self, position=None):
        self.motor_position_control()

    def motor_position_control(self, position=None, waist=None):
        if position is None:
            position = [[2048, 2048, 2048, 2048],
                        [2500, 1549, 1549, 2500],
                        [2100, 1949, 1949, 2100]]

        self.last_commanded_position = position
        for i, motor_list in enumerate(self.leg_motor_list):
            for j, motor in enumerate(motor_list):
                motor.writePosition(int(position[i][j]))

        if waist is None:
            waist = [2048, 2030]
        self.motors['waist_axis1'].writePosition(int(waist[0]))
        self.motors['waist_axis2'].writePosition(int(waist[1]))
        self.motors['head_pan'].writePosition(2048)

        self.dynamixel.sentAllCmd()

    def head_motor_control(self, target_position, step=10, delay=0.01):
        current = self.head_pan_position
        direction = 1 if target_position > current else -1
        while (direction == 1 and current < target_position) or \
              (direction == -1 and current > target_position):
            current += direction * step
            current = min(current, target_position) if direction == 1 else max(current, target_position)
            self.motors['head_pan'].writePosition(current)
            self.dynamixel.sentAllCmd()
            time.sleep(delay)
        self.head_pan_position = target_position

    def mid_motor_position_control(self, mid_position, step=20, delay=0.01):
        """漸進移動腰部馬達到目標位置"""
        current = self.waist_position
        direction = 1 if mid_position > current else -1

        while (direction == 1 and current < mid_position) or \
              (direction == -1 and current > mid_position):
            current += direction * step
            current = min(current, mid_position) if direction == 1 else max(current, mid_position)
            self.motors['waist_axis1'].writePosition(current)
            self.motors['waist_axis2'].writePosition(current)
            self.dynamixel.sentAllCmd()
            time.sleep(delay)
        self.waist_position = mid_position


def main():
    rclpy.init()
    robot_control = RobotControl()
    robot_control.control_cmd.reset_to_original()

    command_dict = {
        "s":        robot_control.start_gait,
        "stop":     robot_control.stop_gait,
        "reset":    robot_control.control_cmd.reset_to_original,
        "enable":   robot_control.control_cmd.enable_all_motor,
        "disable":  robot_control.control_cmd.disable_all_motor,
        "read":     robot_control.control_cmd.read_all_motor_data,
        "test_read":robot_control.control_cmd.test_bulk_read,
        "handshake":  robot_control._handshake_thread,
        "sway":       robot_control._sway_thread,
        "sit":        robot_control.sit,
        "shake_head_left":  lambda: robot_control._shake_head_thread("left"),
        "shake_head_right": lambda: robot_control._shake_head_thread("right"),
        "shake_head_center":lambda: robot_control._shake_head_thread("center"),
    }

    atexit.register(robot_control.cleanup)

    while True:
        try:
            cmd = input("CMD : ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                break
        except Exception as e:
            traceback.print_exc()
            break


if __name__ == "__main__":
    main()
