#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs_v4.srv import (
    ModbusCreate, ModbusClose,
    SetHoldRegs, GetHoldRegs,
    GripperCommand
)
import time
import re
from std_msgs.msg import Float64

# ---------------- 配置 ----------------
class Config:
    PART_CONFIG = {
        -1: {"open_width": 52, "speed": 255, "force": 240}, # 打开最大
         1: {"open_width": 25, "speed": 255, "force": 240}, # 销轴
         2: {"open_width": 35, "speed": 255, "force": 240}, # 滚轮
         3: {"open_width": 35, "speed": 255, "force": 240}, # 直通接头
         4: {"open_width": 35, "speed": 255, "force": 240}, # 油缸直角接头
         5: {"open_width": 20, "speed": 255, "force": 240}, # 小双通头
         6: {"open_width": 35, "speed": 255, "force": 240}, # 踏板挡块
         7: {"open_width": 25, "speed": 255, "force": 240}, # 光电开关支板
         8: {"open_width": 45, "speed": 255, "force": 240}, # 急停开关
         9: {"open_width": 40, "speed": 255, "force": 240}, # 蓝光灯架
        10: {"open_width": 30, "speed": 255, "force": 240}, # 气弹簧组件
        11: {"open_width": 25, "speed": 255, "force": 240}, # 侧板销
        12: {"open_width": 30, "speed": 255, "force": 240}  # 硬管组件
    }

    GRIPPER_SPEC = {
        "max_width": 52,
        "min_width": 0,
        "position_ratio": 1260,
    }

REG_CONTROL  = 1000
REG_POSITION = 1001
REG_SPEED    = 1002
REG_STATUS   = 2000

# ---------------- 独立同步调用器（无线程） ----------------
class SyncCaller:
    """使用独立客户端节点进行阻塞式服务调用，无需背景线程"""
    def __init__(self, node):
        self.node = node

    def call(self, cli, req, timeout=5.0):
        if not cli.service_is_ready():
            if not cli.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Service {cli.srv_name} not available")
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if not fut.done():
            raise TimeoutError("service call timed out")
        return fut.result()

# ---------------- 客户端专用节点 ----------------
class ClientNode(Node):
    def __init__(self):
        super().__init__('gripper_client_node')

        # 创建客户端
        self.cli_mod_create = self.create_client(ModbusCreate, '/dobot_bringup_ros2/srv/ModbusCreate')
        self.cli_mod_close  = self.create_client(ModbusClose , '/dobot_bringup_ros2/srv/ModbusClose')
        self.cli_set_hold   = self.create_client(SetHoldRegs , '/dobot_bringup_ros2/srv/SetHoldRegs')
        self.cli_get_hold   = self.create_client(GetHoldRegs , '/dobot_bringup_ros2/srv/GetHoldRegs')

        for cli in (self.cli_mod_create, self.cli_mod_close,
                    self.cli_set_hold, self.cli_get_hold):
            while not cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{cli.srv_name} not ready, waiting...')

# ---------------- 主节点 ----------------
class GripperServiceNode(Node):
    def __init__(self):
        super().__init__('gripper_service_node')
        self._current_width_mm = 0.0  # 缓存当前夹爪宽度

        # 创建客户端节点和同步调用器
        self.client_node = ClientNode()
        self.sync_caller = SyncCaller(self.client_node)

        # 对外服务
        self.srv_open  = self.create_service(GripperCommand, 'gripper/open',  self.cb_open)
        self.srv_close = self.create_service(GripperCommand, 'gripper/close', self.cb_close)

        # 添加状态发布
        self.pub_status = self.create_publisher(Float64, '/gripper/status', 10)
        self.timer = self.create_timer(0.1, self._publish_status)
        
        self.modbus_id = None
        self.auto_init()

    # ---------------- 通用工具 ----------------
    def call_sync(self, cli, req, timeout=5.0):
        return self.sync_caller.call(cli, req, timeout)

    # ---------------- Modbus 生命周期 ----------
    def modbus_create(self):
        req = ModbusCreate.Request()
        req.ip, req.port, req.slave_id, req.is_rtu = '192.168.1.6', 60000, 9, 1
        res = self.call_sync(self.client_node.cli_mod_create, req)
        # self.get_logger().info(f"------------------------------{res.robot_return}")
        m = re.search(r'\{(\d+)\}', res.robot_return)
        if not m:
            raise RuntimeError('Cannot parse modbus id')
        self.modbus_id = int(m.group(1))
        self.get_logger().info(f'Modbus created, id={self.modbus_id}')

    def modbus_close(self):
        if self.modbus_id is None:
            self.modbus_id = 0  #
        req = ModbusClose.Request()
        req.index = self.modbus_id
        self.call_sync(self.client_node.cli_mod_close, req)
        self.modbus_id = None

    # ---------------- 寄存器读写 ---------------
    def set_hold(self, addr, count, values, dtype='U16'):
        req = SetHoldRegs.Request()
        req.index, req.addr, req.count, req.val_tab, req.val_type = self.modbus_id, addr, count, values, dtype
        return self.call_sync(self.client_node.cli_set_hold, req)

    def get_hold(self, addr, count, dtype='U16'):
        req = GetHoldRegs.Request()
        req.index, req.addr, req.count, req.val_type = self.modbus_id, addr, count, dtype
        return self.call_sync(self.client_node.cli_get_hold, req)

    # ---------------- 内部功能 ----------------
    def _get_status(self):
        try:
            resp = self.get_hold(REG_STATUS, 4, 'U16')
            # self.get_logger().info(f'resp: {resp.robot_return}')
            parts = [int(x.strip()) for x in resp.robot_return.strip('{}').split(',')]

            if len(parts) < 4:
                raise RuntimeError('寄存器长度不足')

            # 第 1 个寄存器（parts[0]）= 0x07D0：状态寄存器
            status_reg = parts[0]
            gACT = (status_reg >> 0) & 0x01
            gMODE = (status_reg >> 1) & 0x01
            gGTO = (status_reg >> 3) & 0x01
            gSTA = (status_reg >> 4) & 0x03
            gOBJ = (status_reg >> 6) & 0x03

            # 第 2 个寄存器（parts[1]）= 0x07D1：故障(低8) + 位置(高8)
            fault_pos = parts[1]
            position = (fault_pos >> 8) & 0xFF

            # 第 3 个寄存器（parts[2]）= 0x07D2：速度(低8) + 力(高8)
            speed_force = parts[2]
            force = (speed_force >> 8) & 0xFF
            speed = speed_force & 0xFF

            return {
                'enabled': bool(gACT),
                'moving': bool(gGTO),
                'system_status': gSTA,
                'object_status': ['MOVING', 'INNER_CONTACT', 'OUTER_CONTACT', 'NO_OBJECT'][gOBJ],
                'position': position,
                'force': force,
                'speed': speed
            }
        except Exception as e:
            return {'error': str(e)}

    def _wait_until_ready(self, timeout=5.0):
        start = time.time()
        while time.time() - start < timeout:
            st = self._get_status()
            if 'error' in st:
                return False
            if st['enabled'] and not st['moving'] and st['system_status'] == 3:
                return True
            time.sleep(0.1)
        self.get_logger().error('等待夹爪就绪超时')
        raise RuntimeError('等待夹爪就绪超时')

    def _apply_force(self, force):
        cmd = f'{{9, {65535}, {force << 8}}}'
        self.set_hold(REG_CONTROL, 3, cmd, 'U16')

    def _update_current_width(self):
        """读取寄存器并更新本地缓存的宽度值"""
        try:
            self._wait_until_ready()
            status = self._get_status()
            if 'error' in status:
                self.get_logger().warn(f'Failed to read width: {status["error"]}')
                return
            position = status['position']
            width_mm = Config.GRIPPER_SPEC['max_width'] * (1 - position / 255.0)
            self._current_width_mm = width_mm / 2000.0
            self.get_logger().info(f'Updated gripper width: {width_mm:.2f} mm')
        except Exception as e:
            self.get_logger().error(f'Failed to update width: {str(e)}')

    def _publish_status(self):
        try:
            # status = self._get_status()
            msg = Float64()
            msg.data = self._current_width_mm
            self.pub_status.publish(msg)
            # self.get_logger().info(f'Published width: {width_mm}')
        except Exception as e:
            self.get_logger().error(f'Publish failed: {str(e)}')

    # ---------------- 初始化 ----------------
    def auto_init(self):
        try:
            self.modbus_close()
            time.sleep(0.2)
            self.modbus_create()
            time.sleep(0.2)
            self.set_hold(REG_CONTROL, 1, '0', 'U16')
            time.sleep(0.2)
            self.set_hold(REG_CONTROL, 1, '1', 'U16')
            time.sleep(0.5)
            self.get_logger().info('Gripper auto-initialized')
        except Exception as e:
            self.get_logger().error(f'Auto-init failed: {e}')

    # ---------------- 对外服务 ----------------
    def cb_open(self, req, response):
        try:
            part_id = req.part_id
            if part_id not in Config.PART_CONFIG:
                raise ValueError(f'Invalid part_id: {part_id}')
            cfg = Config.PART_CONFIG[part_id]
            width_mm = cfg['open_width']

            position = int((Config.GRIPPER_SPEC['max_width'] - width_mm) *
                           (65535 / Config.GRIPPER_SPEC['max_width']))
            position = max(0, min(position, 65535))

            cmd = f'{{9, {position}, {(cfg["force"] << 8) | cfg["speed"]}}}'
            self.set_hold(REG_CONTROL, 3, cmd, 'U16')

            response.success = True
            response.message = f'Opened to {width_mm} mm for part {part_id}'
            self._update_current_width()  # 更新夹爪打开宽度
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def cb_close(self, _, response):
        try:
            self.set_hold(REG_CONTROL, 1, '3', 'U16')
            self._wait_until_ready()
            self.set_hold(REG_CONTROL, 1, '1035', 'U16')
            self._wait_until_ready()
            self._apply_force(240)
            self._wait_until_ready()
            self.set_hold(REG_CONTROL, 1, '3', 'U16')
            self._wait_until_ready()
            self.set_hold(REG_CONTROL, 1, '1035', 'U16')
            self._wait_until_ready()
            st = self._get_status()
            self.get_logger().info(f'_get_status: {st}')
            if st.get('object_status') == 'OUTER_CONTACT':
                response.success, response.message = True, '抓取到物体'
            else:
                response.success, response.message = True, '未抓取到物体'
            self._update_current_width()  # 更新夹爪打开宽度
        except Exception as e:
            response.success, response.message = False, str(e)
        return response

# ---------------- main ----------------
def main():
    rclpy.init()
    node = GripperServiceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.client_node.destroy_node()
        node.modbus_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
