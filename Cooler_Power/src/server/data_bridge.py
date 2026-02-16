#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
共享数据桥接层

在 Qt 桌面应用和 Web 服务器之间共享状态数据和控制指令。
使用线程安全的数据结构，支持双向通信。
"""

import time
import threading
import math
from dataclasses import dataclass, field
from typing import Optional, Dict, Any, List, Callable
from collections import deque


@dataclass
class DeviceState:
    """设备全局状态快照"""

    # 电源连接与状态
    power_connected: bool = False
    power_port: str = ""
    power_baudrate: int = 9600
    power_address: int = 1

    # 电源实时数据
    voltage_real: float = 0.0
    current_real: float = 0.0
    power_real: float = 0.0
    voltage_set: float = 0.0
    current_set: float = 0.0
    is_output_on: bool = False
    power_mode: str = "未知"         # CV / CC / 未知
    protection_status: str = "正常"
    power_temperature: float = 0.0

    # 温度传感器 1
    temp1_connected: bool = False
    temp1_port: str = ""
    temp1_ds18b20: str = "--"
    temp1_hot: str = "--"
    temp1_cold: str = "--"

    # 温度传感器 2
    temp2_connected: bool = False
    temp2_port: str = ""
    temp2_ds18b20: str = "--"
    temp2_hot: str = "--"
    temp2_cold: str = "--"

    # PID 温度控制
    pid_enabled: bool = False
    pid_auto_tuning: bool = False
    target_temp: float = 15.0
    safety_temp: float = 30.0
    kp: float = 1.0
    ki: float = 0.05
    kd: float = 0.5
    max_current: float = 7.0
    max_voltage: float = 12.0
    control_interval: float = 1.0
    fusion_mode: int = 0           # 0=双平均, 1=仅1, 2=仅2
    control_mode: int = 0          # 0=制冷, 1=制热

    # PID 运行状态
    fused_temp: Optional[float] = None
    pid_output: float = 0.0
    temp_error: float = 0.0
    control_elapsed: float = 0.0
    auto_tune_message: str = ""

    # 时间戳
    updated_at: float = 0.0

    def to_dict(self) -> dict:
        """转为JSON可序列化字典"""
        return {
            'power_connected': self.power_connected,
            'power_port': self.power_port,
            'power_baudrate': self.power_baudrate,
            'power_address': self.power_address,
            'voltage_real': round(self.voltage_real, 3),
            'current_real': round(self.current_real, 3),
            'power_real': round(self.power_real, 2),
            'voltage_set': round(self.voltage_set, 3),
            'current_set': round(self.current_set, 3),
            'is_output_on': self.is_output_on,
            'power_mode': self.power_mode,
            'protection_status': self.protection_status,
            'power_temperature': round(self.power_temperature, 1),
            'temp1_connected': self.temp1_connected,
            'temp1_port': self.temp1_port,
            'temp1_ds18b20': self.temp1_ds18b20,
            'temp1_hot': self.temp1_hot,
            'temp1_cold': self.temp1_cold,
            'temp2_connected': self.temp2_connected,
            'temp2_port': self.temp2_port,
            'temp2_ds18b20': self.temp2_ds18b20,
            'temp2_hot': self.temp2_hot,
            'temp2_cold': self.temp2_cold,
            'pid_enabled': self.pid_enabled,
            'pid_auto_tuning': self.pid_auto_tuning,
            'target_temp': round(self.target_temp, 2),
            'safety_temp': round(self.safety_temp, 1),
            'kp': round(self.kp, 3),
            'ki': round(self.ki, 4),
            'kd': round(self.kd, 3),
            'max_current': round(self.max_current, 3),
            'max_voltage': round(self.max_voltage, 1),
            'control_interval': round(self.control_interval, 1),
            'fusion_mode': self.fusion_mode,
            'control_mode': self.control_mode,
            'fused_temp': round(self.fused_temp, 2) if self.fused_temp is not None else None,
            'pid_output': round(self.pid_output, 3),
            'temp_error': round(self.temp_error, 2),
            'control_elapsed': round(self.control_elapsed, 1),
            'auto_tune_message': self.auto_tune_message,
            'updated_at': self.updated_at,
        }


class DataBridge:
    """
    线程安全的数据桥接层

    Qt 主线程写入状态 → Web 服务器读取
    Web 服务器发送指令 → Qt 主线程执行
    """

    def __init__(self, history_max: int = 300):
        self._lock = threading.Lock()
        self._state = DeviceState()
        self._history_max = history_max

        # 温度/电流历史数据（用于 Web 端绘图）
        self._time_history: deque = deque(maxlen=history_max)
        self._target_history: deque = deque(maxlen=history_max)
        self._cold1_history: deque = deque(maxlen=history_max)
        self._hot1_history: deque = deque(maxlen=history_max)
        self._cold2_history: deque = deque(maxlen=history_max)
        self._hot2_history: deque = deque(maxlen=history_max)
        self._output_history: deque = deque(maxlen=history_max)

        # 指令队列：Web → Qt
        self._command_queue: deque = deque(maxlen=100)

        # 可用串口列表缓存
        self._available_ports: List[Dict[str, str]] = []

    # ---- 状态更新 (Qt → Web) ----

    def update_state(self, **kwargs):
        """Qt线程调用：更新设备状态"""
        with self._lock:
            for k, v in kwargs.items():
                if hasattr(self._state, k):
                    setattr(self._state, k, v)
            self._state.updated_at = time.time()

    def get_state(self) -> dict:
        """Web线程调用：获取设备状态快照"""
        with self._lock:
            return self._state.to_dict()

    # ---- 历史数据 (Qt → Web) ----

    def append_history(self, t: float, target: float,
                       cold1: float, hot1: float,
                       cold2: float, hot2: float,
                       output: float):
        """Qt线程调用：追加历史数据点"""
        with self._lock:
            self._time_history.append(t)
            self._target_history.append(target)
            self._cold1_history.append(cold1)
            self._hot1_history.append(hot1)
            self._cold2_history.append(cold2)
            self._hot2_history.append(hot2)
            self._output_history.append(output)

    def get_history(self) -> dict:
        """Web线程调用：获取历史数据"""
        def _clean(values):
            cleaned = []
            for v in values:
                if isinstance(v, (int, float)) and not math.isfinite(v):
                    cleaned.append(None)
                else:
                    cleaned.append(v)
            return cleaned

        with self._lock:
            return {
                'time': _clean(self._time_history),
                'target': _clean(self._target_history),
                'cold1': _clean(self._cold1_history),
                'hot1': _clean(self._hot1_history),
                'cold2': _clean(self._cold2_history),
                'hot2': _clean(self._hot2_history),
                'output': _clean(self._output_history),
            }

    # ---- 串口列表 ----

    def update_ports(self, ports: List[Dict[str, str]]):
        """更新可用串口列表"""
        with self._lock:
            self._available_ports = ports

    def get_ports(self) -> List[Dict[str, str]]:
        with self._lock:
            return list(self._available_ports)

    # ---- 指令队列 (Web → Qt) ----

    def send_command(self, cmd: str, params: dict = None):
        """Web线程调用：发送控制指令"""
        with self._lock:
            self._command_queue.append({
                'cmd': cmd,
                'params': params or {},
                'timestamp': time.time()
            })

    def poll_commands(self) -> List[dict]:
        """Qt线程调用：取出所有待执行指令"""
        with self._lock:
            cmds = list(self._command_queue)
            self._command_queue.clear()
            return cmds


# 全局单例
_bridge_instance: Optional[DataBridge] = None
_bridge_lock = threading.Lock()


def get_bridge() -> DataBridge:
    """获取全局 DataBridge 单例"""
    global _bridge_instance
    if _bridge_instance is None:
        with _bridge_lock:
            if _bridge_instance is None:
                _bridge_instance = DataBridge()
    return _bridge_instance
