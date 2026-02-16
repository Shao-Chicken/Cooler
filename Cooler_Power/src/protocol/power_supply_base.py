#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
可调电源抽象基类 - 协议层

定义统一的电源控制接口，所有具体电源驱动都必须继承此基类。
这样设计的好处：
1. 上层UI代码只需要调用抽象接口，不需要关心具体电源型号
2. 新增电源型号只需要实现这个基类，不需要修改UI代码
3. 便于单元测试和模拟

作者: AI协作团队
日期: 2026-02-05
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Callable, List
import threading


class PowerMode(Enum):
    """电源工作模式"""
    CV = "恒压模式"  # Constant Voltage
    CC = "恒流模式"  # Constant Current
    UNKNOWN = "未知"


class ProtectionStatus(Enum):
    """保护状态"""
    NORMAL = "正常"
    OVP = "过压保护"      # Over Voltage Protection
    OCP = "过流保护"      # Over Current Protection
    OTP = "过温保护"      # Over Temperature Protection
    SCP = "短路保护"      # Short Circuit Protection


@dataclass
class PowerStatus:
    """
    电源状态数据类
    
    包含电源的所有状态信息，UI层通过此对象获取电源状态
    """
    # 实时测量值
    voltage_real: float = 0.0       # 实时电压 (V)
    current_real: float = 0.0       # 实时电流 (A)
    power_real: float = 0.0         # 实时功率 (W)
    
    # 设定值
    voltage_set: float = 0.0        # 设定电压 (V)
    current_set: float = 0.0        # 设定电流 (A)
    
    # 状态信息
    is_output_on: bool = False      # 输出开关状态
    mode: PowerMode = PowerMode.UNKNOWN  # 工作模式
    protection: ProtectionStatus = ProtectionStatus.NORMAL  # 保护状态
    
    # 其他信息
    temperature: float = 0.0        # 温度 (℃)
    is_connected: bool = False      # 连接状态
    
    # 错误信息
    error_message: str = ""         # 错误信息
    
    def __post_init__(self):
        """计算实时功率"""
        self.power_real = self.voltage_real * self.current_real
    
    @property
    def is_protection_triggered(self) -> bool:
        """是否触发了保护"""
        return self.protection != ProtectionStatus.NORMAL
    
    def to_dict(self) -> dict:
        """转换为字典"""
        return {
            'voltage_real': self.voltage_real,
            'current_real': self.current_real,
            'power_real': self.power_real,
            'voltage_set': self.voltage_set,
            'current_set': self.current_set,
            'is_output_on': self.is_output_on,
            'mode': self.mode.value,
            'protection': self.protection.value,
            'temperature': self.temperature,
            'is_connected': self.is_connected,
            'error_message': self.error_message,
        }


@dataclass  
class PowerSpecification:
    """
    电源规格参数
    
    定义电源的能力范围，用于UI验证输入
    """
    # 电压范围
    voltage_min: float = 0.0        # 最小电压 (V)
    voltage_max: float = 60.0       # 最大电压 (V)
    voltage_resolution: float = 0.001  # 电压分辨率 (V)
    
    # 电流范围
    current_min: float = 0.0        # 最小电流 (A)
    current_max: float = 20.0       # 最大电流 (A)
    current_resolution: float = 0.001  # 电流分辨率 (A)
    
    # 功率
    power_max: float = 500.0        # 最大功率 (W)
    
    # 设备信息
    model: str = ""                 # 型号
    manufacturer: str = ""          # 制造商


class PowerSupplyBase(ABC):
    """
    可调电源抽象基类
    
    所有具体电源驱动必须继承此类并实现所有抽象方法。
    UI层只需要调用此基类定义的接口。
    
    使用示例:
    ```python
    # 创建具体电源实例
    power = CL500WDriver(port="COM3")
    
    # 连接
    if power.connect():
        # 设置输出
        power.set_voltage(24.0)
        power.set_current(5.0)
        power.output_on()
        
        # 读取状态
        status = power.get_status()
        print(f"电压: {status.voltage_real}V")
        
        # 断开
        power.disconnect()
    ```
    """
    
    def __init__(self):
        self._status_callbacks: List[Callable[[PowerStatus], None]] = []
        self._error_callbacks: List[Callable[[str], None]] = []
        self._lock = threading.Lock()
    
    # ==================== 抽象属性 ====================
    
    @property
    @abstractmethod
    def specification(self) -> PowerSpecification:
        """
        获取电源规格参数
        
        Returns:
            PowerSpecification: 电源规格
        """
        pass
    
    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """
        检查是否已连接
        
        Returns:
            bool: 连接状态
        """
        pass
    
    # ==================== 连接管理 ====================
    
    @abstractmethod
    def connect(self) -> bool:
        """
        连接电源
        
        Returns:
            bool: 连接是否成功
        """
        pass
    
    @abstractmethod
    def disconnect(self) -> None:
        """断开连接"""
        pass
    
    # ==================== 状态读取 ====================
    
    @abstractmethod
    def get_status(self) -> PowerStatus:
        """
        获取电源完整状态
        
        Returns:
            PowerStatus: 电源状态对象
        """
        pass
    
    @abstractmethod
    def get_voltage(self) -> Optional[float]:
        """
        获取实时电压
        
        Returns:
            float: 电压值(V)，失败返回None
        """
        pass
    
    @abstractmethod
    def get_current(self) -> Optional[float]:
        """
        获取实时电流
        
        Returns:
            float: 电流值(A)，失败返回None
        """
        pass
    
    # ==================== 参数设置 ====================
    
    @abstractmethod
    def set_voltage(self, voltage: float) -> bool:
        """
        设置输出电压
        
        Args:
            voltage: 目标电压值 (V)
            
        Returns:
            bool: 设置是否成功
        """
        pass
    
    @abstractmethod
    def set_current(self, current: float) -> bool:
        """
        设置输出电流限制
        
        Args:
            current: 目标电流值 (A)
            
        Returns:
            bool: 设置是否成功
        """
        pass
    
    # ==================== 输出控制 ====================
    
    @abstractmethod
    def output_on(self) -> bool:
        """
        开启输出
        
        Returns:
            bool: 操作是否成功
        """
        pass
    
    @abstractmethod
    def output_off(self) -> bool:
        """
        关闭输出
        
        Returns:
            bool: 操作是否成功
        """
        pass
    
    @abstractmethod
    def set_output(self, on: bool) -> bool:
        """
        设置输出状态
        
        Args:
            on: True开启，False关闭
            
        Returns:
            bool: 操作是否成功
        """
        pass
    
    # ==================== 其他功能 ====================
    
    @abstractmethod
    def save_settings(self) -> bool:
        """
        保存当前设置到电源存储器
        
        Returns:
            bool: 保存是否成功
        """
        pass
    
    # ==================== 回调注册 ====================
    
    def register_status_callback(self, callback: Callable[[PowerStatus], None]) -> None:
        """
        注册状态更新回调
        
        Args:
            callback: 状态更新时调用的函数
        """
        with self._lock:
            self._status_callbacks.append(callback)
    
    def unregister_status_callback(self, callback: Callable[[PowerStatus], None]) -> None:
        """注销状态更新回调"""
        with self._lock:
            if callback in self._status_callbacks:
                self._status_callbacks.remove(callback)
    
    def register_error_callback(self, callback: Callable[[str], None]) -> None:
        """
        注册错误回调
        
        Args:
            callback: 发生错误时调用的函数
        """
        with self._lock:
            self._error_callbacks.append(callback)
    
    def _notify_status(self, status: PowerStatus) -> None:
        """通知所有状态回调"""
        with self._lock:
            for callback in self._status_callbacks:
                try:
                    callback(status)
                except Exception:
                    pass
    
    def _notify_error(self, message: str) -> None:
        """通知所有错误回调"""
        with self._lock:
            for callback in self._error_callbacks:
                try:
                    callback(message)
                except Exception:
                    pass
    
    # ==================== 辅助方法 ====================
    
    def validate_voltage(self, voltage: float) -> bool:
        """验证电压值是否在有效范围内"""
        spec = self.specification
        return spec.voltage_min <= voltage <= spec.voltage_max
    
    def validate_current(self, current: float) -> bool:
        """验证电流值是否在有效范围内"""
        spec = self.specification
        return spec.current_min <= current <= spec.current_max
    
    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()
        return False
