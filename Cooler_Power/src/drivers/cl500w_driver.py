#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CL-500W 可调电源驱动

实现 PowerSupplyBase 抽象类，提供 CL-500W 电源的具体控制功能。
此驱动使用 MODBUS RTU 协议与电源通信。

技术规格:
- 电压范围: 0-60V (0-30V/0-20A 或 0-60V/0-10A)
- 电流范围: 0-20A
- 分辨率: 1mV/1mA
- 通信: RS485 MODBUS RTU

作者: AI协作团队
日期: 2026-02-05
"""

from typing import Optional
import logging

from ..protocol.power_supply_base import (
    PowerSupplyBase,
    PowerStatus,
    PowerSpecification,
    PowerMode,
    ProtectionStatus
)
from .modbus_rtu import ModbusRTU

# 配置日志
logger = logging.getLogger(__name__)


class CL500WRegister:
    """CL-500W 寄存器地址定义"""
    
    # 只读寄存器 (功能码 0x03)
    VOLTAGE_REAL = 1024      # 实时电压 (mV)
    CURRENT_REAL = 1025      # 实时电流 (mA)
    CC_CV_STATUS = 1026      # CC/CV状态 (0=CV, 1=CC)
    TEMPERATURE = 1027       # 温度 (℃)
    OTP_STATUS = 1028        # 过温保护 (0=正常, 1=触发)
    
    # 读写寄存器 (功能码 0x03/0x06)
    VOLTAGE_SET = 1280       # 设定电压 (mV)
    CURRENT_SET = 1281       # 设定电流 (mA)
    OUTPUT_SWITCH = 1282     # 输出开关 (0=关, 1=开)
    DEVICE_ADDRESS = 1283    # 本机地址 (1-127)
    SAVE_SETTINGS = 1284     # 保存设置 (写255)


class CL500WDriver(PowerSupplyBase):
    """
    CL-500W 可调电源驱动
    
    继承 PowerSupplyBase，实现具体的 MODBUS RTU 通信。
    
    使用示例:
    ```python
    power = CL500WDriver(port="COM3")
    
    with power:  # 自动连接和断开
        power.set_voltage(24.0)
        power.set_current(5.0)
        power.output_on()
        
        status = power.get_status()
        print(f"输出: {status.voltage_real}V / {status.current_real}A")
    ```
    """
    
    def __init__(
        self,
        port: str,
        slave_address: int = 1,
        baudrate: int = 9600,
        timeout: float = 1.0
    ):
        """
        初始化 CL-500W 驱动
        
        Args:
            port: 串口号 (如 "COM3")
            slave_address: MODBUS 从站地址 (1-127)
            baudrate: 波特率 (默认 9600)
            timeout: 通信超时 (秒)
        """
        super().__init__()
        
        self._port = port
        self._slave_address = slave_address
        self._baudrate = baudrate
        self._timeout = timeout
        
        # MODBUS 通信实例
        self._modbus: Optional[ModbusRTU] = None
        
        # 电源规格
        self._spec = PowerSpecification(
            voltage_min=0.0,
            voltage_max=60.0,
            voltage_resolution=0.001,
            current_min=0.0,
            current_max=20.0,
            current_resolution=0.001,
            power_max=500.0,
            model="CL-500W",
            manufacturer="百思泰电子"
        )
    
    # ==================== 属性实现 ====================
    
    @property
    def specification(self) -> PowerSpecification:
        return self._spec
    
    @property
    def is_connected(self) -> bool:
        return self._modbus is not None and self._modbus.is_connected
    
    @property
    def port(self) -> str:
        return self._port
    
    @property
    def slave_address(self) -> int:
        return self._slave_address
    
    # ==================== 连接管理 ====================
    
    def connect(self) -> bool:
        """连接电源"""
        if self.is_connected:
            return True
        
        self._modbus = ModbusRTU(
            port=self._port,
            baudrate=self._baudrate,
            timeout=self._timeout
        )
        
        if self._modbus.connect():
            logger.info(f"CL-500W 已连接: {self._port}")
            return True
        
        self._modbus = None
        return False
    
    def disconnect(self) -> None:
        """断开连接"""
        if self._modbus:
            self._modbus.disconnect()
            self._modbus = None
            logger.info("CL-500W 已断开")
    
    # ==================== 底层读写 ====================
    
    def _read_registers(self, start_address: int, count: int, use_input_register: bool = False) -> Optional[list]:
        """
        读取寄存器
        
        Args:
            start_address: 起始地址
            count: 读取数量
            use_input_register: 是否使用功能码 0x04 (读输入寄存器)
            
        Returns:
            寄存器值列表或 None
        """
        if not self.is_connected:
            self._notify_error("未连接电源")
            return None
        
        if use_input_register:
            response = self._modbus.read_input_registers(
                self._slave_address,
                start_address,
                count
            )
        else:
            response = self._modbus.read_holding_registers(
                self._slave_address,
                start_address,
                count
            )
        
        if response is None:
            self._notify_error("通信超时")
            return None
        
        if response.is_error:
            self._notify_error(f"MODBUS错误: {response.error_code}")
            return None
        
        registers = response.registers
        logger.debug(f"读取寄存器 {start_address}: {registers}")
        return registers
    
    def _write_register(self, address: int, value: int) -> bool:
        """
        写入单个寄存器
        
        Args:
            address: 寄存器地址
            value: 写入值
            
        Returns:
            是否成功
        """
        if not self.is_connected:
            self._notify_error("未连接电源")
            return False
        
        success = self._modbus.write_single_register(
            self._slave_address,
            address,
            value
        )
        
        if not success:
            self._notify_error("写入失败")
        
        return success
    
    # ==================== 状态读取 ====================
    
    def get_status(self) -> PowerStatus:
        """获取电源完整状态"""
        status = PowerStatus(is_connected=self.is_connected)
        
        if not self.is_connected:
            status.error_message = "未连接"
            return status
        
        try:
            # 批量读取实时数据 (1024-1028, 5个寄存器) - 经测试可以工作
            # 返回格式: [电压, 电流, CC/CV, 温度, OTP]
            real_values = self._read_registers(CL500WRegister.VOLTAGE_REAL, 5)
            
            if real_values and len(real_values) >= 5:
                # 电压: 单位 mV (0.001V)
                status.voltage_real = real_values[0] / 1000.0
                # 电流: 单位 mA (0.001A)
                status.current_real = real_values[1] / 1000.0
                # CC/CV: 0=CV, 1=CC
                status.mode = PowerMode.CC if real_values[2] else PowerMode.CV
                # 温度: ℃
                status.temperature = float(real_values[3])
                # OTP: 过温保护
                if real_values[4]:
                    status.protection = ProtectionStatus.OTP
            
            # 读取输出开关状态 (1282)
            output_values = self._read_registers(CL500WRegister.OUTPUT_SWITCH, 1)
            if output_values and len(output_values) >= 1:
                status.is_output_on = bool(output_values[0])
            
            # 读取设定值 (单独读取，批量读1280-1282超时)
            vset_values = self._read_registers(CL500WRegister.VOLTAGE_SET, 1)
            if vset_values and len(vset_values) >= 1:
                status.voltage_set = vset_values[0] / 1000.0
            
            iset_values = self._read_registers(CL500WRegister.CURRENT_SET, 1)
            if iset_values and len(iset_values) >= 1:
                status.current_set = iset_values[0] / 1000.0
            
            status.is_connected = True
            
        except Exception as e:
            status.error_message = str(e)
            logger.error(f"读取状态失败: {e}")
        
        return status
    
    def get_voltage(self) -> Optional[float]:
        """获取实时电压"""
        values = self._read_registers(CL500WRegister.VOLTAGE_REAL, 1)
        if values and len(values) >= 1:
            return values[0] / 1000.0
        return None
    
    def get_current(self) -> Optional[float]:
        """获取实时电流"""
        values = self._read_registers(CL500WRegister.CURRENT_REAL, 1)
        if values and len(values) >= 1:
            return values[0] / 1000.0
        return None
    
    def get_temperature(self) -> Optional[float]:
        """获取温度"""
        values = self._read_registers(CL500WRegister.TEMPERATURE, 1)
        if values and len(values) >= 1:
            return float(values[0])
        return None
    
    # ==================== 参数设置 ====================
    
    def set_voltage(self, voltage: float) -> bool:
        """设置输出电压"""
        if not self.validate_voltage(voltage):
            self._notify_error(f"电压超出范围: {voltage}V")
            return False
        
        # V -> mV
        voltage_mv = int(voltage * 1000)
        return self._write_register(CL500WRegister.VOLTAGE_SET, voltage_mv)
    
    def set_current(self, current: float) -> bool:
        """设置输出电流限制（CV模式下为电流上限，CC模式下为恒定电流值）"""
        if not self.validate_current(current):
            self._notify_error(f"电流超出范围: {current}A")
            return False
        
        # A -> mA
        current_ma = int(current * 1000)
        logger.info(f"设置电流: {current}A -> {current_ma}mA, 寄存器地址: {CL500WRegister.CURRENT_SET}")
        result = self._write_register(CL500WRegister.CURRENT_SET, current_ma)
        logger.info(f"设置电流结果: {result}")
        return result
    
    # ==================== 输出控制 ====================
    
    def output_on(self) -> bool:
        """开启输出"""
        logger.info("尝试开启输出...")
        result = self._write_register(CL500WRegister.OUTPUT_SWITCH, 1)
        logger.info(f"开启输出结果: {result}")
        return result
    
    def output_off(self) -> bool:
        """关闭输出"""
        logger.info("尝试关闭输出...")
        result = self._write_register(CL500WRegister.OUTPUT_SWITCH, 0)
        logger.info(f"关闭输出结果: {result}")
        return result
    
    def set_output(self, on: bool) -> bool:
        """设置输出状态"""
        return self.output_on() if on else self.output_off()
    
    # ==================== 其他功能 ====================
    
    def save_settings(self) -> bool:
        """保存设置到存储器"""
        return self._write_register(CL500WRegister.SAVE_SETTINGS, 255)
    
    def set_device_address(self, address: int) -> bool:
        """
        设置设备 MODBUS 地址
        
        Args:
            address: 新地址 (1-127)
            
        Returns:
            是否成功
        """
        if not 1 <= address <= 127:
            self._notify_error("地址超出范围 (1-127)")
            return False
        
        return self._write_register(CL500WRegister.DEVICE_ADDRESS, address)


# ==================== 测试代码 ====================
if __name__ == "__main__":
    import serial.tools.list_ports
    
    print("=" * 60)
    print("CL-500W 驱动测试")
    print("=" * 60)
    
    # 列出可用串口
    print("\n可用串口:")
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(f"  {port.device}: {port.description}")
    
    if not ports:
        print("  (无可用串口)")
        exit(1)
    
    # 选择串口
    port_name = input("\n请输入串口号 (如 COM3): ").strip()
    
    # 创建驱动
    power = CL500WDriver(port=port_name, slave_address=1)
    
    print(f"\n连接电源: {port_name}")
    if power.connect():
        print("连接成功!")
        
        # 读取状态
        print("\n读取电源状态...")
        status = power.get_status()
        
        print(f"  实时电压: {status.voltage_real:.3f} V")
        print(f"  实时电流: {status.current_real:.3f} A")
        print(f"  设定电压: {status.voltage_set:.3f} V")
        print(f"  设定电流: {status.current_set:.3f} A")
        print(f"  输出状态: {'ON' if status.is_output_on else 'OFF'}")
        print(f"  工作模式: {status.mode.value}")
        print(f"  温度: {status.temperature:.1f} ℃")
        print(f"  保护状态: {status.protection.value}")
        
        # 断开连接
        power.disconnect()
        print("\n已断开连接")
    else:
        print("连接失败!")
