#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
单元测试 - CL-500W 驱动和 MODBUS 协议

运行测试:
    python -m pytest tests/test_driver.py -v
    
或直接运行:
    python tests/test_driver.py
"""

import sys
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

import unittest
from unittest.mock import Mock, patch, MagicMock

from src.drivers.modbus_rtu import CRC16, ModbusRTU, ModbusResponse
from src.drivers.cl500w_driver import CL500WDriver, CL500WRegister
from src.protocol.power_supply_base import PowerSupplyBase, PowerStatus, PowerMode


class TestCRC16(unittest.TestCase):
    """CRC16 测试"""
    
    def test_crc_calculation(self):
        """测试 CRC 计算"""
        # 测试数据: 01 03 04 00 01 00 02
        data = bytes([0x01, 0x03, 0x04, 0x00, 0x01, 0x00, 0x02])
        crc = CRC16.calculate(data)
        
        # CRC 应该是 16 位整数
        self.assertIsInstance(crc, int)
        self.assertGreaterEqual(crc, 0)
        self.assertLessEqual(crc, 0xFFFF)
    
    def test_crc_append_and_verify(self):
        """测试 CRC 追加和验证"""
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01])
        
        # 追加 CRC
        data_with_crc = CRC16.append(data)
        self.assertEqual(len(data_with_crc), len(data) + 2)
        
        # 验证 CRC
        self.assertTrue(CRC16.verify(data_with_crc))
    
    def test_crc_verify_invalid(self):
        """测试无效 CRC 验证"""
        data = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF])
        self.assertFalse(CRC16.verify(data))


class TestModbusRTU(unittest.TestCase):
    """MODBUS RTU 测试"""
    
    def test_instance_creation(self):
        """测试实例创建"""
        modbus = ModbusRTU(port="COM1")
        
        self.assertEqual(modbus.port, "COM1")
        self.assertEqual(modbus.baudrate, 9600)
        self.assertFalse(modbus.is_connected)
    
    def test_default_parameters(self):
        """测试默认参数"""
        modbus = ModbusRTU(port="COM1", baudrate=19200, timeout=2.0)
        
        self.assertEqual(modbus.baudrate, 19200)
        self.assertEqual(modbus.timeout, 2.0)


class TestCL500WDriver(unittest.TestCase):
    """CL-500W 驱动测试"""
    
    def setUp(self):
        """测试准备"""
        self.driver = CL500WDriver(port="COM1", slave_address=1)
    
    def test_specification(self):
        """测试电源规格"""
        spec = self.driver.specification
        
        self.assertEqual(spec.voltage_max, 60.0)
        self.assertEqual(spec.current_max, 20.0)
        self.assertEqual(spec.power_max, 500.0)
        self.assertEqual(spec.model, "CL-500W")
    
    def test_validate_voltage(self):
        """测试电压验证"""
        self.assertTrue(self.driver.validate_voltage(0))
        self.assertTrue(self.driver.validate_voltage(30))
        self.assertTrue(self.driver.validate_voltage(60))
        
        self.assertFalse(self.driver.validate_voltage(-1))
        self.assertFalse(self.driver.validate_voltage(61))
    
    def test_validate_current(self):
        """测试电流验证"""
        self.assertTrue(self.driver.validate_current(0))
        self.assertTrue(self.driver.validate_current(10))
        self.assertTrue(self.driver.validate_current(20))
        
        self.assertFalse(self.driver.validate_current(-1))
        self.assertFalse(self.driver.validate_current(21))
    
    def test_register_addresses(self):
        """测试寄存器地址定义"""
        # 只读寄存器
        self.assertEqual(CL500WRegister.VOLTAGE_REAL, 1024)
        self.assertEqual(CL500WRegister.CURRENT_REAL, 1025)
        self.assertEqual(CL500WRegister.CC_CV_STATUS, 1026)
        self.assertEqual(CL500WRegister.TEMPERATURE, 1027)
        self.assertEqual(CL500WRegister.OTP_STATUS, 1028)
        
        # 读写寄存器
        self.assertEqual(CL500WRegister.VOLTAGE_SET, 1280)
        self.assertEqual(CL500WRegister.CURRENT_SET, 1281)
        self.assertEqual(CL500WRegister.OUTPUT_SWITCH, 1282)
    
    def test_not_connected_by_default(self):
        """测试默认未连接状态"""
        self.assertFalse(self.driver.is_connected)
    
    @patch('src.drivers.cl500w_driver.ModbusRTU')
    def test_connect_success(self, mock_modbus_class):
        """测试连接成功"""
        mock_modbus = Mock()
        mock_modbus.connect.return_value = True
        mock_modbus.is_connected = True
        mock_modbus_class.return_value = mock_modbus
        
        driver = CL500WDriver(port="COM1")
        result = driver.connect()
        
        self.assertTrue(result)
    
    @patch('src.drivers.cl500w_driver.ModbusRTU')
    def test_connect_failure(self, mock_modbus_class):
        """测试连接失败"""
        mock_modbus = Mock()
        mock_modbus.connect.return_value = False
        mock_modbus_class.return_value = mock_modbus
        
        driver = CL500WDriver(port="COM1")
        result = driver.connect()
        
        self.assertFalse(result)


class TestPowerSupplyBase(unittest.TestCase):
    """抽象基类测试"""
    
    def test_power_status_defaults(self):
        """测试 PowerStatus 默认值"""
        status = PowerStatus()
        
        self.assertFalse(status.is_connected)
        self.assertFalse(status.is_output_on)
        self.assertEqual(status.voltage_real, 0.0)
        self.assertEqual(status.current_real, 0.0)
        self.assertEqual(status.mode, PowerMode.UNKNOWN)
    
    def test_power_status_power_calculation(self):
        """测试功率计算 - 通过 __post_init__ 自动计算"""
        status = PowerStatus(
            voltage_real=24.0,
            current_real=5.0
        )
        
        # PowerStatus 使用 power_real 字段，在 __post_init__ 中计算
        self.assertEqual(status.power_real, 120.0)


class TestModbusResponse(unittest.TestCase):
    """MODBUS 响应测试"""
    
    def test_normal_response(self):
        """测试正常响应"""
        # 模拟 5 个寄存器的数据: 12000, 5000, 0, 35, 0
        import struct
        data = struct.pack('>HHHHH', 12000, 5000, 0, 35, 0)
        
        response = ModbusResponse(
            slave_address=1,
            function_code=3,
            data=data
        )
        
        self.assertFalse(response.is_error)
        self.assertEqual(len(response.registers), 5)
        self.assertEqual(response.registers[0], 12000)  # 12V
        self.assertEqual(response.registers[1], 5000)   # 5A
    
    def test_error_response(self):
        """测试错误响应"""
        response = ModbusResponse(
            slave_address=1,
            function_code=0x83,  # 0x03 | 0x80 = 错误
            data=bytes([2]),     # 错误码
            is_error=True,
            error_code=2
        )
        
        self.assertTrue(response.is_error)
        self.assertEqual(response.error_code, 2)


if __name__ == "__main__":
    print("=" * 60)
    print("CL-500W 驱动单元测试")
    print("=" * 60)
    
    unittest.main(verbosity=2)
