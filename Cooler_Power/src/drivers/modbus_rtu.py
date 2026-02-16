#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MODBUS RTU 协议实现

提供 MODBUS RTU 通信的底层实现，包括：
- CRC16 校验
- 帧构建和解析
- 串口通信
- 增强的错误处理和重试机制

此模块是通用的 MODBUS RTU 实现，可被任何 MODBUS 设备驱动使用。

作者: AI协作团队
日期: 2026-02-05
更新: 增强错误处理、添加重试延迟、串口重置功能
"""

import struct
import time
import threading
from typing import Optional, List, Tuple
from dataclasses import dataclass
from enum import IntEnum
import logging

try:
    import serial
except ImportError:
    serial = None
    print("警告: pyserial 未安装，请运行 pip install pyserial")


# 配置日志
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

# 常量定义
DEFAULT_INTER_RETRY_DELAY = 0.2  # 重试间隔 200ms
MAX_CONSECUTIVE_FAILURES = 5     # 连续失败此次数后重置串口
MIN_RESPONSE_LENGTH = 5          # MODBUS RTU 最小响应长度 (地址+功能码+数据长度+CRC)


class FunctionCode(IntEnum):
    """MODBUS 功能码"""
    READ_COILS = 0x01                    # 读线圈
    READ_DISCRETE_INPUTS = 0x02          # 读离散输入
    READ_HOLDING_REGISTERS = 0x03        # 读保持寄存器
    READ_INPUT_REGISTERS = 0x04          # 读输入寄存器
    WRITE_SINGLE_COIL = 0x05             # 写单个线圈
    WRITE_SINGLE_REGISTER = 0x06         # 写单个寄存器
    WRITE_MULTIPLE_COILS = 0x0F          # 写多个线圈
    WRITE_MULTIPLE_REGISTERS = 0x10      # 写多个寄存器


class ModbusError(Exception):
    """MODBUS 通信错误"""
    pass


class CRC16:
    """
    MODBUS CRC16 校验计算

    算法说明：
    1. 预置 16 位寄存器为 0xFFFF
    2. 把第一个数据字节与寄存器低 8 位异或
    3. 右移一位，检查移出位
    4. 移出位为 1 则与 0xA001 异或
    5. 重复步骤 3-4 共 8 次
    6. 对下一个字节重复步骤 2-5
    7. 最终结果低字节在前
    """

    # 预计算的 CRC 查表 (提高性能)
    _TABLE: List[int] = []

    @classmethod
    def _init_table(cls) -> None:
        """初始化 CRC 查表"""
        if cls._TABLE:
            return

        for i in range(256):
            crc = i
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
            cls._TABLE.append(crc)

    @classmethod
    def calculate(cls, data: bytes) -> int:
        """
        计算 CRC16 校验码

        Args:
            data: 待计算的数据

        Returns:
            16位 CRC 值
        """
        cls._init_table()

        crc = 0xFFFF
        for byte in data:
            crc = (crc >> 8) ^ cls._TABLE[(crc ^ byte) & 0xFF]
        return crc

    @classmethod
    def append(cls, data: bytes) -> bytes:
        """
        为数据追加 CRC16 (低字节在前)

        Args:
            data: 原始数据

        Returns:
            追加 CRC 后的完整数据帧
        """
        crc = cls.calculate(data)
        # MODBUS RTU: 低字节在前
        return data + struct.pack('<H', crc)

    @classmethod
    def verify(cls, data: bytes) -> bool:
        """
        验证数据帧的 CRC16

        Args:
            data: 包含 CRC 的完整帧 (至少 3 字节)

        Returns:
            校验是否通过
        """
        if len(data) < 3:
            return False

        payload = data[:-2]
        received_crc = struct.unpack('<H', data[-2:])[0]
        calculated_crc = cls.calculate(payload)
        
        return received_crc == calculated_crc


@dataclass
class ModbusResponse:
    """MODBUS 响应数据"""
    slave_address: int
    function_code: int
    data: bytes
    is_error: bool = False
    error_code: int = 0

    @property
    def registers(self) -> List[int]:
        """将数据解析为寄存器列表 (16位大端)"""
        result = []
        for i in range(0, len(self.data), 2):
            if i + 1 < len(self.data):
                value = struct.unpack('>H', self.data[i:i+2])[0]
                result.append(value)
        return result


class ModbusRTU:
    """
    MODBUS RTU 主站实现

    提供标准的 MODBUS RTU 通信功能，包含增强的错误处理机制。

    增强功能：
    - 重试间隔延迟 (避免连续快速重试)
    - 连续失败后自动重置串口
    - 更详细的错误日志
    - 改进的响应长度验证

    使用示例：
    ```python
    modbus = ModbusRTU(port="COM3", baudrate=9600)

    if modbus.connect():
        # 读取寄存器
        response = modbus.read_holding_registers(
            slave_address=1,
            start_address=1024,
            count=5
        )
        if response:
            print(f"寄存器值: {response.registers}")

        # 写入寄存器
        success = modbus.write_single_register(
            slave_address=1,
            address=1280,
            value=25000
        )

        modbus.disconnect()
    ```
    """

    # 帧间隔时间 (3.5 字符时间)
    FRAME_DELAY_MAP = {
        4800: 0.0073,
        9600: 0.00365,
        19200: 0.00183,
        38400: 0.00091,
        57600: 0.00061,
        115200: 0.00030,
    }

    def __init__(
        self,
        port: str,
        baudrate: int = 9600,
        timeout: float = 1.0,
        retry_count: int = 3,
        inter_retry_delay: float = DEFAULT_INTER_RETRY_DELAY
    ):
        """
        初始化 MODBUS RTU 主站

        Args:
            port: 串口号 (如 "COM3" 或 "/dev/ttyUSB0")
            baudrate: 波特率
            timeout: 通信超时 (秒)
            retry_count: 失败重试次数
            inter_retry_delay: 重试间隔时间 (秒)，默认 200ms
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.retry_count = retry_count
        self.inter_retry_delay = inter_retry_delay

        self._serial: Optional[serial.Serial] = None
        self._frame_delay = self.FRAME_DELAY_MAP.get(baudrate, 0.005)
        self._consecutive_failures = 0  # 连续失败计数
        self._io_lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._serial is not None and self._serial.is_open

    def connect(self) -> bool:
        """
        连接串口

        Returns:
            连接是否成功
        """
        if serial is None:
            logger.error("pyserial 未安装")
            return False

        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )

            # 等待串口稳定
            time.sleep(0.1)

            # 清空缓冲区
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

            logger.info(f"串口已连接: {self.port} @ {self.baudrate}bps")
            self._consecutive_failures = 0
            return True

        except Exception as e:
            logger.error(f"串口连接失败: {e}")
            self._serial = None
            return False

    def disconnect(self) -> None:
        """断开串口连接"""
        if self._serial and self._serial.is_open:
            self._serial.close()
            logger.info("串口已断开")
        self._serial = None
        self._consecutive_failures = 0

    def _reset_serial_port(self) -> bool:
        """
        重置串口 (关闭后重新打开)
        
        Returns:
            重置是否成功
        """
        logger.warning(f"由于连续 {self._consecutive_failures} 次失败，正在重置串口...")

        try:
            with self._io_lock:
                if self._serial and self._serial.is_open:
                    self._serial.close()

                time.sleep(1.0)  # 等待串口释放

                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout
                )

                time.sleep(0.1)
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()

                self._consecutive_failures = 0
                logger.info("串口重置成功")
                return True

        except Exception as e:
            logger.error(f"串口重置失败: {e}")
            return False

    def _send_frame(self, frame: bytes) -> bool:
        """
        发送数据帧

        Args:
            frame: 完整的 MODBUS 帧 (含 CRC)

        Returns:
            发送是否成功
        """
        if not self.is_connected:
            logger.error("串口未连接")
            return False

        try:
            # 发送前等待帧间隔
            time.sleep(self._frame_delay)

            # 清空输入缓冲区
            self._serial.reset_input_buffer()

            # 发送数据
            bytes_written = self._serial.write(frame)
            self._serial.flush()

            logger.debug(f"发送: {frame.hex().upper()} ({bytes_written} bytes)")
            return bytes_written == len(frame)

        except Exception as e:
            logger.error(f"发送失败: {e}")
            return False

    def _receive_frame(self, expected_length: int = 256) -> Optional[bytes]:
        """
        接收数据帧 (增强版：更好的响应长度验证)

        Args:
            expected_length: 期望的最大长度

        Returns:
            接收到的数据，失败返回 None
        """
        if not self.is_connected:
            return None

        try:
            # 等待响应 (帧间隔 + 额外等待)
            time.sleep(self._frame_delay * 2)

            # 等待最少 MIN_RESPONSE_LENGTH 字节数据到达
            start_time = time.time()
            while self._serial.in_waiting < MIN_RESPONSE_LENGTH:
                if time.time() - start_time > self.timeout:
                    logger.warning(f"接收超时: 仅收到 {self._serial.in_waiting} 字节，期望至少 {MIN_RESPONSE_LENGTH} 字节")
                    return None
                time.sleep(0.01)

            # 读取头部以确定完整帧长度
            header = self._serial.read(3)  # 地址 + 功能码 + 字节数/错误码
            if len(header) < 3:
                logger.warning(f"响应数据过短: 仅收到头部 {len(header)} 字节")
                return None

            # 根据功能码确定剩余长度
            func_code = header[1]
            if func_code & 0x80:
                # 错误响应: 地址(1) + 功能码(1) + 错误码(1) + CRC(2) = 5字节
                remaining = 2  # 还需读取 CRC
            else:
                # 正常响应: 根据字节数确定
                byte_count = header[2]
                remaining = byte_count + 2  # 数据 + CRC

            # 等待剩余数据
            wait_start = time.time()
            while self._serial.in_waiting < remaining:
                if time.time() - wait_start > self.timeout:
                    logger.warning(f"等待剩余数据超时: 期望 {remaining} 字节，仅有 {self._serial.in_waiting} 字节")
                    break
                time.sleep(0.01)

            # 读取剩余数据
            rest = self._serial.read(self._serial.in_waiting or remaining)
            response = header + rest

            if len(response) < MIN_RESPONSE_LENGTH:
                logger.warning(f"响应数据过短: 仅收到 {len(response)} 字节")
                return None

            logger.debug(f"接收: {response.hex().upper()} ({len(response)} bytes)")
            return response

        except Exception as e:
            logger.error(f"接收失败: {e}")
            return None

    def _transact(self, request: bytes) -> Optional[ModbusResponse]:
        """
        发送请求并接收响应 (增强版：带重试延迟和串口重置)

        Args:
            request: 不含 CRC 的请求数据

        Returns:
            ModbusResponse 或 None
        """
        # 添加 CRC
        frame = CRC16.append(request)

        with self._io_lock:
            for attempt in range(self.retry_count):
                # 发送
                if not self._send_frame(frame):
                    self._consecutive_failures += 1
                    if attempt < self.retry_count - 1:
                        logger.info(f"发送失败，{self.inter_retry_delay}秒后重试...")
                        time.sleep(self.inter_retry_delay)
                    continue

                # 接收
                response = self._receive_frame()

                if response is None:
                    self._consecutive_failures += 1
                    logger.warning(f"重试 {attempt + 1}/{self.retry_count}")

                    # 检查是否需要重置串口
                    if self._consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                        self._reset_serial_port()

                    if attempt < self.retry_count - 1:
                        time.sleep(self.inter_retry_delay)
                    continue

                # 验证 CRC
                if not CRC16.verify(response):
                    self._consecutive_failures += 1
                    logger.error(f"CRC 校验失败: {response.hex().upper()}")

                    if self._consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                        self._reset_serial_port()

                    if attempt < self.retry_count - 1:
                        time.sleep(self.inter_retry_delay)
                    continue

                # 解析响应
                if len(response) < 3:
                    logger.error("响应过短")
                    continue

                slave_addr = response[0]
                func_code = response[1]

                # 检查是否是错误响应
                if func_code & 0x80:
                    error_code = response[2] if len(response) > 2 else 0
                    self._consecutive_failures = 0  # 收到有效响应，重置计数
                    return ModbusResponse(
                        slave_address=slave_addr,
                        function_code=func_code & 0x7F,
                        data=b'',
                        is_error=True,
                        error_code=error_code
                    )

                # 正常响应
                data = response[2:-2]  # 去掉地址、功能码和 CRC
                self._consecutive_failures = 0  # 成功，重置失败计数
                return ModbusResponse(
                    slave_address=slave_addr,
                    function_code=func_code,
                    data=data
                )

        logger.error(f"通信失败: 已重试 {self.retry_count} 次")
        return None

    def read_holding_registers(
        self,
        slave_address: int,
        start_address: int,
        count: int
    ) -> Optional[ModbusResponse]:
        """
        读取保持寄存器 (功能码 0x03)

        Args:
            slave_address: 从站地址 (1-247)
            start_address: 起始寄存器地址
            count: 读取数量 (1-125)

        Returns:
            ModbusResponse 或 None
        """
        request = struct.pack(
            '>BBHH',
            slave_address,
            FunctionCode.READ_HOLDING_REGISTERS,
            start_address,
            count
        )

        response = self._transact(request)

        if response and not response.is_error:
            # 第一个字节是数据长度，后面是实际数据
            if len(response.data) > 0:
                byte_count = response.data[0]
                response.data = response.data[1:1+byte_count]

        return response

    def read_input_registers(
        self,
        slave_address: int,
        start_address: int,
        count: int
    ) -> Optional[ModbusResponse]:
        """
        读取输入寄存器 (功能码 0x04)

        某些设备的设定值寄存器需要使用此功能码读取。

        Args:
            slave_address: 从站地址 (1-247)
            start_address: 起始寄存器地址
            count: 读取数量 (1-125)

        Returns:
            ModbusResponse 或 None
        """
        request = struct.pack(
            '>BBHH',
            slave_address,
            FunctionCode.READ_INPUT_REGISTERS,
            start_address,
            count
        )

        response = self._transact(request)

        if response and not response.is_error:
            # 第一个字节是数据长度，后面是实际数据
            if len(response.data) > 0:
                byte_count = response.data[0]
                response.data = response.data[1:1+byte_count]

        return response

    def write_single_register(
        self,
        slave_address: int,
        address: int,
        value: int
    ) -> bool:
        """
        写单个寄存器 (功能码 0x06)

        Args:
            slave_address: 从站地址
            address: 寄存器地址
            value: 写入值 (0-65535)

        Returns:
            写入是否成功
        """
        request = struct.pack(
            '>BBHH',
            slave_address,
            FunctionCode.WRITE_SINGLE_REGISTER,
            address,
            value
        )

        response = self._transact(request)

        # 写入成功时，从站返回相同的请求
        return response is not None and not response.is_error


# ==================== 测试代码 ====================
if __name__ == "__main__":
    print("=" * 60)
    print("MODBUS RTU 协议模块测试")
    print("=" * 60)

    # 测试 CRC16
    print("\n[1] CRC16 测试:")
    
    # 使用标准 MODBUS 测试用例
    test_cases = [
        (bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x01]), "读单个寄存器"),
        (bytes([0x01, 0x06, 0x00, 0x01, 0x00, 0x03]), "写单个寄存器"),
    ]
    
    for test_data, desc in test_cases:
        crc = CRC16.calculate(test_data)
        frame = CRC16.append(test_data)
        print(f"    {desc}:")
        print(f"      数据: {test_data.hex().upper()}")
        print(f"      CRC:  {crc:04X}")
        print(f"      完整帧: {frame.hex().upper()}")
        print(f"      验证: {'通过' if CRC16.verify(frame) else '失败'}")

    # 测试帧构建
    print("\n[2] 帧构建测试:")
    modbus = ModbusRTU("COM1")

    # 读取寄存器请求帧
    read_req = struct.pack('>BBHH', 1, 0x03, 1024, 1)
    read_frame = CRC16.append(read_req)
    print(f"    读取电压请求: {read_frame.hex().upper()}")

    # 写入寄存器请求帧
    write_req = struct.pack('>BBHH', 1, 0x06, 1280, 25000)
    write_frame = CRC16.append(write_req)
    print(f"    设置电压25V:  {write_frame.hex().upper()}")

    print("\n[3] 增强功能说明:")
    print(f"    - 默认重试间隔: {DEFAULT_INTER_RETRY_DELAY} 秒")
    print(f"    - 连续失败重置阈值: {MAX_CONSECUTIVE_FAILURES} 次")
    print(f"    - 最小响应长度: {MIN_RESPONSE_LENGTH} 字节")
