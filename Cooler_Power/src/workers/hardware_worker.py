#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
硬件 I/O 工作者线程

所有串口通信（电源 Modbus RTU、温度传感器）在此工作者的 QThread 中运行，
确保 UI 主线程永远不被阻塞。

通过 Qt Signal/Slot 机制与 UI 双向通信：
  UI → Worker: 请求信号 (connect_power, set_voltage, ...)
  Worker → UI: 结果信号 (power_connect_result, power_status_updated, ...)

作者: AI协作团队
日期: 2026-02-15
"""

import re
import time
import math
import threading
from typing import Optional, List, Dict

import serial
import serial.tools.list_ports

from PySide6.QtCore import QObject, QTimer, Signal, Slot

try:
    from ..drivers.cl500w_driver import CL500WDriver
    from ..protocol.power_supply_base import PowerStatus, PowerMode, ProtectionStatus
    from ..server.data_bridge import get_bridge
    from ..pid_controller import PIDController, PIDAutoTuner
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent.parent))
    from src.drivers.cl500w_driver import CL500WDriver
    from src.protocol.power_supply_base import PowerStatus, PowerMode, ProtectionStatus
    from src.server.data_bridge import get_bridge
    from src.pid_controller import PIDController, PIDAutoTuner


class HardwareWorker(QObject):
    """
    硬件 I/O 工作者 — 运行在专用 QThread 中。

    职责：
    - 电源串口通信 (Modbus RTU)
    - 温度传感器串口读取
    - PID 温度闭环控制
    - DataBridge 状态同步 (供 Web 服务器)
    - Web 指令处理

    所有串口 I/O 都在此线程完成，UI 线程零阻塞。
    """

    # ====================== 信号: Worker → UI ======================

    # 电源连接
    power_connect_result = Signal(bool, str)      # success, message
    power_disconnected = Signal()

    # 电源状态轮询
    power_status_updated = Signal(object)         # PowerStatus
    poll_error_occurred = Signal(str)

    # 电源一次性指令结果
    set_voltage_result = Signal(bool)
    set_current_result = Signal(bool)
    output_on_result = Signal(bool)
    output_off_result = Signal(bool)

    # 温度传感器
    temp_connect_result = Signal(int, bool, str)  # index, success, message
    temp_disconnected_sig = Signal(int)
    temp_data_updated = Signal(int, dict)         # index, {ds18b20, hot, cold}

    # PID 控制
    control_start_result = Signal(bool, str)      # success, message
    control_stopped_sig = Signal()
    control_status_sig = Signal(dict)             # {measured, target, error, output, elapsed}

    # 自动整定
    auto_tune_start_result = Signal(bool, str)
    auto_tune_msg_sig = Signal(str)
    auto_tune_done_sig = Signal(float, float, float, str)  # kp, ki, kd, message
    auto_tune_failed_sig = Signal(str)

    # 安全保护
    safety_triggered_sig = Signal(float, float, int)  # measured, limit, count
    safety_recovered_sig = Signal(float)              # measured

    # 串口列表
    ports_refreshed_sig = Signal(list)

    # 图表数据 (1s)
    chart_data_sig = Signal(dict)

    # Web 桥接 → UI 参数同步
    bridge_params_sig = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)

        # ---- 电源驱动 ----
        self.power: Optional[CL500WDriver] = None
        self._power_port = ""
        self._power_baudrate = 9600
        self._power_address = 1

        # ---- 温度串口 ----
        self._temp_serial_1: Optional[serial.Serial] = None
        self._temp_serial_2: Optional[serial.Serial] = None
        self._temp_thread_1: Optional[threading.Thread] = None
        self._temp_thread_2: Optional[threading.Thread] = None
        self._temp_running_1 = False
        self._temp_running_2 = False
        self._temp_data_1 = {'ds18b20': '--', 'hot': '--', 'cold': '--'}
        self._temp_data_2 = {'ds18b20': '--', 'hot': '--', 'cold': '--'}

        # ---- PID 控制器 ----
        self._pid = PIDController()
        self._pid_enabled = False
        self._auto_tuning = False
        self._auto_tuner: Optional[PIDAutoTuner] = None
        self._safety_triggered = False
        self._safety_stop_count = 0
        self._control_start_time = 0.0

        # ---- 控制参数 (由 UI 通过 update_params 更新) ----
        self._target_temp = 15.0
        self._safety_temp = 30.0
        self._kp = 1.0
        self._ki = 0.05
        self._kd = 0.5
        self._max_current = 7.0
        self._max_voltage = 12.0
        self._control_interval = 1.0
        self._fusion_mode = 0   # 0=双传感器平均, 1=仅1, 2=仅2
        self._control_mode = 0  # 0=制冷, 1=制热

        # ---- DataBridge ----
        self._bridge = get_bridge()
        self._auto_tune_msg = ""

        # ---- 图表计数器 ----
        self._chart_counter = 0

        # ---- 定时器 (在 startup 中创建) ----
        self._poll_timer: Optional[QTimer] = None
        self._control_timer: Optional[QTimer] = None
        self._bridge_sync_timer: Optional[QTimer] = None
        self._bridge_cmd_timer: Optional[QTimer] = None
        self._temp_emit_timer: Optional[QTimer] = None
        self._chart_timer: Optional[QTimer] = None
        self._safety_recovery_timer: Optional[QTimer] = None

    # ====================== 生命周期 ======================

    @Slot()
    def startup(self):
        """线程启动后调用 — 创建 QTimer (必须在工作者线程中创建)"""

        # 状态轮询 (500ms, 连接后启动)
        self._poll_timer = QTimer()
        self._poll_timer.setInterval(500)
        self._poll_timer.timeout.connect(self._poll_status)

        # PID 控制 (可变间隔, 启动控制时开始)
        self._control_timer = QTimer()
        self._control_timer.timeout.connect(self._control_loop)

        # DataBridge 同步 (500ms)
        self._bridge_sync_timer = QTimer()
        self._bridge_sync_timer.setInterval(500)
        self._bridge_sync_timer.timeout.connect(self._sync_to_bridge)
        self._bridge_sync_timer.start()

        # Web 指令处理 (200ms)
        self._bridge_cmd_timer = QTimer()
        self._bridge_cmd_timer.setInterval(200)
        self._bridge_cmd_timer.timeout.connect(self._process_bridge_commands)
        self._bridge_cmd_timer.start()

        # 温度数据推送 (100ms)
        self._temp_emit_timer = QTimer()
        self._temp_emit_timer.setInterval(100)
        self._temp_emit_timer.timeout.connect(self._emit_temp_data)
        self._temp_emit_timer.start()

        # 图表数据 (1s)
        self._chart_timer = QTimer()
        self._chart_timer.setInterval(1000)
        self._chart_timer.timeout.connect(self._emit_chart_data)
        self._chart_timer.start()

        # 安全恢复检查 (启动后按需开启)
        self._safety_recovery_timer = QTimer()
        self._safety_recovery_timer.setInterval(2000)
        self._safety_recovery_timer.timeout.connect(self._check_safety_recovery)

    @Slot()
    def shutdown(self):
        """线程退出前清理"""
        for timer in [self._poll_timer, self._control_timer, self._bridge_sync_timer,
                      self._bridge_cmd_timer, self._temp_emit_timer, self._chart_timer,
                      self._safety_recovery_timer]:
            if timer:
                timer.stop()

        self._do_stop_control()
        self._do_disconnect_power()
        self._do_disconnect_temp(1)
        self._do_disconnect_temp(2)

    # ====================== 电源连接 ======================

    @Slot(str, int, int)
    def connect_power(self, port: str, baudrate: int, address: int):
        """连接电源 (工作者线程执行,不阻塞 UI)"""
        try:
            self._power_port = port
            self._power_baudrate = baudrate
            self._power_address = address

            self.power = CL500WDriver(
                port=port, slave_address=address, baudrate=baudrate)

            if self.power.connect():
                self._poll_timer.start()
                self.power_connect_result.emit(True, f"已连接到 {port}")
            else:
                self.power = None
                self.power_connect_result.emit(False, "连接失败，请检查串口和电源")
        except Exception as e:
            self.power = None
            self.power_connect_result.emit(False, f"连接异常: {e}")

    @Slot()
    def disconnect_power(self):
        """断开电源"""
        if self._pid_enabled:
            self._do_stop_control()
        self._do_disconnect_power()
        self.power_disconnected.emit()

    def _do_disconnect_power(self):
        if self._poll_timer:
            self._poll_timer.stop()
        if self.power:
            try:
                self.power.disconnect()
            except Exception:
                pass
            self.power = None

    # ====================== 电源指令 ======================

    @Slot(float)
    def set_voltage(self, voltage: float):
        if self.power and self.power.is_connected:
            result = self.power.set_voltage(voltage)
            self.set_voltage_result.emit(result)
        else:
            self.set_voltage_result.emit(False)

    @Slot(float)
    def set_current(self, current: float):
        if self.power and self.power.is_connected:
            result = self.power.set_current(current)
            self.set_current_result.emit(result)
        else:
            self.set_current_result.emit(False)

    @Slot()
    def output_on(self):
        if self.power and self.power.is_connected:
            result = self.power.output_on()
            self.output_on_result.emit(result)
        else:
            self.output_on_result.emit(False)

    @Slot()
    def output_off(self):
        if self.power and self.power.is_connected:
            result = self.power.output_off()
            self.output_off_result.emit(result)
        else:
            self.output_off_result.emit(False)

    # ====================== 状态轮询 ======================

    def _poll_status(self):
        """定时器回调: 轮询电源状态 (工作者线程,不阻塞 UI)"""
        if not self.power or not self.power.is_connected:
            return
        try:
            status = self.power.get_status()
            self.power_status_updated.emit(status)
        except Exception as e:
            self.poll_error_occurred.emit(str(e))

    # ====================== 温度传感器 ======================

    @Slot(int, str)
    def connect_temp(self, index: int, port: str):
        """连接温度传感器串口"""
        try:
            ser = serial.Serial(port, 115200, timeout=0.1)
            if index == 1:
                self._temp_serial_1 = ser
                self._temp_running_1 = True
                self._temp_thread_1 = threading.Thread(
                    target=self._read_temp_data, args=(1,), daemon=True)
                self._temp_thread_1.start()
            else:
                self._temp_serial_2 = ser
                self._temp_running_2 = True
                self._temp_thread_2 = threading.Thread(
                    target=self._read_temp_data, args=(2,), daemon=True)
                self._temp_thread_2.start()
            self.temp_connect_result.emit(index, True, f"传感器 {index} 已连接")
        except Exception as e:
            self.temp_connect_result.emit(index, False, f"连接失败: {e}")

    @Slot(int)
    def disconnect_temp(self, index: int):
        """断开温度传感器"""
        self._do_disconnect_temp(index)
        self.temp_disconnected_sig.emit(index)

    def _do_disconnect_temp(self, index: int):
        if index == 1:
            self._temp_running_1 = False
            if self._temp_serial_1:
                try:
                    self._temp_serial_1.close()
                except Exception:
                    pass
                self._temp_serial_1 = None
            self._temp_data_1 = {'ds18b20': '--', 'hot': '--', 'cold': '--'}
        else:
            self._temp_running_2 = False
            if self._temp_serial_2:
                try:
                    self._temp_serial_2.close()
                except Exception:
                    pass
                self._temp_serial_2 = None
            self._temp_data_2 = {'ds18b20': '--', 'hot': '--', 'cold': '--'}

    def _read_temp_data(self, index: int):
        """
        后台子线程：字节缓冲区方式读取温度串口数据。
        按 b'\\n' 分割完整行后再解码，避免 UTF-8 多字节截断。
        """
        ser = self._temp_serial_1 if index == 1 else self._temp_serial_2
        byte_buffer = b""

        while (self._temp_running_1 if index == 1 else self._temp_running_2):
            try:
                if not (ser and ser.is_open):
                    time.sleep(0.1)
                    continue

                waiting = ser.in_waiting
                if waiting > 0:
                    byte_buffer += ser.read(waiting)
                else:
                    chunk = ser.read(1)
                    if chunk:
                        byte_buffer += chunk
                    continue

                while b'\n' in byte_buffer:
                    line_bytes, byte_buffer = byte_buffer.split(b'\n', 1)
                    try:
                        line = line_bytes.decode('utf-8').strip()
                    except UnicodeDecodeError:
                        try:
                            line = line_bytes.decode('gbk').strip()
                        except Exception:
                            line = line_bytes.decode('utf-8', errors='replace').strip()
                    if line:
                        self._parse_temp_line(index, line)

            except serial.SerialException as e:
                print(f"温度串口异常 (传感器{index}): {e}")
                break
            except Exception as e:
                print(f"温度读取错误 (传感器{index}): {e}")
                time.sleep(0.05)

    def _parse_temp_line(self, index: int, line: str):
        """解析一行温度数据"""
        data = self._temp_data_1 if index == 1 else self._temp_data_2

        m = re.search(r'DS18B20\s*温度[：:]\s*([\d.]+)', line)
        if m:
            data['ds18b20'] = m.group(1)
            return

        m = re.search(r'热端温度[：:]\s*([\d.]+)', line)
        if m:
            data['hot'] = m.group(1)
            return

        m = re.search(r'冷端温度[：:]\s*([\d.]+)', line)
        if m:
            data['cold'] = m.group(1)
            return

    def _emit_temp_data(self):
        """100ms 定时器: 向 UI 推送温度数据"""
        self.temp_data_updated.emit(1, dict(self._temp_data_1))
        self.temp_data_updated.emit(2, dict(self._temp_data_2))

    # ====================== 传感器融合 ======================

    def _get_fused_temperature(self) -> Optional[float]:
        """
        传感器融合: 优先级 热端 > DS18B20 > 冷端。
        多传感器时取平均。
        """
        d1 = self._temp_data_1
        d2 = self._temp_data_2

        s1_connected = self._temp_running_1 and self._temp_serial_1 is not None
        s2_connected = self._temp_running_2 and self._temp_serial_2 is not None

        if self._fusion_mode == 1:
            sources = [(d1, 1)] if s1_connected else []
        elif self._fusion_mode == 2:
            sources = [(d2, 2)] if s2_connected else []
        else:
            sources = []
            if s1_connected:
                sources.append((d1, 1))
            if s2_connected:
                sources.append((d2, 2))

        if not sources:
            return None

        temp_priority = [('hot', '热端'), ('ds18b20', 'DS18B20'), ('cold', '冷端')]

        for temp_key, _ in temp_priority:
            values = []
            for src_data, _ in sources:
                val_str = src_data.get(temp_key, '--')
                if val_str != '--':
                    try:
                        values.append(float(val_str))
                    except ValueError:
                        pass
            if values:
                return sum(values) / len(values)

        return None

    # ====================== PID 温度控制 ======================

    @Slot(dict)
    def start_control(self, params: dict):
        """启动 PID 温度控制"""
        if not self.power or not self.power.is_connected:
            self.control_start_result.emit(False, "请先连接电源，PID 将控制电源电流输出")
            return

        fused = self._get_fused_temperature()
        if fused is None:
            self.control_start_result.emit(False, "无可用温度数据，请先连接温度传感器")
            return

        # 应用参数
        self._apply_params(params)

        # 配置 PID
        self._pid.kp = self._kp
        self._pid.ki = self._ki
        self._pid.kd = self._kd
        self._pid.output_max = self._max_current
        self._pid.reverse = (self._control_mode == 1)
        self._pid.reset()

        # 设置电压确保 CC 模式
        try:
            self.power.set_voltage(self._max_voltage)
        except Exception as e:
            print(f"PID 设置电压失败: {e}")

        self._pid_enabled = True
        self._auto_tuning = False
        self._safety_triggered = False
        self._control_start_time = time.time()

        interval_ms = int(self._control_interval * 1000)
        self._control_timer.setInterval(interval_ms)
        self._control_timer.start()

        self.control_start_result.emit(True, "")

    @Slot()
    def stop_control(self):
        """停止 PID 温度控制"""
        self._do_stop_control()
        self.control_stopped_sig.emit()

    def _do_stop_control(self):
        """内部: 停止控制, 不发信号"""
        self._pid_enabled = False
        self._auto_tuning = False
        if self._control_timer:
            self._control_timer.stop()
        if self._safety_recovery_timer:
            self._safety_recovery_timer.stop()
        if self.power and self.power.is_connected:
            try:
                self.power.set_current(0.0)
            except Exception:
                pass

    def _control_loop(self):
        """PID 控制回路 (定时器回调, 工作者线程)"""
        if not self._pid_enabled:
            return

        measured = self._get_fused_temperature()
        if measured is None:
            return

        # ===== 安全检查 =====
        if measured > self._safety_temp:
            self._safety_stop(measured, self._safety_temp)
            return

        target = self._target_temp
        is_heating = (self._control_mode == 1)
        self._pid.reverse = is_heating

        # ===== 自动整定模式 =====
        if self._auto_tuning and self._auto_tuner:
            output = self._auto_tuner.step(measured)
            self._auto_tune_msg = self._auto_tuner.message
            self.auto_tune_msg_sig.emit(self._auto_tuner.message)

            if self._auto_tuner.state == PIDAutoTuner.State.DONE:
                self._on_auto_tune_done()
                return
            elif self._auto_tuner.state == PIDAutoTuner.State.FAILED:
                self._on_auto_tune_failed()
                return
        else:
            # ===== 正常 PID 控制 =====
            output = self._pid.compute(target, measured)

        # 发送电流指令到电源 (工作者线程, 不阻塞 UI)
        if self.power and self.power.is_connected:
            try:
                self.power.set_current(output)
            except Exception as e:
                print(f"PID 设置电流失败: {e}")

        # 向 UI 推送状态
        error = measured - target
        elapsed = time.time() - self._control_start_time
        self.control_status_sig.emit({
            'measured': measured,
            'target': target,
            'error': error,
            'output': output,
            'elapsed': elapsed,
            'is_auto_tuning': self._auto_tuning,
        })

    # ====================== 自动整定 ======================

    @Slot(dict)
    def start_auto_tune(self, params: dict):
        """启动 PID 自动整定"""
        if not self.power or not self.power.is_connected:
            self.auto_tune_start_result.emit(False, "请先连接电源")
            return

        fused = self._get_fused_temperature()
        if fused is None:
            self.auto_tune_start_result.emit(False, "无可用温度数据，请先连接温度传感器")
            return

        self._apply_params(params)

        is_heating = (self._control_mode == 1)
        self._auto_tuner = PIDAutoTuner(
            setpoint=self._target_temp,
            output_high=self._max_current,
            output_low=0.0,
            max_time=300,
            min_change=2.0,
            heating=is_heating
        )
        self._auto_tuner.start()

        try:
            self.power.set_voltage(self._max_voltage)
        except Exception as e:
            print(f"PID 设置电压失败: {e}")

        self._auto_tuning = True
        self._pid_enabled = True
        self._safety_triggered = False
        self._control_start_time = time.time()

        interval_ms = int(self._control_interval * 1000)
        self._control_timer.setInterval(interval_ms)
        self._control_timer.start()

        self.auto_tune_start_result.emit(True, "")

    def _on_auto_tune_done(self):
        """自动整定完成"""
        self._auto_tuning = False
        self._pid_enabled = False
        self._control_timer.stop()

        if self.power and self.power.is_connected:
            try:
                self.power.set_current(0.0)
            except Exception:
                pass

        kp = self._auto_tuner.kp
        ki = self._auto_tuner.ki
        kd = self._auto_tuner.kd
        msg = self._auto_tuner.message

        # 更新内部参数
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._auto_tune_msg = msg

        self.auto_tune_done_sig.emit(kp, ki, kd, msg)

    def _on_auto_tune_failed(self):
        """自动整定失败"""
        self._auto_tuning = False
        self._pid_enabled = False
        self._control_timer.stop()

        if self.power and self.power.is_connected:
            try:
                self.power.set_current(0.0)
            except Exception:
                pass

        # 新算法失败时也会算出回退参数
        if self._auto_tuner.state == PIDAutoTuner.State.DONE:
            self._on_auto_tune_done()
            return

        self._auto_tune_msg = self._auto_tuner.message
        self.auto_tune_failed_sig.emit(self._auto_tuner.message)

    @Slot()
    def apply_tune(self):
        """应用整定结果并自动启动控制"""
        if self._auto_tuner and self._auto_tuner.state == PIDAutoTuner.State.DONE:
            self._kp = self._auto_tuner.kp
            self._ki = self._auto_tuner.ki
            self._kd = self._auto_tuner.kd
            msg = f"✅ 已应用: Kp={self._kp:.3f}, Ki={self._ki:.4f}, Kd={self._kd:.3f}"
            self._auto_tune_msg = msg
            self.auto_tune_done_sig.emit(self._kp, self._ki, self._kd, msg)

            # 延迟启动控制
            QTimer.singleShot(500, self._auto_start_after_tune)

    def _auto_start_after_tune(self):
        """整定后自动启动"""
        self.start_control(self._get_current_params())

    # ====================== 安全保护 ======================

    def _safety_stop(self, measured: float, limit: float):
        """温度超限，紧急停止"""
        self._safety_triggered = True
        self._safety_stop_count += 1

        self._pid_enabled = False
        self._auto_tuning = False
        self._control_timer.stop()

        if self.power and self.power.is_connected:
            try:
                self.power.set_current(0.0)
            except Exception:
                pass

        self.safety_triggered_sig.emit(measured, limit, self._safety_stop_count)
        self._safety_recovery_timer.start()

    def _check_safety_recovery(self):
        """检查温度是否回落到安全范围"""
        measured = self._get_fused_temperature()
        if measured is None:
            return

        if measured < self._safety_temp - 2.0:
            self._safety_recovery_timer.stop()
            self._safety_triggered = False
            self.safety_recovered_sig.emit(measured)

            # 自动重新整定
            QTimer.singleShot(1000, self._auto_restart_tune)

    def _auto_restart_tune(self):
        """安全恢复后自动重启整定"""
        self.start_auto_tune(self._get_current_params())

    # ====================== 参数管理 ======================

    @Slot(dict)
    def update_params(self, params: dict):
        """从 UI 更新控制参数"""
        self._apply_params(params)

    def _apply_params(self, params: dict):
        if 'target_temp' in params:
            self._target_temp = params['target_temp']
        if 'safety_temp' in params:
            self._safety_temp = params['safety_temp']
        if 'kp' in params:
            self._kp = params['kp']
        if 'ki' in params:
            self._ki = params['ki']
        if 'kd' in params:
            self._kd = params['kd']
        if 'max_current' in params:
            self._max_current = params['max_current']
        if 'max_voltage' in params:
            self._max_voltage = params['max_voltage']
        if 'control_interval' in params:
            self._control_interval = params['control_interval']
        if 'fusion_mode' in params:
            self._fusion_mode = params['fusion_mode']
        if 'control_mode' in params:
            self._control_mode = params['control_mode']

    def _get_current_params(self) -> dict:
        return {
            'target_temp': self._target_temp,
            'safety_temp': self._safety_temp,
            'kp': self._kp,
            'ki': self._ki,
            'kd': self._kd,
            'max_current': self._max_current,
            'max_voltage': self._max_voltage,
            'control_interval': self._control_interval,
            'fusion_mode': self._fusion_mode,
            'control_mode': self._control_mode,
        }

    # ====================== 串口列表 ======================

    @Slot()
    def refresh_ports(self):
        """扫描可用串口 (可能耗时, 在工作者线程执行)"""
        ports = serial.tools.list_ports.comports()
        port_list = [{"device": p.device, "description": p.description,
                      "display": f"{p.device} - {p.description}"} for p in ports]
        self.ports_refreshed_sig.emit(port_list)

    # ====================== 图表数据 ======================

    def _emit_chart_data(self):
        """1s 定时器: 向 UI 推送图表数据点"""
        self._chart_counter += 1
        t = self._chart_counter

        def _safe_float(val):
            if val == '--':
                return float('nan')
            try:
                return float(val)
            except ValueError:
                return float('nan')

        d1 = self._temp_data_1
        d2 = self._temp_data_2

        point = {
            't': t,
            'target': self._target_temp,
            'cold1': _safe_float(d1['cold']),
            'hot1': _safe_float(d1['hot']),
            'cold2': _safe_float(d2['cold']),
            'hot2': _safe_float(d2['hot']),
            'output': self._pid._output if self._pid_enabled else float('nan'),
        }

        # 同步到 DataBridge (供 Web 图表)
        self._bridge.append_history(
            t, point['target'],
            point['cold1'], point['hot1'],
            point['cold2'], point['hot2'],
            point['output']
        )

        self.chart_data_sig.emit(point)

    # ====================== DataBridge 同步 ======================

    def _sync_to_bridge(self):
        """500ms: 将状态同步到 DataBridge 供 Web 端读取"""
        bridge = self._bridge

        power_connected = self.power is not None and self.power.is_connected

        bridge.update_state(
            power_connected=power_connected,
            power_port=self._power_port,
            power_baudrate=self._power_baudrate,
            power_address=self._power_address,
        )

        # 温度传感器
        bridge.update_state(
            temp1_connected=self._temp_running_1 and self._temp_serial_1 is not None,
            temp1_ds18b20=self._temp_data_1['ds18b20'],
            temp1_hot=self._temp_data_1['hot'],
            temp1_cold=self._temp_data_1['cold'],
            temp2_connected=self._temp_running_2 and self._temp_serial_2 is not None,
            temp2_ds18b20=self._temp_data_2['ds18b20'],
            temp2_hot=self._temp_data_2['hot'],
            temp2_cold=self._temp_data_2['cold'],
        )

        # PID 控制
        fused = self._get_fused_temperature()
        elapsed = time.time() - self._control_start_time if self._pid_enabled else 0

        bridge.update_state(
            pid_enabled=self._pid_enabled,
            pid_auto_tuning=self._auto_tuning,
            target_temp=self._target_temp,
            safety_temp=self._safety_temp,
            kp=self._kp,
            ki=self._ki,
            kd=self._kd,
            max_current=self._max_current,
            max_voltage=self._max_voltage,
            control_interval=self._control_interval,
            fusion_mode=self._fusion_mode,
            control_mode=self._control_mode,
            fused_temp=fused,
            pid_output=self._pid._output if self._pid_enabled else 0.0,
            temp_error=(fused - self._target_temp) if fused is not None else 0.0,
            control_elapsed=elapsed,
            auto_tune_message=self._auto_tune_msg,
        )

        # 串口列表 (这里不频繁扫描, 由 refresh_ports 按需更新)

    # ====================== Web 指令处理 ======================

    def _process_bridge_commands(self):
        """200ms: 处理 Web 端发来的指令"""
        cmds = self._bridge.poll_commands()
        for cmd_obj in cmds:
            cmd = cmd_obj.get('cmd', '')
            params = cmd_obj.get('params', {})
            try:
                self._execute_bridge_command(cmd, params)
            except Exception as e:
                print(f"[Bridge] 执行指令 '{cmd}' 出错: {e}")

    def _execute_bridge_command(self, cmd: str, params: dict):
        """执行来自 Web 端的单个指令"""

        if cmd == "refresh_ports":
            self.refresh_ports()

        elif cmd == "power_connect":
            port = params.get('port', '')
            baudrate = params.get('baudrate', 9600)
            address = params.get('address', 1)
            self.connect_power(port, baudrate, address)

        elif cmd == "power_disconnect":
            self.disconnect_power()

        elif cmd == "set_voltage":
            self.set_voltage(params.get('voltage', 0))

        elif cmd == "set_current":
            self.set_current(params.get('current', 0))

        elif cmd == "output_on":
            self.output_on()

        elif cmd == "output_off":
            self.output_off()

        elif cmd == "temp_connect":
            idx = params.get('index', 1)
            port = params.get('port', '')
            self.connect_temp(idx, port)

        elif cmd == "temp_disconnect":
            idx = params.get('index', 1)
            self.disconnect_temp(idx)

        elif cmd == "update_pid_params":
            self.update_params(params)
            # 同步给 UI 更新界面
            self.bridge_params_sig.emit(params)

        elif cmd == "start_control":
            self.start_control(self._get_current_params())

        elif cmd == "stop_control":
            self.stop_control()

        elif cmd == "start_auto_tune":
            self.start_auto_tune(self._get_current_params())

        elif cmd == "apply_tune":
            self.apply_tune()
