#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CL-500W 可调电源控制器 - 主界面 + 温度控制

支持:
- 电压/电流设定
- 输出开关控制
- 实时状态监控
- 串口连接管理
- 双路温度传感器采集
- PID 温度闭环控制（半导体制冷片）
- 实时温度曲线绘图

作者: AI协作团队
日期: 2026-02-05
更新: 2026-02-14 - 增加温度控制与曲线图
"""

import sys
import math
from pathlib import Path
from typing import Optional
from collections import deque

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QComboBox,
    QGridLayout, QFrame, QMessageBox, QSpinBox, QDoubleSpinBox,
    QProgressBar, QSlider, QStackedWidget, QScrollArea, QSizePolicy,
    QCheckBox, QAbstractSpinBox
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot, QRect, QThread, QMetaObject, Q_ARG
from PySide6.QtGui import QFont, QPalette, QColor, QIcon, QPainter, QPen, QBrush

import tomllib

# matplotlib 嵌入 Qt
import matplotlib
import logging
matplotlib.use('QtAgg')
matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False
logging.getLogger('matplotlib.font_manager').setLevel(logging.WARNING)
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# 导入驱动 & 工作者
try:
    from ..protocol.power_supply_base import PowerStatus, PowerMode, ProtectionStatus
    from ..workers.hardware_worker import HardwareWorker
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent.parent))
    from src.protocol.power_supply_base import PowerStatus, PowerMode, ProtectionStatus
    from src.workers.hardware_worker import HardwareWorker


# ==================== 样式定义 ====================

MODERN_STYLE = """
QMainWindow { background-color: #f0f0f0; }
QWidget { background-color: transparent; color: #333333;
    font-family: "Microsoft YaHei", "Segoe UI", Arial; font-size: 12px; }
QWidget#content_area { background-color: #ffffff; border-radius: 8px; }

QGroupBox { font-size: 13px; font-weight: bold; color: #262626;
    border: none; margin-top: 8px; padding-top: 8px; background-color: transparent; }
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left;
    left: 0px; padding: 0 4px; }

QLabel { color: #595959; font-size: 12px; background-color: transparent; }
QLabel#section_title { color: #262626; font-size: 14px; font-weight: bold; }

QLineEdit { background-color: #ffffff; border: 1px solid #d9d9d9;
    border-radius: 4px; padding: 6px 10px; color: #333333; font-size: 12px; }
QSpinBox, QDoubleSpinBox { background-color: #ffffff; border: 1px solid #d9d9d9;
    border-radius: 4px; padding: 4px; color: #333333; font-size: 12px; min-height: 22px; }
QLineEdit:focus, QSpinBox:focus, QDoubleSpinBox:focus { border: 1px solid #1890ff; }
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled {
    background-color: #f5f5f5; color: #bfbfbf; }

QComboBox { background-color: #ffffff; border: 1px solid #d9d9d9;
    border-radius: 4px; padding: 6px 10px; color: #333333; font-size: 12px; min-width: 80px; }
QComboBox:hover { border-color: #1890ff; }
QComboBox::drop-down { border: none; width: 20px; }
QComboBox QAbstractItemView { background-color: #ffffff; border: 1px solid #d9d9d9;
    selection-background-color: #e6f7ff; selection-color: #1890ff; }

QPushButton { background-color: #ffffff; color: #333333; border: 1px solid #d9d9d9;
    border-radius: 4px; padding: 6px 16px; font-size: 12px; }
QPushButton:hover { border-color: #1890ff; color: #1890ff; }
QPushButton:pressed { background-color: #e6f7ff; }
QPushButton:disabled { background-color: #f5f5f5; color: #bfbfbf; border-color: #d9d9d9; }

QPushButton#primary_btn { background-color: #1890ff; color: #ffffff; border: none; }
QPushButton#primary_btn:hover { background-color: #40a9ff; }
QPushButton#primary_btn:disabled { background-color: #bfbfbf; }

QPushButton#success_btn { background-color: #52c41a; color: #ffffff;
    border: none; font-weight: bold; }
QPushButton#success_btn:hover { background-color: #73d13d; }

QPushButton#danger_btn { background-color: #ff4d4f; color: #ffffff;
    border: none; font-weight: bold; }
QPushButton#danger_btn:hover { background-color: #ff7875; }

QPushButton#warning_btn { background-color: #faad14; color: #ffffff;
    border: none; font-weight: bold; }
QPushButton#warning_btn:hover { background-color: #ffc53d; }

QLabel#status_connected { color: #52c41a; font-weight: bold; }
QLabel#status_disconnected { color: #ff4d4f; font-weight: bold; }

QLabel#mode_cv { color: #3498db; font-weight: bold; font-size: 14px; }
QLabel#mode_cc { color: #e67e22; font-weight: bold; font-size: 14px; }
QLabel#protection_normal { color: #27ae60; }
QLabel#protection_warning { color: #e74c3c; font-weight: bold; }
"""


# PID 控制器已移至 src/pid_controller.py，由 HardwareWorker 使用


# ==================== 数值显示组件 ====================

class ValueDisplay(QFrame):
    """现代风格的数值显示组件"""

    def __init__(self, title: str, unit: str, decimals: int = 3, parent=None):
        super().__init__(parent)
        self.decimals = decimals
        self.unit = unit

        self.setFrameStyle(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: #fafafa;
                border: 1px solid #f0f0f0;
                border-radius: 8px;
            }
        """)

        layout = QVBoxLayout(self)
        layout.setSpacing(4)
        layout.setContentsMargins(12, 8, 12, 8)

        self.title_label = QLabel(title)
        self.title_label.setStyleSheet("font-size: 11px; color: #8c8c8c; border: none; background: transparent;")
        layout.addWidget(self.title_label)

        value_layout = QHBoxLayout()
        value_layout.setSpacing(4)

        self.value_label = QLabel("---.---")
        self.value_label.setStyleSheet("""
            font-size: 24px; font-weight: bold; color: #262626;
            border: none; background: transparent;
        """)
        value_layout.addWidget(self.value_label)

        self.unit_label = QLabel(unit)
        self.unit_label.setStyleSheet("font-size: 14px; color: #1890ff; font-weight: bold; border: none; background: transparent;")
        value_layout.addWidget(self.unit_label)
        value_layout.addStretch()

        layout.addLayout(value_layout)

    def set_value(self, value: float):
        self.value_label.setText(f"{value:.{self.decimals}f}")

    def set_warning(self, warning: bool):
        color = "#ff4d4f" if warning else "#262626"
        self.value_label.setStyleSheet(f"""
            font-size: 24px; font-weight: bold; color: {color};
            border: none; background: transparent;
        """)


# ==================== 主窗口 ====================

class MainWindow(QMainWindow):
    """CL-500W 电源控制器 + 温度控制主界面 (纯 UI 层, 零硬件 I/O)"""

    # ---- 请求信号: UI → HardwareWorker (跨线程) ----
    req_connect_power = Signal(str, int, int)     # port, baudrate, address
    req_disconnect_power = Signal()
    req_set_voltage = Signal(float)
    req_set_current = Signal(float)
    req_output_on = Signal()
    req_output_off = Signal()
    req_connect_temp = Signal(int, str)            # index, port
    req_disconnect_temp = Signal(int)
    req_start_control = Signal(dict)
    req_stop_control = Signal()
    req_start_auto_tune = Signal(dict)
    req_apply_tune = Signal()
    req_refresh_ports = Signal()
    req_update_params = Signal(dict)

    def __init__(self):
        super().__init__()

        # UI 状态标志 (只用于控制界面显示, 不涉及硬件)
        self._power_connected = False
        self._pid_enabled = False
        self._auto_tuning = False

        self._setup_ui()
        self._load_config()

        # ---- 创建工作者线程 (所有硬件 I/O 在此线程) ----
        self._worker_thread = QThread(self)
        self._worker = HardwareWorker()
        self._worker.moveToThread(self._worker_thread)
        self._worker_thread.started.connect(self._worker.startup)

        # 连接: UI → Worker (请求)
        self.req_connect_power.connect(self._worker.connect_power)
        self.req_disconnect_power.connect(self._worker.disconnect_power)
        self.req_set_voltage.connect(self._worker.set_voltage)
        self.req_set_current.connect(self._worker.set_current)
        self.req_output_on.connect(self._worker.output_on)
        self.req_output_off.connect(self._worker.output_off)
        self.req_connect_temp.connect(self._worker.connect_temp)
        self.req_disconnect_temp.connect(self._worker.disconnect_temp)
        self.req_start_control.connect(self._worker.start_control)
        self.req_stop_control.connect(self._worker.stop_control)
        self.req_start_auto_tune.connect(self._worker.start_auto_tune)
        self.req_apply_tune.connect(self._worker.apply_tune)
        self.req_refresh_ports.connect(self._worker.refresh_ports)
        self.req_update_params.connect(self._worker.update_params)

        # 连接: Worker → UI (结果)
        self._worker.power_connect_result.connect(self._on_power_connected)
        self._worker.power_disconnected.connect(self._on_power_disconnected)
        self._worker.power_status_updated.connect(self._update_display)
        self._worker.poll_error_occurred.connect(self._handle_poll_error)
        self._worker.set_voltage_result.connect(self._on_set_voltage_result)
        self._worker.set_current_result.connect(self._on_set_current_result)
        self._worker.output_on_result.connect(self._on_output_on_result)
        self._worker.output_off_result.connect(self._on_output_off_result)
        self._worker.temp_connect_result.connect(self._on_temp_connected)
        self._worker.temp_disconnected_sig.connect(self._on_temp_disconnected)
        self._worker.temp_data_updated.connect(self._on_temp_data)
        self._worker.control_start_result.connect(self._on_control_start_result)
        self._worker.control_stopped_sig.connect(self._on_control_stopped)
        self._worker.control_status_sig.connect(self._on_control_status)
        self._worker.auto_tune_start_result.connect(self._on_auto_tune_start_result)
        self._worker.auto_tune_msg_sig.connect(self._on_auto_tune_msg)
        self._worker.auto_tune_done_sig.connect(self._on_auto_tune_done)
        self._worker.auto_tune_failed_sig.connect(self._on_auto_tune_failed)
        self._worker.safety_triggered_sig.connect(self._on_safety_triggered)
        self._worker.safety_recovered_sig.connect(self._on_safety_recovered)
        self._worker.ports_refreshed_sig.connect(self._on_ports_refreshed)
        self._worker.chart_data_sig.connect(self._on_chart_data)
        self._worker.bridge_params_sig.connect(self._on_bridge_params)

        # 启动工作者线程
        self._worker_thread.start()

        # 请求初始串口列表 (异步, 不阻塞 UI)
        QTimer.singleShot(100, lambda: self.req_refresh_ports.emit())

    # ==================== UI 构建 ====================

    def _setup_ui(self):
        """设置 UI - 左右布局"""
        self.setWindowTitle("多设备控制系统 - 电源控制")
        self.setMinimumSize(1600, 750)
        self.resize(1800, 860)
        self.setStyleSheet(MODERN_STYLE)

        # 温度历史记录（用于绘图, UI 端维护）
        self._history_max = 300
        self._time_history = deque(maxlen=self._history_max)
        self._target_history = deque(maxlen=self._history_max)
        self._cold1_history = deque(maxlen=self._history_max)
        self._hot1_history = deque(maxlen=self._history_max)
        self._cold2_history = deque(maxlen=self._history_max)
        self._hot2_history = deque(maxlen=self._history_max)
        self._output_history = deque(maxlen=self._history_max)

        # ---- 中央部件 ----
        central = QWidget()
        self.setCentralWidget(central)

        # 整体用单个 QScrollArea 包裹，保证左右等高并底部对齐
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("QScrollArea { border: none; background-color: #f0f0f0; }")

        scroll_content = QWidget()
        top_layout = QHBoxLayout(scroll_content)
        top_layout.setSpacing(0)
        top_layout.setContentsMargins(0, 0, 0, 0)

        # ==== 左侧：电源 + 温度采集 ====
        left_content = QWidget()
        left_content.setObjectName("content_area")
        left_layout = QVBoxLayout(left_content)
        left_layout.setSpacing(15)
        left_layout.setContentsMargins(20, 15, 20, 15)

        left_layout.addWidget(self._create_connection_section())

        power_row = QHBoxLayout()
        power_row.setSpacing(20)
        power_row.addWidget(self._create_parameter_section(), stretch=1)
        power_row.addWidget(self._create_status_section(), stretch=1)
        power_row.addWidget(self._create_output_section(), stretch=1)
        left_layout.addLayout(power_row)

        left_layout.addWidget(self._create_temperature_area())
        left_layout.addStretch()

        top_layout.addWidget(left_content, stretch=55)

        # ==== 右侧：温度控制面板 ====
        right_content = self._create_temp_control_panel()
        top_layout.addWidget(right_content, stretch=45)

        scroll.setWidget(scroll_content)

        outer = QVBoxLayout(central)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(scroll)

        # 注: 温度更新/PID 控制/图表数据/Bridge 同步 全部由 HardwareWorker 线程驱动
        # UI 只通过信号接收数据并更新显示

    # ---------- 连接区域 ----------

    def _create_connection_section(self) -> QFrame:
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame { background-color: #fafafa; border: 1px solid #f0f0f0; border-radius: 8px; }
        """)
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(15, 10, 15, 10)
        layout.setSpacing(15)

        port_label = QLabel("串口:")
        port_label.setStyleSheet("border: none;")
        layout.addWidget(port_label)
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(150)
        self.combo_port.setStyleSheet("border: 1px solid #d9d9d9; border-radius: 4px;")
        layout.addWidget(self.combo_port)

        self.btn_refresh = QPushButton("刷新")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        layout.addWidget(self.btn_refresh)

        baud_label = QLabel("波特率:")
        baud_label.setStyleSheet("border: none;")
        layout.addWidget(baud_label)
        self.combo_baudrate = QComboBox()
        self.combo_baudrate.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.combo_baudrate.setCurrentText("9600")
        self.combo_baudrate.setStyleSheet("border: 1px solid #d9d9d9; border-radius: 4px;")
        layout.addWidget(self.combo_baudrate)

        addr_label = QLabel("地址:")
        addr_label.setStyleSheet("border: none;")
        layout.addWidget(addr_label)
        self.spin_address = QSpinBox()
        self.spin_address.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_address.setRange(1, 127)
        self.spin_address.setValue(1)
        self.spin_address.setFixedWidth(70)
        self.spin_address.setFixedHeight(28)
        layout.addWidget(self.spin_address)

        layout.addStretch()

        self.btn_connect = QPushButton("连接")
        self.btn_connect.setObjectName("success_btn")
        self.btn_connect.setFixedWidth(80)
        self.btn_connect.clicked.connect(self._connect)
        layout.addWidget(self.btn_connect)

        self.btn_disconnect = QPushButton("断开")
        self.btn_disconnect.setObjectName("danger_btn")
        self.btn_disconnect.setFixedWidth(80)
        self.btn_disconnect.setEnabled(False)
        self.btn_disconnect.clicked.connect(self._disconnect)
        layout.addWidget(self.btn_disconnect)

        self.label_status = QLabel("● 未连接")
        self.label_status.setObjectName("status_disconnected")
        self.label_status.setStyleSheet("border: none; font-weight: bold;")
        layout.addWidget(self.label_status)

        return frame

    # ---------- 参数设置 ----------

    def _create_parameter_section(self) -> QFrame:
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #fafafa; border: 1px solid #f0f0f0; border-radius: 8px; }")

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(15)

        title = QLabel("参数设置")
        title.setObjectName("section_title")
        title.setStyleSheet("border: none; font-size: 14px; font-weight: bold; color: #262626;")
        layout.addWidget(title)

        # 电压
        v_label = QLabel("设定电压")
        v_label.setStyleSheet("border: none; color: #8c8c8c;")
        layout.addWidget(v_label)

        v_row = QHBoxLayout()
        self.input_voltage = QDoubleSpinBox()
        self.input_voltage.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.input_voltage.setRange(0, 15)
        self.input_voltage.setDecimals(3)
        self.input_voltage.setSingleStep(0.1)
        self.input_voltage.setValue(12.0)
        self.input_voltage.setEnabled(False)
        self.input_voltage.setSuffix(" V")
        v_row.addWidget(self.input_voltage)

        self.btn_set_voltage = QPushButton("设置")
        self.btn_set_voltage.setObjectName("primary_btn")
        self.btn_set_voltage.setFixedWidth(60)
        self.btn_set_voltage.setEnabled(False)
        self.btn_set_voltage.clicked.connect(self._set_voltage)
        v_row.addWidget(self.btn_set_voltage)
        layout.addLayout(v_row)

        self.label_voltage_set = QLabel("当前设定: --")
        self.label_voltage_set.setStyleSheet("border: none; color: #1890ff; font-size: 11px;")
        layout.addWidget(self.label_voltage_set)

        # 电流
        i_label = QLabel("设定电流限制")
        i_label.setStyleSheet("border: none; color: #8c8c8c;")
        layout.addWidget(i_label)

        i_row = QHBoxLayout()
        self.input_current = QDoubleSpinBox()
        self.input_current.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.input_current.setRange(0, 14)
        self.input_current.setDecimals(3)
        self.input_current.setSingleStep(0.1)
        self.input_current.setValue(7.0)
        self.input_current.setEnabled(False)
        self.input_current.setSuffix(" A")
        i_row.addWidget(self.input_current)

        self.btn_set_current = QPushButton("设置")
        self.btn_set_current.setObjectName("primary_btn")
        self.btn_set_current.setFixedWidth(60)
        self.btn_set_current.setEnabled(False)
        self.btn_set_current.clicked.connect(self._set_current)
        i_row.addWidget(self.btn_set_current)
        layout.addLayout(i_row)

        self.label_current_set = QLabel("当前限制: --")
        self.label_current_set.setStyleSheet("border: none; color: #fa8c16; font-size: 11px;")
        layout.addWidget(self.label_current_set)

        hint = QLabel("ℹ CV模式: 电流由负载决定\nCC模式: 负载电流>限制时触发")
        hint.setStyleSheet("border: none; color: #bfbfbf; font-size: 10px;")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        layout.addStretch()
        return frame

    # ---------- 实时监测 ----------

    def _create_status_section(self) -> QFrame:
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #fafafa; border: 1px solid #f0f0f0; border-radius: 8px; }")

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(12)

        title = QLabel("实时监测")
        title.setObjectName("section_title")
        title.setStyleSheet("border: none; font-size: 14px; font-weight: bold; color: #262626;")
        layout.addWidget(title)

        self.display_voltage = ValueDisplay("输出电压", "V", 3)
        layout.addWidget(self.display_voltage)
        self.display_current = ValueDisplay("输出电流", "A", 3)
        layout.addWidget(self.display_current)
        self.display_power = ValueDisplay("输出功率", "W", 2)
        layout.addWidget(self.display_power)

        info_frame = QFrame()
        info_frame.setStyleSheet("border: none;")
        info_layout = QHBoxLayout(info_frame)
        info_layout.setContentsMargins(0, 5, 0, 0)
        info_layout.setSpacing(20)

        mode_label = QLabel("模式:")
        mode_label.setStyleSheet("color: #8c8c8c;")
        info_layout.addWidget(mode_label)
        self.label_mode = QLabel("--")
        self.label_mode.setStyleSheet("color: #1890ff; font-weight: bold;")
        info_layout.addWidget(self.label_mode)

        temp_label = QLabel("温度:")
        temp_label.setStyleSheet("color: #8c8c8c;")
        info_layout.addWidget(temp_label)
        self.label_temp = QLabel("-- ℃")
        self.label_temp.setStyleSheet("color: #262626;")
        info_layout.addWidget(self.label_temp)
        info_layout.addStretch()

        layout.addWidget(info_frame)
        layout.addStretch()
        return frame

    # ---------- 输出控制 ----------

    def _create_output_section(self) -> QFrame:
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #fafafa; border: 1px solid #f0f0f0; border-radius: 8px; }")

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(15)

        title = QLabel("输出控制")
        title.setObjectName("section_title")
        title.setStyleSheet("border: none; font-size: 14px; font-weight: bold; color: #262626;")
        layout.addWidget(title)

        status_frame = QFrame()
        status_frame.setStyleSheet("QFrame { background-color: #ffffff; border: 2px solid #f0f0f0; border-radius: 8px; }")
        status_layout = QVBoxLayout(status_frame)
        status_layout.setContentsMargins(15, 15, 15, 15)

        self.label_output_status = QLabel("输出状态")
        self.label_output_status.setStyleSheet("border: none; color: #8c8c8c; font-size: 12px;")
        self.label_output_status.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.label_output_status)

        self.label_output_indicator = QLabel("--")
        self.label_output_indicator.setStyleSheet("border: none; font-size: 20px; font-weight: bold; color: #bfbfbf;")
        self.label_output_indicator.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.label_output_indicator)
        layout.addWidget(status_frame)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)
        self.btn_output_on = QPushButton("开启输出")
        self.btn_output_on.setObjectName("success_btn")
        self.btn_output_on.setEnabled(False)
        self.btn_output_on.setMinimumHeight(40)
        self.btn_output_on.clicked.connect(self._output_on)
        btn_row.addWidget(self.btn_output_on)

        self.btn_output_off = QPushButton("关闭输出")
        self.btn_output_off.setObjectName("danger_btn")
        self.btn_output_off.setEnabled(False)
        self.btn_output_off.setMinimumHeight(40)
        self.btn_output_off.clicked.connect(self._output_off)
        btn_row.addWidget(self.btn_output_off)
        layout.addLayout(btn_row)

        prot_frame = QFrame()
        prot_frame.setStyleSheet("border: none;")
        prot_layout = QHBoxLayout(prot_frame)
        prot_layout.setContentsMargins(0, 10, 0, 0)
        prot_label = QLabel("保护状态:")
        prot_label.setStyleSheet("color: #8c8c8c;")
        prot_layout.addWidget(prot_label)
        self.label_protection = QLabel("正常")
        self.label_protection.setStyleSheet("color: #52c41a; font-weight: bold;")
        prot_layout.addWidget(self.label_protection)
        prot_layout.addStretch()
        layout.addWidget(prot_frame)

        layout.addStretch()
        return frame

    # ---------- 温度检测区域（两个面板） ----------

    def _create_temperature_area(self) -> QFrame:
        frame = QFrame()
        frame.setStyleSheet("QFrame { border: none; background: transparent; }")
        layout = QHBoxLayout(frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(20)
        layout.addWidget(self._create_temp_panel(1), stretch=1)
        layout.addWidget(self._create_temp_panel(2), stretch=1)
        return frame

    def _create_temp_panel(self, index: int) -> QFrame:
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #fafafa; border: 1px solid #f0f0f0; border-radius: 8px; }")

        layout = QVBoxLayout(frame)
        layout.setContentsMargins(15, 12, 15, 12)
        layout.setSpacing(10)

        title = QLabel(f"温度检测 {index}")
        title.setObjectName("section_title")
        title.setStyleSheet("border: none; font-size: 14px; font-weight: bold; color: #262626;")
        layout.addWidget(title)

        # 串口连接
        conn_frame = QFrame()
        conn_frame.setStyleSheet("QFrame { background-color: #ffffff; border: 1px solid #f0f0f0; border-radius: 6px; }")
        conn_layout = QGridLayout(conn_frame)
        conn_layout.setContentsMargins(10, 8, 10, 8)
        conn_layout.setHorizontalSpacing(6)
        conn_layout.setVerticalSpacing(6)
        conn_layout.setColumnStretch(1, 1)

        port_label = QLabel("串口:")
        port_label.setStyleSheet("border: none; color: #8c8c8c;")
        conn_layout.addWidget(port_label, 0, 0)

        combo_port = QComboBox()
        combo_port.setMinimumWidth(140)
        combo_port.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        combo_port.setStyleSheet("border: 1px solid #d9d9d9; border-radius: 4px; font-size: 11px;")
        conn_layout.addWidget(combo_port, 0, 1)

        btn_refresh = QPushButton("刷新")
        btn_refresh.setFixedWidth(48)
        btn_refresh.setStyleSheet("font-size: 11px; padding: 4px;")
        conn_layout.addWidget(btn_refresh, 0, 2)

        btn_connect = QPushButton("连接")
        btn_connect.setObjectName("success_btn")
        btn_connect.setFixedWidth(52)
        btn_connect.setStyleSheet("font-size: 11px; padding: 4px;")
        conn_layout.addWidget(btn_connect, 0, 3)

        btn_disconnect = QPushButton("断开")
        btn_disconnect.setObjectName("danger_btn")
        btn_disconnect.setFixedWidth(52)
        btn_disconnect.setStyleSheet("font-size: 11px; padding: 4px;")
        btn_disconnect.setEnabled(False)
        conn_layout.addWidget(btn_disconnect, 0, 4)

        status_label = QLabel("● 未连接")
        status_label.setStyleSheet("border: none; font-weight: bold; color: #ff4d4f;")
        conn_layout.addWidget(status_label, 1, 0, 1, 5)
        layout.addWidget(conn_frame)

        # 温度显示
        ds_display = ValueDisplay("DS18B20 温度", "℃", 2)
        layout.addWidget(ds_display)
        hot_display = ValueDisplay("热端温度", "℃", 2)
        layout.addWidget(hot_display)
        cold_display = ValueDisplay("冷端温度", "℃", 2)
        layout.addWidget(cold_display)

        # 保存引用
        if index == 1:
            self.temp1_combo_port = combo_port
            self.temp1_btn_refresh = btn_refresh
            self.temp1_btn_connect = btn_connect
            self.temp1_btn_disconnect = btn_disconnect
            self.temp1_status_label = status_label
            self.temp1_ds_display = ds_display
            self.temp1_hot_display = hot_display
            self.temp1_cold_display = cold_display
            btn_refresh.clicked.connect(lambda: self._refresh_temp_ports(1))
            btn_connect.clicked.connect(lambda: self._connect_temp(1))
            btn_disconnect.clicked.connect(lambda: self._disconnect_temp(1))
            self._refresh_temp_ports(1)
        else:
            self.temp2_combo_port = combo_port
            self.temp2_btn_refresh = btn_refresh
            self.temp2_btn_connect = btn_connect
            self.temp2_btn_disconnect = btn_disconnect
            self.temp2_status_label = status_label
            self.temp2_ds_display = ds_display
            self.temp2_hot_display = hot_display
            self.temp2_cold_display = cold_display
            btn_refresh.clicked.connect(lambda: self._refresh_temp_ports(2))
            btn_connect.clicked.connect(lambda: self._connect_temp(2))
            btn_disconnect.clicked.connect(lambda: self._disconnect_temp(2))
            self._refresh_temp_ports(2)

        return frame

    # ---------- 右侧温度控制面板 ----------

    def _create_temp_control_panel(self) -> QWidget:
        """创建温度控制面板（右侧）"""
        panel = QWidget()
        panel.setObjectName("content_area")

        layout = QVBoxLayout(panel)
        layout.setSpacing(8)
        layout.setContentsMargins(15, 10, 15, 10)

        # ===== 控制源与目标温度 =====
        src_frame = self._make_card()
        src_layout = QVBoxLayout(src_frame)
        src_layout.setContentsMargins(10, 8, 10, 8)
        src_layout.setSpacing(6)

        src_title = QLabel("控制源与目标温度")
        src_title.setStyleSheet("border: none; color: #8c8c8c; font-size: 11px;")
        src_layout.addWidget(src_title)

        src_row = QHBoxLayout()
        src_row.setSpacing(10)

        fusion_lbl = QLabel("融合策略:")
        fusion_lbl.setStyleSheet("border: none;")
        src_row.addWidget(fusion_lbl)

        self.combo_fusion_mode = QComboBox()
        self.combo_fusion_mode.addItems(["双传感器平均", "仅传感器 1", "仅传感器 2"])
        self.combo_fusion_mode.setStyleSheet("border: 1px solid #d9d9d9; border-radius: 4px;")
        src_row.addWidget(self.combo_fusion_mode)

        mode_lbl = QLabel("控制模式:")
        mode_lbl.setStyleSheet("border: none;")
        src_row.addWidget(mode_lbl)

        self.combo_control_mode = QComboBox()
        self.combo_control_mode.addItems(["制冷模式", "制热模式"])
        self.combo_control_mode.setStyleSheet("border: 1px solid #d9d9d9; border-radius: 4px;")
        src_row.addWidget(self.combo_control_mode)

        target_lbl = QLabel("目标温度:")
        target_lbl.setStyleSheet("border: none;")
        src_row.addWidget(target_lbl)

        self.spin_target_temp = QDoubleSpinBox()
        self.spin_target_temp.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_target_temp.setRange(-20, 80)
        self.spin_target_temp.setDecimals(2)
        self.spin_target_temp.setSingleStep(0.5)
        self.spin_target_temp.setValue(15.0)
        self.spin_target_temp.setSuffix(" ℃")
        self.spin_target_temp.setFixedHeight(30)
        src_row.addWidget(self.spin_target_temp)

        safety_lbl = QLabel("⚠上限:")
        safety_lbl.setStyleSheet("border: none; color: #ff4d4f; font-weight: bold;")
        src_row.addWidget(safety_lbl)

        self.spin_safety_temp = QDoubleSpinBox()
        self.spin_safety_temp.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_safety_temp.setRange(0, 80)
        self.spin_safety_temp.setDecimals(1)
        self.spin_safety_temp.setValue(30.0)
        self.spin_safety_temp.setSuffix(" ℃")
        self.spin_safety_temp.setFixedHeight(30)
        self.spin_safety_temp.setToolTip("温度超过此值将自动暂停实验并重新优化")
        src_row.addWidget(self.spin_safety_temp)

        src_layout.addLayout(src_row)

        layout.addWidget(src_frame)

        # ===== PID 参数 =====
        pid_frame = self._make_card()
        pid_layout = QGridLayout(pid_frame)
        pid_layout.setContentsMargins(10, 6, 10, 6)
        pid_layout.setSpacing(6)

        # Kp
        kp_lbl = QLabel("Kp:")
        kp_lbl.setStyleSheet("border: none;")
        pid_layout.addWidget(kp_lbl, 1, 0)
        self.spin_kp = QDoubleSpinBox()
        self.spin_kp.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_kp.setRange(0, 50)
        self.spin_kp.setDecimals(2)
        self.spin_kp.setValue(1.00)
        self.spin_kp.setSingleStep(0.1)
        pid_layout.addWidget(self.spin_kp, 1, 1)

        # Ki
        ki_lbl = QLabel("Ki:")
        ki_lbl.setStyleSheet("border: none;")
        pid_layout.addWidget(ki_lbl, 1, 2)
        self.spin_ki = QDoubleSpinBox()
        self.spin_ki.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_ki.setRange(0, 10)
        self.spin_ki.setDecimals(3)
        self.spin_ki.setValue(0.050)
        self.spin_ki.setSingleStep(0.01)
        pid_layout.addWidget(self.spin_ki, 1, 3)

        # Kd
        kd_lbl = QLabel("Kd:")
        kd_lbl.setStyleSheet("border: none;")
        pid_layout.addWidget(kd_lbl, 2, 0)
        self.spin_kd = QDoubleSpinBox()
        self.spin_kd.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_kd.setRange(0, 50)
        self.spin_kd.setDecimals(2)
        self.spin_kd.setValue(0.50)
        self.spin_kd.setSingleStep(0.1)
        pid_layout.addWidget(self.spin_kd, 2, 1)

        # 最大电流
        max_lbl = QLabel("最大电流:")
        max_lbl.setStyleSheet("border: none;")
        pid_layout.addWidget(max_lbl, 2, 2)
        self.spin_max_current = QDoubleSpinBox()
        self.spin_max_current.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_max_current.setRange(0.1, 14)
        self.spin_max_current.setDecimals(3)
        self.spin_max_current.setValue(7.000)
        self.spin_max_current.setSuffix(" A")
        pid_layout.addWidget(self.spin_max_current, 2, 3)

        # 控制周期
        interval_lbl = QLabel("控制周期:")
        interval_lbl.setStyleSheet("border: none;")
        pid_layout.addWidget(interval_lbl, 3, 0)
        self.spin_interval = QDoubleSpinBox()
        self.spin_interval.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_interval.setRange(0.01, 10)
        self.spin_interval.setDecimals(2)
        self.spin_interval.setValue(1.0)
        self.spin_interval.setSuffix(" s")
        pid_layout.addWidget(self.spin_interval, 3, 1)

        # 最大电压（确保CC模式）
        max_v_lbl = QLabel("最大电压:")
        max_v_lbl.setStyleSheet("border: none;")
        pid_layout.addWidget(max_v_lbl, 3, 2)
        self.spin_max_voltage = QDoubleSpinBox()
        self.spin_max_voltage.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.spin_max_voltage.setRange(1.0, 15.0)
        self.spin_max_voltage.setDecimals(1)
        self.spin_max_voltage.setValue(12.0)
        self.spin_max_voltage.setSuffix(" V")
        self.spin_max_voltage.setToolTip("PID控制时自动设置此电压，确保电源工作在CC恒流模式")
        pid_layout.addWidget(self.spin_max_voltage, 3, 3)

        layout.addWidget(pid_frame)

        # ===== 自动整定 + 控制按钮 (左右两列布局) =====
        action_frame = self._make_card()
        action_layout = QVBoxLayout(action_frame)
        action_layout.setContentsMargins(10, 6, 10, 6)
        action_layout.setSpacing(6)

        buttons_grid = QGridLayout()
        buttons_grid.setSpacing(6)

        self.btn_auto_tune = QPushButton("🔧 开始自动整定")
        self.btn_auto_tune.setObjectName("warning_btn")
        self.btn_auto_tune.setMinimumHeight(32)
        self.btn_auto_tune.clicked.connect(self._start_auto_tune)
        buttons_grid.addWidget(self.btn_auto_tune, 0, 0)

        self.btn_start_control = QPushButton("▶ 启动控制")
        self.btn_start_control.setObjectName("success_btn")
        self.btn_start_control.setMinimumHeight(32)
        self.btn_start_control.clicked.connect(self._start_control)
        buttons_grid.addWidget(self.btn_start_control, 0, 1)

        self.btn_apply_tune = QPushButton("✓ 应用整定结果")
        self.btn_apply_tune.setObjectName("primary_btn")
        self.btn_apply_tune.setMinimumHeight(32)
        self.btn_apply_tune.setEnabled(False)
        self.btn_apply_tune.clicked.connect(self._apply_tuned_params)
        buttons_grid.addWidget(self.btn_apply_tune, 1, 0)

        self.btn_stop_control = QPushButton("⏹ 停止控制")
        self.btn_stop_control.setObjectName("danger_btn")
        self.btn_stop_control.setMinimumHeight(32)
        self.btn_stop_control.setEnabled(False)
        self.btn_stop_control.clicked.connect(self._stop_control)
        buttons_grid.addWidget(self.btn_stop_control, 1, 1)

        action_layout.addLayout(buttons_grid)

        self.auto_tune_status = QLabel("未整定 — 使用默认参数")
        self.auto_tune_status.setStyleSheet("border: none; color: #bfbfbf; font-size: 11px;")
        self.auto_tune_status.setWordWrap(True)
        action_layout.addWidget(self.auto_tune_status)

        layout.addWidget(action_frame)

        # ===== 控制状态 =====
        status_frame = self._make_card()
        status_grid = QGridLayout(status_frame)
        status_grid.setContentsMargins(10, 6, 10, 6)
        status_grid.setHorizontalSpacing(10)
        status_grid.setVerticalSpacing(4)

        def _add_status_item(row, col, label_text):
            container = QWidget()
            h = QHBoxLayout(container)
            h.setContentsMargins(0, 0, 0, 0)
            h.setSpacing(4)
            lbl = QLabel(label_text)
            lbl.setStyleSheet("border: none; color: #8c8c8c;")
            val = QLabel("--")
            val.setStyleSheet("border: none; color: #262626; font-weight: bold;")
            h.addWidget(lbl)
            h.addWidget(val)
            h.addStretch()
            status_grid.addWidget(container, row, col)
            return val

        self.ctrl_status_label = _add_status_item(0, 0, "控制状态:")
        self.ctrl_status_label.setText("● 未启动")
        self.ctrl_status_label.setStyleSheet("border: none; color: #bfbfbf; font-weight: bold;")
        self.ctrl_temp_label = _add_status_item(0, 1, "当前温度:")
        self.ctrl_target_label = _add_status_item(0, 2, "目标温度:")

        self.ctrl_error_label = _add_status_item(1, 0, "温度偏差:")
        self.ctrl_output_label = _add_status_item(1, 1, "输出电流:")
        self.ctrl_time_label = _add_status_item(1, 2, "运行时间:")

        layout.addWidget(status_frame)

        # ===== 温度曲线图 =====
        chart_frame = self._make_card()
        chart_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        chart_layout = QVBoxLayout(chart_frame)
        chart_layout.setContentsMargins(4, 2, 4, 4)
        chart_layout.setSpacing(0)

        self._chart_fig = Figure(figsize=(6, 2.8), dpi=110, facecolor='#fafafa')
        self._chart_canvas = FigureCanvas(self._chart_fig)
        self._chart_canvas.setMinimumHeight(200)
        self._chart_canvas.setStyleSheet("border: none;")

        self._chart_ax = self._chart_fig.add_subplot(111)
        self._chart_ax2 = self._chart_ax.twinx()

        # 初始化线条
        self._line_target, = self._chart_ax.plot([], [], 'g--', label='目标', linewidth=2.5, alpha=0.9)
        self._line_cold1, = self._chart_ax.plot([], [], '#1890ff', label='冷端1', linewidth=2)
        self._line_hot1, = self._chart_ax.plot([], [], '#ff7875', label='热端1', linewidth=1.5, alpha=0.7, linestyle='--')
        self._line_cold2, = self._chart_ax.plot([], [], '#722ed1', label='冷端2', linewidth=2)
        self._line_hot2, = self._chart_ax.plot([], [], '#eb2f96', label='热端2', linewidth=1.5, alpha=0.7, linestyle='--')
        self._line_output, = self._chart_ax2.plot([], [], '#faad14', label='输出电流', linewidth=2, alpha=0.8)

        self._chart_ax.set_xlabel('时间 (s)', fontsize=10)
        self._chart_ax.set_ylabel('温度 (℃)', fontsize=10, color='#1890ff')
        self._chart_ax2.set_ylabel('电流 (A)', fontsize=10, color='#faad14')
        self._chart_ax.grid(True, alpha=0.3)
        self._chart_ax.tick_params(labelsize=9)
        self._chart_ax2.tick_params(labelsize=9)

        # 图例
        lines = [self._line_target, self._line_cold1, self._line_hot1,
                 self._line_cold2, self._line_hot2, self._line_output]
        labels = [l.get_label() for l in lines]
        self._chart_ax.legend(lines, labels, loc='upper left', fontsize=8, ncol=3,
                              framealpha=0.9, edgecolor='#d9d9d9')

        self._chart_fig.tight_layout(pad=1.2)
        chart_layout.addWidget(self._chart_canvas, stretch=1)
        layout.addWidget(chart_frame, stretch=1)

        return panel

    def _make_card(self) -> QFrame:
        """创建卡片式容器"""
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #fafafa; border: 1px solid #f0f0f0; border-radius: 8px; }")
        return frame

    # ==================== 温度串口逻辑 (信号驱动) ====================

    def _refresh_temp_ports(self, index: int):
        """刷新温度串口列表 — 请求 Worker 提供最新列表"""
        self.req_refresh_ports.emit()

    def _connect_temp(self, index: int):
        """连接温度传感器 — 发送信号到 Worker"""
        combo = self.temp1_combo_port if index == 1 else self.temp2_combo_port
        port = combo.currentData()
        if not port:
            QMessageBox.warning(self, "错误", "请选择有效的串口")
            return
        self.req_connect_temp.emit(index, port)

    def _disconnect_temp(self, index: int):
        """断开温度传感器 — 发送信号到 Worker"""
        self.req_disconnect_temp.emit(index)

    def _set_temp_connected_state(self, index: int, connected: bool):
        if index == 1:
            combo, btn_c, btn_d, btn_r, lbl = (
                self.temp1_combo_port, self.temp1_btn_connect,
                self.temp1_btn_disconnect, self.temp1_btn_refresh,
                self.temp1_status_label)
        else:
            combo, btn_c, btn_d, btn_r, lbl = (
                self.temp2_combo_port, self.temp2_btn_connect,
                self.temp2_btn_disconnect, self.temp2_btn_refresh,
                self.temp2_status_label)

        combo.setEnabled(not connected)
        btn_c.setEnabled(not connected)
        btn_d.setEnabled(connected)
        btn_r.setEnabled(not connected)

        if connected:
            lbl.setText("● 已连接")
            lbl.setStyleSheet("border: none; font-weight: bold; color: #52c41a;")
        else:
            lbl.setText("● 未连接")
            lbl.setStyleSheet("border: none; font-weight: bold; color: #ff4d4f;")

    # ==================== Worker 信号处理 (温度) ====================

    def _on_temp_connected(self, index: int, success: bool, msg: str):
        """Worker: 温度传感器连接结果"""
        if success:
            self._set_temp_connected_state(index, True)
        else:
            QMessageBox.critical(self, "错误", f"连接温度串口失败: {msg}")

    def _on_temp_disconnected(self, index: int):
        """Worker: 温度传感器已断开"""
        self._set_temp_connected_state(index, False)

    def _on_temp_data(self, index: int, data: dict):
        """Worker: 温度数据更新 (100ms)"""
        if index == 1:
            ds_disp, hot_disp, cold_disp = self.temp1_ds_display, self.temp1_hot_display, self.temp1_cold_display
        else:
            ds_disp, hot_disp, cold_disp = self.temp2_ds_display, self.temp2_hot_display, self.temp2_cold_display

        ds = data.get('ds18b20', '--')
        hot = data.get('hot', '--')
        cold = data.get('cold', '--')
        if ds != '--':
            try: ds_disp.set_value(float(ds))
            except ValueError: pass
        if hot != '--':
            try: hot_disp.set_value(float(hot))
            except ValueError: pass
        if cold != '--':
            try: cold_disp.set_value(float(cold))
            except ValueError: pass

    # ==================== PID 控制 (信号驱动) ====================

    def _collect_control_params(self) -> dict:
        """从 UI 控件收集控制参数"""
        return {
            'target_temp': self.spin_target_temp.value(),
            'safety_temp': self.spin_safety_temp.value(),
            'kp': self.spin_kp.value(),
            'ki': self.spin_ki.value(),
            'kd': self.spin_kd.value(),
            'max_current': self.spin_max_current.value(),
            'max_voltage': self.spin_max_voltage.value(),
            'control_interval': self.spin_interval.value(),
            'fusion_mode': self.combo_fusion_mode.currentIndex(),
            'control_mode': self.combo_control_mode.currentIndex(),
        }

    def _start_control(self):
        """启动 PID 控制 — 发送信号到 Worker"""
        params = self._collect_control_params()
        self.req_update_params.emit(params)
        self.req_start_control.emit(params)

    def _stop_control(self):
        """停止 PID 控制 — 发送信号到 Worker"""
        self.req_stop_control.emit()

    def _on_control_start_result(self, success: bool, msg: str):
        """Worker: 控制启动结果"""
        if not success:
            QMessageBox.warning(self, "提示", msg)
            return

        self._pid_enabled = True
        self._auto_tuning = False
        self.btn_start_control.setEnabled(False)
        self.btn_stop_control.setEnabled(True)
        self.btn_auto_tune.setEnabled(False)
        mode_text = "制热" if self.combo_control_mode.currentIndex() == 1 else "制冷"
        self.ctrl_status_label.setText(f"● {mode_text}运行中")
        self.ctrl_status_label.setStyleSheet("border: none; color: #52c41a; font-weight: bold;")
        self._lock_control_inputs(True)

    def _on_control_stopped(self):
        """Worker: 控制已停止"""
        self._pid_enabled = False
        self._auto_tuning = False
        self.btn_start_control.setEnabled(True)
        self.btn_stop_control.setEnabled(False)
        self.btn_auto_tune.setEnabled(True)
        self.ctrl_status_label.setText("● 已停止")
        self.ctrl_status_label.setStyleSheet("border: none; color: #ff4d4f; font-weight: bold;")
        self._lock_control_inputs(False)

    def _on_control_status(self, info: dict):
        """Worker: PID 控制状态更新"""
        measured = info.get('measured', 0)
        target = info.get('target', 0)
        error = info.get('error', 0)
        output = info.get('output', 0)
        elapsed = info.get('elapsed', 0)

        self.ctrl_temp_label.setText(f"{measured:.2f} ℃")
        self.ctrl_target_label.setText(f"{target:.2f} ℃")
        self.ctrl_error_label.setText(f"{error:+.2f} ℃")
        err_color = '#ff4d4f' if abs(error) > 1.0 else '#52c41a'
        self.ctrl_error_label.setStyleSheet(f"border: none; color: {err_color}; font-weight: bold;")
        self.ctrl_output_label.setText(f"{output:.3f} A")

        h = int(elapsed // 3600)
        m = int((elapsed % 3600) // 60)
        s = int(elapsed % 60)
        self.ctrl_time_label.setText(f"{h:02d}:{m:02d}:{s:02d}")

    # ==================== 自动整定 (信号驱动) ====================

    def _start_auto_tune(self):
        """启动自动整定 — 发送信号到 Worker"""
        params = self._collect_control_params()
        self.req_update_params.emit(params)
        self.req_start_auto_tune.emit(params)

    def _on_auto_tune_start_result(self, success: bool, msg: str):
        """Worker: 自动整定启动结果"""
        if not success:
            QMessageBox.warning(self, "提示", msg)
            return

        self._auto_tuning = True
        self._pid_enabled = True
        self.btn_auto_tune.setEnabled(False)
        self.btn_start_control.setEnabled(False)
        self.btn_stop_control.setEnabled(True)
        self.btn_apply_tune.setEnabled(False)
        self.ctrl_status_label.setText("● 自动整定中")
        self.ctrl_status_label.setStyleSheet("border: none; color: #faad14; font-weight: bold;")
        self.auto_tune_status.setText("自动整定启动...")
        self.auto_tune_status.setStyleSheet("border: none; color: #faad14; font-size: 11px; font-weight: bold;")
        self._lock_control_inputs(True)

    def _on_auto_tune_msg(self, msg: str):
        """Worker: 自动整定进度消息"""
        self.auto_tune_status.setText(msg)

    def _on_auto_tune_done(self, kp: float, ki: float, kd: float, msg: str):
        """Worker: 自动整定完成"""
        self._auto_tuning = False
        self._pid_enabled = False

        self.spin_kp.setValue(kp)
        self.spin_ki.setValue(ki)
        self.spin_kd.setValue(kd)

        self.auto_tune_status.setText(msg)
        self.auto_tune_status.setStyleSheet("border: none; color: #52c41a; font-size: 11px; font-weight: bold;")
        self.btn_apply_tune.setEnabled(True)
        self.btn_auto_tune.setEnabled(True)
        self.btn_start_control.setEnabled(True)
        self.btn_stop_control.setEnabled(False)
        self.ctrl_status_label.setText("● 整定完成")
        self.ctrl_status_label.setStyleSheet("border: none; color: #1890ff; font-weight: bold;")
        self._lock_control_inputs(False)

    def _on_auto_tune_failed(self, msg: str):
        """Worker: 自动整定失败"""
        self._auto_tuning = False
        self._pid_enabled = False

        self.auto_tune_status.setText(f"❌ {msg}")
        self.auto_tune_status.setStyleSheet("border: none; color: #ff4d4f; font-size: 11px; font-weight: bold;")
        self.btn_auto_tune.setEnabled(True)
        self.btn_start_control.setEnabled(True)
        self.btn_stop_control.setEnabled(False)
        self.ctrl_status_label.setText("● 整定失败")
        self.ctrl_status_label.setStyleSheet("border: none; color: #ff4d4f; font-weight: bold;")
        self._lock_control_inputs(False)

    def _apply_tuned_params(self):
        """应用整定结果 — 发送信号到 Worker"""
        self.btn_apply_tune.setEnabled(False)
        self.req_apply_tune.emit()

    # ==================== 安全保护 (信号驱动) ====================

    def _on_safety_triggered(self, measured: float, limit: float, count: int):
        """Worker: 安全保护触发"""
        self._pid_enabled = False
        self._auto_tuning = False

        self.ctrl_status_label.setText(f"⚠ 安全停止 ({measured:.1f}℃ > {limit:.1f}℃)")
        self.ctrl_status_label.setStyleSheet("border: none; color: #ff4d4f; font-weight: bold;")

        self.btn_start_control.setEnabled(False)
        self.btn_stop_control.setEnabled(False)
        self.btn_auto_tune.setEnabled(False)
        self._lock_control_inputs(False)

        self.auto_tune_status.setText(
            f"⚠ 安全保护触发 (第{count}次)！"
            f"温度 {measured:.1f}℃ 超过上限 {limit:.1f}℃，已暂停实验。"
            f"\n等待温度回落后将自动重新整定...")
        self.auto_tune_status.setStyleSheet(
            "border: none; color: #ff4d4f; font-size: 11px; font-weight: bold;")

    def _on_safety_recovered(self, measured: float):
        """Worker: 安全恢复 — 温度已回落"""
        self.auto_tune_status.setText(
            f"温度已回落至 {measured:.1f}℃，正在自动重新整定...")
        self.auto_tune_status.setStyleSheet(
            "border: none; color: #faad14; font-size: 11px; font-weight: bold;")
        self.btn_auto_tune.setEnabled(True)
        self.btn_start_control.setEnabled(True)

    # ==================== 控制输入锁定 ====================

    def _lock_control_inputs(self, locked: bool):
        """锁定/解锁控制相关输入"""
        enabled = not locked
        self.spin_kp.setEnabled(enabled)
        self.spin_ki.setEnabled(enabled)
        self.spin_kd.setEnabled(enabled)
        self.spin_max_current.setEnabled(enabled)
        self.spin_max_voltage.setEnabled(enabled)
        self.spin_interval.setEnabled(enabled)
        self.combo_fusion_mode.setEnabled(enabled)
        self.combo_control_mode.setEnabled(enabled)
        self.spin_safety_temp.setEnabled(enabled)

    # ==================== 图表更新 (信号驱动) ====================

    def _on_chart_data(self, point: dict):
        """Worker: 图表数据点 (1s)"""
        t = point['t']

        self._time_history.append(t)
        self._target_history.append(point['target'])
        self._cold1_history.append(point['cold1'])
        self._hot1_history.append(point['hot1'])
        self._cold2_history.append(point['cold2'])
        self._hot2_history.append(point['hot2'])
        self._output_history.append(point['output'])

        times = list(self._time_history)
        self._line_target.set_data(times, list(self._target_history))
        self._line_cold1.set_data(times, list(self._cold1_history))
        self._line_hot1.set_data(times, list(self._hot1_history))
        self._line_cold2.set_data(times, list(self._cold2_history))
        self._line_hot2.set_data(times, list(self._hot2_history))
        self._line_output.set_data(times, list(self._output_history))

        self._chart_ax.relim()
        self._chart_ax.autoscale_view()
        self._chart_ax2.relim()
        self._chart_ax2.autoscale_view()

        if len(times) > 1:
            x_min = max(0, times[-1] - self._history_max)
            self._chart_ax.set_xlim(x_min, times[-1] + 5)

        try:
            self._chart_canvas.draw_idle()
        except Exception:
            pass

    # ==================== 电源槽函数 (信号驱动) ====================

    def _refresh_ports(self):
        """请求 Worker 刷新串口列表"""
        self.req_refresh_ports.emit()

    def _on_ports_refreshed(self, port_list: list):
        """Worker: 串口列表已刷新"""
        # 电源串口
        current_power = self.combo_port.currentData()
        self.combo_port.clear()
        for p in port_list:
            self.combo_port.addItem(p['display'], p['device'])
        if not port_list:
            self.combo_port.addItem("(无可用串口)", "")
        if current_power:
            self._select_combo_by_data(self.combo_port, current_power)

        # 温度串口
        for idx, combo in [(1, self.temp1_combo_port), (2, self.temp2_combo_port)]:
            current_temp = combo.currentData()
            combo.clear()
            for p in port_list:
                combo.addItem(p['display'], p['device'])
            if not port_list:
                combo.addItem("(无可用串口)", "")
            if current_temp:
                self._select_combo_by_data(combo, current_temp)

    def _connect(self):
        """连接电源 — 发送信号到 Worker"""
        port = self.combo_port.currentData()
        if not port:
            QMessageBox.warning(self, "错误", "请选择有效的串口")
            return
        baudrate = int(self.combo_baudrate.currentText())
        address = self.spin_address.value()
        self.req_connect_power.emit(port, baudrate, address)

    def _on_power_connected(self, success: bool, msg: str):
        """Worker: 电源连接结果"""
        if success:
            self._power_connected = True
            self._set_connected_state(True)
        else:
            self._power_connected = False
            QMessageBox.warning(self, "错误", msg)

    def _disconnect(self):
        """断开电源 — 发送信号到 Worker"""
        if self._pid_enabled:
            self.req_stop_control.emit()
        self.req_disconnect_power.emit()

    def _on_power_disconnected(self):
        """Worker: 电源已断开"""
        self._power_connected = False
        self._set_connected_state(False)

    def _set_connected_state(self, connected: bool):
        self.btn_connect.setEnabled(not connected)
        self.btn_disconnect.setEnabled(connected)
        self.combo_port.setEnabled(not connected)
        self.combo_baudrate.setEnabled(not connected)
        self.spin_address.setEnabled(not connected)

        self.input_voltage.setEnabled(connected)
        self.input_current.setEnabled(connected)
        self.btn_set_voltage.setEnabled(connected)
        self.btn_set_current.setEnabled(connected)
        self.btn_output_on.setEnabled(connected)
        self.btn_output_off.setEnabled(connected)

        if connected:
            self.label_status.setText("● 已连接")
            self.label_status.setStyleSheet("border: none; font-weight: bold; color: #52c41a;")
        else:
            self.label_status.setText("● 未连接")
            self.label_status.setStyleSheet("border: none; font-weight: bold; color: #ff4d4f;")
            self.display_voltage.set_value(0)
            self.display_current.set_value(0)
            self.display_power.set_value(0)
            self.label_mode.setText("--")
            self.label_temp.setText("-- ℃")
            self.label_output_indicator.setText("--")
            self.label_output_indicator.setStyleSheet("border: none; font-size: 20px; font-weight: bold; color: #bfbfbf;")

    def _handle_poll_error(self, message: str):
        if message:
            print(f"轮询错误: {message}")

    def _update_display(self, status: PowerStatus):
        """Worker: 电源状态更新"""
        self.display_voltage.set_value(status.voltage_real)
        self.display_current.set_value(status.current_real)
        power = status.voltage_real * status.current_real
        self.display_power.set_value(power)

        if status.voltage_set > 0:
            self.label_voltage_set.setText(f"当前设定: {status.voltage_set:.3f} V")
        if status.current_set > 0:
            self.label_current_set.setText(f"当前限制: {status.current_set:.3f} A")

        if status.is_output_on:
            if status.mode == PowerMode.CV:
                self.label_mode.setText("CV")
                self.label_mode.setStyleSheet("color: #1890ff; font-weight: bold;")
            else:
                self.label_mode.setText("CC")
                self.label_mode.setStyleSheet("color: #fa8c16; font-weight: bold;")
        else:
            self.label_mode.setText("--")
            self.label_mode.setStyleSheet("color: #bfbfbf; font-weight: bold;")

        self.label_temp.setText(f"{status.temperature:.0f} ℃")

        if status.is_output_on:
            self.label_output_indicator.setText("ON")
            self.label_output_indicator.setStyleSheet("border: none; font-size: 20px; font-weight: bold; color: #52c41a;")
        else:
            self.label_output_indicator.setText("OFF")
            self.label_output_indicator.setStyleSheet("border: none; font-size: 20px; font-weight: bold; color: #ff4d4f;")

        if status.protection == ProtectionStatus.NORMAL:
            self.label_protection.setText("正常")
            self.label_protection.setStyleSheet("color: #52c41a; font-weight: bold;")
            self.display_voltage.set_warning(False)
            self.display_current.set_warning(False)
        else:
            self.label_protection.setText(status.protection.value)
            self.label_protection.setStyleSheet("color: #ff4d4f; font-weight: bold;")
            self.display_voltage.set_warning(True)
            self.display_current.set_warning(True)

    def _set_voltage(self):
        """设置电压 — 发送信号到 Worker"""
        voltage = self.input_voltage.value()
        self.req_set_voltage.emit(voltage)

    def _on_set_voltage_result(self, success: bool):
        if not success:
            QMessageBox.warning(self, "错误", "设置电压失败")

    def _set_current(self):
        """设置电流 — 发送信号到 Worker"""
        current = self.input_current.value()
        self.req_set_current.emit(current)

    def _on_set_current_result(self, success: bool):
        if not success:
            QMessageBox.warning(self, "错误", "设置电流失败")

    def _output_on(self):
        """开启输出 — 发送信号到 Worker"""
        self.req_output_on.emit()

    def _on_output_on_result(self, success: bool):
        if not success:
            QMessageBox.warning(self, "错误", "开启输出失败")

    def _output_off(self):
        """关闭输出 — 发送信号到 Worker"""
        self.req_output_off.emit()

    def _on_output_off_result(self, success: bool):
        if not success:
            QMessageBox.warning(self, "错误", "关闭输出失败")

    # ==================== Bridge 参数同步 ====================

    def _on_bridge_params(self, params: dict):
        """Worker: Web 端修改了参数, 同步到 UI"""
        if 'kp' in params: self.spin_kp.setValue(params['kp'])
        if 'ki' in params: self.spin_ki.setValue(params['ki'])
        if 'kd' in params: self.spin_kd.setValue(params['kd'])
        if 'max_current' in params: self.spin_max_current.setValue(params['max_current'])
        if 'max_voltage' in params: self.spin_max_voltage.setValue(params['max_voltage'])
        if 'control_interval' in params: self.spin_interval.setValue(params['control_interval'])
        if 'target_temp' in params: self.spin_target_temp.setValue(params['target_temp'])
        if 'safety_temp' in params: self.spin_safety_temp.setValue(params['safety_temp'])
        if 'fusion_mode' in params: self.combo_fusion_mode.setCurrentIndex(params['fusion_mode'])
        if 'control_mode' in params: self.combo_control_mode.setCurrentIndex(params['control_mode'])

    def closeEvent(self, event):
        self._save_config()

        # 在 Worker 线程内调用 shutdown(), 确保 QTimer 在正确线程停止
        QMetaObject.invokeMethod(self._worker, "shutdown", Qt.ConnectionType.BlockingQueuedConnection)
        self._worker_thread.quit()
        self._worker_thread.wait(3000)

        event.accept()

    # ==================== 配置文件 (config.toml) ====================

    def _get_config_path(self) -> Path:
        return Path(__file__).resolve().parents[2] / "config.toml"

    def _select_combo_by_data(self, combo: QComboBox, value: str) -> None:
        if not value:
            return
        for i in range(combo.count()):
            if combo.itemData(i) == value:
                combo.setCurrentIndex(i)
                return
        for i in range(combo.count()):
            text = combo.itemText(i)
            if text.startswith(value):
                combo.setCurrentIndex(i)
                return

    def _load_config(self) -> None:
        path = self._get_config_path()
        if not path.exists():
            return
        try:
            data = tomllib.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            print(f"读取配置失败: {exc}")
            return

        power = data.get("power", {})
        self._select_combo_by_data(self.combo_port, power.get("port", ""))
        if "baudrate" in power:
            self.combo_baudrate.setCurrentText(str(power["baudrate"]))
        if "address" in power:
            self.spin_address.setValue(int(power["address"]))
        if "voltage" in power:
            self.input_voltage.setValue(float(power["voltage"]))
        if "current" in power:
            self.input_current.setValue(float(power["current"]))

        temp = data.get("temperature", {})
        self._select_combo_by_data(self.temp1_combo_port, temp.get("sensor1_port", ""))
        self._select_combo_by_data(self.temp2_combo_port, temp.get("sensor2_port", ""))

        pid = data.get("pid", {})
        if "target_temp" in pid:
            self.spin_target_temp.setValue(float(pid["target_temp"]))
        if "safety_temp" in pid:
            self.spin_safety_temp.setValue(float(pid["safety_temp"]))
        if "kp" in pid:
            self.spin_kp.setValue(float(pid["kp"]))
        if "ki" in pid:
            self.spin_ki.setValue(float(pid["ki"]))
        if "kd" in pid:
            self.spin_kd.setValue(float(pid["kd"]))
        if "max_current" in pid:
            self.spin_max_current.setValue(float(pid["max_current"]))
        if "max_voltage" in pid:
            self.spin_max_voltage.setValue(float(pid["max_voltage"]))
        if "control_interval" in pid:
            self.spin_interval.setValue(float(pid["control_interval"]))
        if "fusion_mode" in pid:
            self.combo_fusion_mode.setCurrentIndex(int(pid["fusion_mode"]))
        if "control_mode" in pid:
            self.combo_control_mode.setCurrentIndex(int(pid["control_mode"]))

        window = data.get("window", {})
        width = int(window.get("width", 0))
        height = int(window.get("height", 0))
        if width >= 800 and height >= 600:
            self.resize(width, height)

    def _toml_format_value(self, value) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, int):
            return str(value)
        if isinstance(value, float):
            return repr(value)
        if isinstance(value, str):
            escaped = value.replace("\\", "\\\\").replace('"', '\\"')
            escaped = escaped.replace("\n", "\\n").replace("\t", "\\t")
            return f"\"{escaped}\""
        return "\"\""

    def _toml_dump(self, data: dict) -> str:
        lines = []
        for section, values in data.items():
            lines.append(f"[{section}]")
            for key, value in values.items():
                if value is None:
                    continue
                lines.append(f"{key} = {self._toml_format_value(value)}")
            lines.append("")
        return "\n".join(lines).rstrip() + "\n"

    def _save_config(self) -> None:
        data = {
            "power": {
                "port": self.combo_port.currentData() or "",
                "baudrate": int(self.combo_baudrate.currentText()),
                "address": int(self.spin_address.value()),
                "voltage": float(self.input_voltage.value()),
                "current": float(self.input_current.value()),
            },
            "temperature": {
                "sensor1_port": self.temp1_combo_port.currentData() or "",
                "sensor2_port": self.temp2_combo_port.currentData() or "",
            },
            "pid": {
                "target_temp": float(self.spin_target_temp.value()),
                "safety_temp": float(self.spin_safety_temp.value()),
                "kp": float(self.spin_kp.value()),
                "ki": float(self.spin_ki.value()),
                "kd": float(self.spin_kd.value()),
                "max_current": float(self.spin_max_current.value()),
                "max_voltage": float(self.spin_max_voltage.value()),
                "control_interval": float(self.spin_interval.value()),
                "fusion_mode": int(self.combo_fusion_mode.currentIndex()),
                "control_mode": int(self.combo_control_mode.currentIndex()),
            },
            "window": {
                "width": int(self.width()),
                "height": int(self.height()),
            },
        }

        try:
            path = self._get_config_path()
            path.write_text(self._toml_dump(data), encoding="utf-8")
        except Exception as exc:
            print(f"保存配置失败: {exc}")



def main():
    app = QApplication(sys.argv)
    app.setApplicationName("多设备控制系统")
    app.setOrganizationName("AI Team")

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
