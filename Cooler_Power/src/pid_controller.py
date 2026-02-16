#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
PID 控制器与自动整定器

提供:
- PIDController: 增强型 PID 温度控制器
- PIDAutoTuner: 阶跃响应自动整定

从 main_window.py 抽取为独立模块，供 UI 和 HardwareWorker 共用。
"""

import time


# ==================== PID 控制器 ====================

class PIDController:
    """
    增强型 PID 温度控制器，用于半导体制冷片 (TEC) 电流控制

    特性：
    - 微分项低通滤波，抑制测量噪声
    - 反向计算抗积分饱和
    - 输出变化率限制，保护 TEC
    - 自动时间步长计算
    """

    def __init__(self, kp=1.0, ki=0.05, kd=0.5, output_min=0.0, output_max=7.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0
        self._output = 0.0
        self._last_time = None
        self.reverse = False           # True 表示制热模式（翻转 PID 误差方向）
        self.derivative_filter = 0.3   # 微分滤波系数 (0~1, 越小越平滑)
        self.output_rate_limit = 2.0   # 输出变化率限制 (A/s)

    def reset(self):
        """重置控制器状态"""
        self._integral = 0.0
        self._last_error = 0.0
        self._last_derivative = 0.0
        self._output = 0.0
        self._last_time = None

    def compute(self, setpoint: float, measured: float) -> float:
        """
        计算 PID 输出

        Args:
            setpoint: 目标温度
            measured: 当前测量温度

        Returns:
            输出电流值 (A)
        """
        now = time.time()
        if self._last_time is None:
            dt = 1.0
        else:
            dt = max(0.01, now - self._last_time)
        self._last_time = now

        # 制冷模式: error>0 表示温度高于目标，需加大电流制冷
        # 制热模式(reverse): error>0 表示温度低于目标，需加大电流制热
        if self.reverse:
            error = setpoint - measured
        else:
            error = measured - setpoint

        # P 比例项
        p_term = self.kp * error

        # I 积分项（带抗饱和）
        self._integral += error * dt
        i_limit = self.output_max / max(self.ki, 0.001)
        self._integral = max(-i_limit, min(i_limit, self._integral))
        i_term = self.ki * self._integral

        # D 微分项（带低通滤波）
        if dt > 0:
            raw_derivative = (error - self._last_error) / dt
        else:
            raw_derivative = 0.0
        alpha = self.derivative_filter
        filtered_d = alpha * raw_derivative + (1 - alpha) * self._last_derivative
        self._last_derivative = filtered_d
        d_term = self.kd * filtered_d

        self._last_error = error

        # 计算原始输出
        raw_output = p_term + i_term + d_term

        # 输出限幅
        new_output = max(self.output_min, min(self.output_max, raw_output))

        # 抗积分饱和：输出被限幅时回退积分
        if abs(new_output - raw_output) > 0.001 and abs(self.ki) > 0.0001:
            self._integral -= (raw_output - new_output) / self.ki * 0.5

        # 输出变化率限制
        if self.output_rate_limit > 0 and dt > 0:
            max_change = self.output_rate_limit * dt
            change = new_output - self._output
            if abs(change) > max_change:
                new_output = self._output + max_change * (1 if change > 0 else -1)

        self._output = new_output
        return self._output


# ==================== PID 自动整定器 ====================

class PIDAutoTuner:
    """
    PID 自动整定器 - 阶跃响应法 (Step Response)

    专为慢速热系统优化：
    1. 施加最大制冷电流（阶跃输入）
    2. 记录温度随时间的响应曲线
    3. 用两点法识别滞后时间 L 和时间常数 T
    4. 用 Cohen-Coon 公式计算 PID 参数

    优势：只需一次阶跃响应，无需多次振荡，适合慢系统
    """

    class State:
        IDLE = 'idle'
        STEP_RESPONSE = 'step_response'  # 阶跃响应采集
        ANALYZING = 'analyzing'          # 分析中
        DONE = 'done'
        FAILED = 'failed'

    def __init__(self, setpoint: float, output_high: float, output_low: float = 0.0,
                 max_time: float = 300, min_change: float = 3.0, heating: bool = False):
        """
        Args:
            setpoint: 目标温度
            output_high: 最大制冷/制热电流
            output_low: 最小电流
            max_time: 最大整定时间 (秒)
            min_change: 最小温度变化量才认为有效 (℃)
            heating: True 表示制热模式
        """
        self.setpoint = setpoint
        self.output_high = output_high
        self.output_low = output_low
        self.max_time = max_time
        self.min_change = min_change
        self.heating = heating

        self.state = self.State.IDLE
        self._start_time = 0
        self._current_output = 0.0
        self._initial_temp = None

        # 阶跃响应数据
        self._time_data = []
        self._temp_data = []
        self._step_start_time = 0
        self._step_applied = False
        self._settle_wait = 5.0  # 等待温度稳定的初始时间(s)

        # 整定结果
        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        self.dead_time = 0.0
        self.time_constant = 0.0
        self.message = ""

    def start(self):
        """开始自动整定"""
        self.state = self.State.STEP_RESPONSE
        self._start_time = time.time()
        self._current_output = 0.0  # 先输出0，稳定几秒
        self._initial_temp = None
        self._time_data = []
        self._temp_data = []
        self._step_applied = False
        self._step_start_time = 0
        self.message = "自动整定：等待温度稳定..."

    def step(self, measured: float) -> float:
        """整定控制一步，返回应输出的电流值"""
        elapsed = time.time() - self._start_time

        if elapsed > self.max_time:
            # 超时前尝试分析已有数据
            if len(self._time_data) > 20:
                self._analyze_response()
                if self.state == self.State.DONE:
                    return 0.0
            self.state = self.State.FAILED
            self.message = f"自动整定超时 ({self.max_time:.0f}s)，已采用经验参数"
            self._use_fallback_params(measured)
            return 0.0

        if self.state == self.State.STEP_RESPONSE:
            # 阶段 1：稳定期 (前几秒不加电流，记录初始温度)
            if not self._step_applied:
                if elapsed < self._settle_wait:
                    self._current_output = self.output_low
                    self.message = f"自动整定：稳定采样中 ({elapsed:.0f}/{self._settle_wait:.0f}s)..."
                    return self._current_output
                else:
                    # 记录初始温度，施加阶跃
                    self._initial_temp = measured
                    self._step_applied = True
                    self._step_start_time = time.time()
                    self._current_output = self.output_high
                    self.message = f"自动整定：阶跃已施加 (初始 {measured:.2f}℃, 电流 {self.output_high:.1f}A)..."
                    return self._current_output

            # 阶段 2：采集响应数据
            t = time.time() - self._step_start_time
            self._time_data.append(t)
            self._temp_data.append(measured)
            self._current_output = self.output_high

            temp_change = abs(self._initial_temp - measured)
            self.message = (f"自动整定：采集响应 ({elapsed:.0f}s) | "
                            f"温变: {temp_change:.2f}℃ | 点数: {len(self._time_data)}")

            # 检查是否可以分析：至少降温 min_change 度且数据点足够
            if temp_change >= self.min_change and len(self._time_data) >= 30:
                self._analyze_response()
                if self.state == self.State.DONE:
                    return 0.0
            # 如果温度变化超过 60% 目标距离且数据点够，也可以分析
            elif temp_change >= self.min_change * 0.6 and len(self._time_data) >= 60:
                self._analyze_response()
                if self.state == self.State.DONE:
                    return 0.0

            return self._current_output

        return 0.0

    def _analyze_response(self):
        """分析阶跃响应曲线，识别系统参数"""
        try:
            if len(self._time_data) < 10 or self._initial_temp is None:
                return

            times = self._time_data
            temps = self._temp_data
            T0 = self._initial_temp
            T_final = temps[-1]
            # 制冷: 温度下降; 制热: 温度上升
            if self.heating:
                delta_T = T_final - T0  # 温度上升量（正值）
            else:
                delta_T = T0 - T_final  # 温度下降量（正值）

            if delta_T < 0.5:
                return  # 变化太小

            # 用两点法识别一阶模型
            # 找 28.3% 和 63.2% 的时间点
            if self.heating:
                target_283 = T0 + delta_T * 0.283
                target_632 = T0 + delta_T * 0.632
            else:
                target_283 = T0 - delta_T * 0.283
                target_632 = T0 - delta_T * 0.632

            t_283 = None
            t_632 = None

            for i in range(len(temps)):
                if self.heating:
                    if t_283 is None and temps[i] >= target_283:
                        t_283 = times[i]
                    if t_632 is None and temps[i] >= target_632:
                        t_632 = times[i]
                else:
                    if t_283 is None and temps[i] <= target_283:
                        t_283 = times[i]
                    if t_632 is None and temps[i] <= target_632:
                        t_632 = times[i]

            if t_283 is None or t_632 is None:
                # 还没达到63.2%，尝试用已有数据估算
                if t_283 is not None:
                    # 只达到28.3%，估算tau
                    tau_est = t_283 * 2.5  # 粗略估计
                    L_est = t_283 * 0.5
                    self.dead_time = max(1.0, L_est)
                    self.time_constant = max(5.0, tau_est)
                else:
                    return  # 数据不足
            else:
                # 两点法公式
                self.time_constant = max(5.0, 1.5 * (t_632 - t_283))
                self.dead_time = max(1.0, t_632 - self.time_constant)

            # 系统增益 K = delta_T / delta_U
            delta_U = self.output_high - self.output_low
            K = delta_T / max(delta_U, 0.01)
            L = self.dead_time
            T = self.time_constant

            # Cohen-Coon 公式 (偏保守版，适合热系统)
            r = L / T
            if r < 0.01:
                r = 0.01

            self.kp = (1.0 / K) * (T / L) * (0.9 + r / 12.0)
            ti = L * (30.0 + 3.0 * r) / (9.0 + 20.0 * r)
            td = L * 4.0 / (11.0 + 2.0 * r)

            self.ki = self.kp / ti if ti > 0 else 0
            self.kd = self.kp * td

            # 安全限制
            self.kp = max(0.1, min(15.0, self.kp))
            self.ki = max(0.001, min(1.0, self.ki))
            self.kd = max(0.0, min(15.0, self.kd))

            self.state = self.State.DONE
            self.message = (f"✅ 整定完成! K={K:.2f}, L={L:.1f}s, T={T:.1f}s → "
                            f"Kp={self.kp:.3f}, Ki={self.ki:.4f}, Kd={self.kd:.3f}")

        except Exception as e:
            self.state = self.State.FAILED
            self.message = f"整定计算错误: {e}"

    def _use_fallback_params(self, last_temp: float):
        """整定失败时，根据已有数据计算经验参数"""
        if self._initial_temp is not None and len(self._time_data) > 5:
            delta_T = abs(self._initial_temp - last_temp)
            elapsed_step = self._time_data[-1] if self._time_data else 60
            delta_U = self.output_high

            if delta_T > 0.5 and elapsed_step > 10:
                K = delta_T / max(delta_U, 0.01)
                # 估算时间常数
                T_est = elapsed_step * 0.8
                L_est = elapsed_step * 0.1

                self.kp = max(0.3, min(5.0, 0.9 * T_est / (K * L_est)))
                self.ki = max(0.005, min(0.5, self.kp / (T_est * 1.2)))
                self.kd = max(0.0, min(5.0, self.kp * L_est * 0.5))

                self.state = self.State.DONE
                self.message = (f"✅ 经验估算: ΔT={delta_T:.1f}℃/{elapsed_step:.0f}s → "
                                f"Kp={self.kp:.3f}, Ki={self.ki:.4f}, Kd={self.kd:.3f}")
                return

        # 最后回退到保守默认参数
        self.kp = 1.5
        self.ki = 0.02
        self.kd = 2.0
        self.state = self.State.DONE
        self.message = "⚠ 数据不足，使用保守默认参数 (Kp=1.5, Ki=0.02, Kd=2.0)"
