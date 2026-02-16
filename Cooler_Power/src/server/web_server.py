#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
FastAPI Web 服务端

提供 REST API 和 WebSocket 实时数据推送，
实现设备远程监控与控制。

端口: 8080 (可配置)
"""

import asyncio
import json
import math
import time
import threading
from pathlib import Path
from typing import List

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

from .data_bridge import get_bridge

# ==================== 数据模型 ====================

class SetVoltageRequest(BaseModel):
    voltage: float

class SetCurrentRequest(BaseModel):
    current: float

class ConnectPowerRequest(BaseModel):
    port: str
    baudrate: int = 9600
    address: int = 1

class ConnectTempRequest(BaseModel):
    index: int  # 1 or 2
    port: str

class PIDParamsRequest(BaseModel):
    kp: float = None
    ki: float = None
    kd: float = None
    max_current: float = None
    max_voltage: float = None
    control_interval: float = None
    target_temp: float = None
    safety_temp: float = None
    fusion_mode: int = None
    control_mode: int = None


# ==================== FastAPI App ====================

app = FastAPI(title="电源温控系统 - 远程控制", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)


# ---- REST API ----

@app.get("/")
async def index():
    """返回主页面"""
    html_path = Path(__file__).parent / "templates" / "index.html"
    return HTMLResponse(content=html_path.read_text(encoding='utf-8'))


@app.get("/api/state")
async def get_state():
    """获取设备全局状态"""
    bridge = get_bridge()
    return bridge.get_state()


@app.get("/api/history")
async def get_history():
    """获取温度历史数据"""
    bridge = get_bridge()
    return bridge.get_history()


@app.get("/api/ports")
async def get_ports():
    """获取可用串口列表"""
    bridge = get_bridge()
    bridge.send_command("refresh_ports")
    await asyncio.sleep(0.3)
    return bridge.get_ports()


# ---- 电源控制 ----

@app.post("/api/power/connect")
async def power_connect(req: ConnectPowerRequest):
    bridge = get_bridge()
    bridge.send_command("power_connect", {
        "port": req.port, "baudrate": req.baudrate, "address": req.address
    })
    return {"ok": True, "message": "连接指令已发送"}


@app.post("/api/power/disconnect")
async def power_disconnect():
    bridge = get_bridge()
    bridge.send_command("power_disconnect")
    return {"ok": True, "message": "断开指令已发送"}


@app.post("/api/power/set_voltage")
async def set_voltage(req: SetVoltageRequest):
    bridge = get_bridge()
    bridge.send_command("set_voltage", {"voltage": req.voltage})
    return {"ok": True}


@app.post("/api/power/set_current")
async def set_current(req: SetCurrentRequest):
    bridge = get_bridge()
    bridge.send_command("set_current", {"current": req.current})
    return {"ok": True}


@app.post("/api/power/output_on")
async def output_on():
    bridge = get_bridge()
    bridge.send_command("output_on")
    return {"ok": True}


@app.post("/api/power/output_off")
async def output_off():
    bridge = get_bridge()
    bridge.send_command("output_off")
    return {"ok": True}


# ---- 温度传感器 ----

@app.post("/api/temp/connect")
async def temp_connect(req: ConnectTempRequest):
    bridge = get_bridge()
    bridge.send_command("temp_connect", {"index": req.index, "port": req.port})
    return {"ok": True}


@app.post("/api/temp/disconnect")
async def temp_disconnect(req: ConnectTempRequest):
    bridge = get_bridge()
    bridge.send_command("temp_disconnect", {"index": req.index})
    return {"ok": True}


# ---- PID 控制 ----

@app.post("/api/pid/update_params")
async def update_pid_params(req: PIDParamsRequest):
    params = {k: v for k, v in req.dict().items() if v is not None}
    bridge = get_bridge()
    bridge.send_command("update_pid_params", params)
    return {"ok": True}


@app.post("/api/pid/start")
async def start_pid():
    bridge = get_bridge()
    bridge.send_command("start_control")
    return {"ok": True}


@app.post("/api/pid/stop")
async def stop_pid():
    bridge = get_bridge()
    bridge.send_command("stop_control")
    return {"ok": True}


@app.post("/api/pid/auto_tune")
async def auto_tune():
    bridge = get_bridge()
    bridge.send_command("start_auto_tune")
    return {"ok": True}


@app.post("/api/pid/apply_tune")
async def apply_tune():
    bridge = get_bridge()
    bridge.send_command("apply_tune")
    return {"ok": True}


# ---- WebSocket 实时推送 ----

class ConnectionManager:
    """管理 WebSocket 连接"""

    def __init__(self):
        self.active: List[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.active.append(ws)

    def disconnect(self, ws: WebSocket):
        if ws in self.active:
            self.active.remove(ws)

    async def broadcast(self, data: dict):
        dead = []
        for ws in self.active:
            try:
                await ws.send_json(data)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.disconnect(ws)


ws_manager = ConnectionManager()


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws_manager.connect(ws)
    try:
        while True:
            # 接收客户端指令（可选）
            try:
                text = await asyncio.wait_for(ws.receive_text(), timeout=0.05)
                msg = json.loads(text)
                if 'cmd' in msg:
                    bridge = get_bridge()
                    bridge.send_command(msg['cmd'], msg.get('params', {}))
            except asyncio.TimeoutError:
                pass
            except json.JSONDecodeError:
                pass

            # 推送最新状态
            bridge = get_bridge()
            state = bridge.get_state()
            await ws.send_json({"type": "state", "data": state})

            await asyncio.sleep(0.5)
    except WebSocketDisconnect:
        ws_manager.disconnect(ws)
    except Exception:
        ws_manager.disconnect(ws)


# ==================== 启动函数 ====================

def run_server(host: str = "0.0.0.0", port: int = 8080):
    """在独立线程中启动 Web 服务器"""
    config = uvicorn.Config(app, host=host, port=port, log_level="info")
    server = uvicorn.Server(config)
    server.run()


def start_server_thread(host: str = "0.0.0.0", port: int = 8080) -> threading.Thread:
    """在后台线程启动 Web 服务器"""
    t = threading.Thread(target=run_server, args=(host, port), daemon=True)
    t.start()
    print(f"✅ Web 服务器已启动: http://localhost:{port}")
    return t
