#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CL-500W 可调电源控制器 - MetaGPT 多Agent 协作系统

三个 AI Agent 持续运行并讨论改进软件:
1. 架构师 - 分析代码结构和设计
2. 开发者 - 提出代码修改方案
3. 测试员 - 运行测试并反馈

使用方法:
    python run_agents.py

作者: AI协作团队
日期: 2026-02-05
"""

import asyncio
import sys
import os
from pathlib import Path
import subprocess

# 设置编码
if sys.platform == 'win32':
    os.environ['PYTHONIOENCODING'] = 'utf-8'

# 设置项目路径
project_root = Path(__file__).parent.absolute()
metagpt_path = project_root.parent / "MetaGPT-main"

# 添加路径
sys.path.insert(0, str(metagpt_path))
sys.path.insert(0, str(project_root))

# 设置配置文件路径
os.environ["METAGPT_CONFIG_PATH"] = str(project_root / "config2.yaml")

from metagpt.logs import logger
from metagpt.actions import Action
from metagpt.roles import Role
from metagpt.team import Team
from metagpt.environment import Environment
from metagpt.schema import Message
from metagpt.actions.add_requirement import UserRequirement


# ==================== 项目配置 ====================

PROJECT_ROOT = str(project_root).replace("\\", "/")
PYTHON_PATH = "C:/Users/11/AppData/Local/Programs/Python/Python311/python.exe"

PROJECT_INFO = f"""
CL-500W 电源控制器项目信息:
- 项目路径: {PROJECT_ROOT}
- 主要文件: src/drivers/modbus_rtu.py (MODBUS通信), src/drivers/cl500w_driver.py (驱动), src/ui/main_window.py (界面)
- 测试文件: tests/test_driver.py
- 已知问题: 通信偶尔出现"响应数据过短"和"CRC校验失败"
"""


# ==================== 辅助函数 ====================

def run_tests() -> str:
    """运行单元测试并返回结果"""
    try:
        cmd = f'"{PYTHON_PATH}" -m pytest "{PROJECT_ROOT}/tests/test_driver.py" -v --tb=short'
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=60,
            encoding='utf-8',
            errors='replace',
            cwd=PROJECT_ROOT
        )
        output = result.stdout + result.stderr
        return output[:3000] if len(output) > 3000 else output
    except Exception as e:
        return f"测试执行失败: {e}"


def read_file_content(filepath: str) -> str:
    """读取文件内容"""
    try:
        full_path = f"{PROJECT_ROOT}/{filepath}" if not filepath.startswith(PROJECT_ROOT) else filepath
        with open(full_path, 'r', encoding='utf-8') as f:
            content = f.read()
        lines = content.split('\n')[:100]  # 只取前100行
        return '\n'.join([f"{i+1}: {line}" for i, line in enumerate(lines)])
    except Exception as e:
        return f"读取失败: {e}"


# ==================== 预读取项目代码 ====================

def get_project_context() -> str:
    """获取项目上下文"""
    modbus_code = read_file_content("src/drivers/modbus_rtu.py")
    test_results = run_tests()
    
    return f"""
{PROJECT_INFO}

## 当前测试结果
```
{test_results}
```

## modbus_rtu.py 代码片段 (前100行)
```python
{modbus_code}
```
"""


# ==================== Actions (使用 instruction 模式) ====================

# 架构师的分析行为
ArchitectAnalyze = Action(
    name="ArchitectAnalyze",
    instruction="""
你是软件架构师。分析以下项目信息，找出问题并提出改进建议。

重点分析:
1. MODBUS通信代码的错误处理是否完善
2. 是否有可能导致"响应数据过短"或"CRC校验失败"的问题
3. 代码架构是否合理

请用中文回答，给出具体的分析和建议。
"""
)

# 开发者的代码建议
DeveloperSuggest = Action(
    name="DeveloperSuggest", 
    instruction="""
你是软件开发工程师。根据架构师的分析，提出具体的代码修改方案。

要求:
1. 针对每个问题给出具体的修改建议
2. 说明修改的文件和位置
3. 给出修改后的代码示例

请用中文回答。
"""
)

# 测试员的反馈
TesterFeedback = Action(
    name="TesterFeedback",
    instruction="""
你是测试工程师。根据当前的测试结果和开发者的建议，给出反馈。

要求:
1. 分析当前测试是否全部通过
2. 如果有失败，分析失败原因
3. 对开发者的修改方案给出可行性评估

请用中文回答。
"""
)


# ==================== 主程序 ====================

def print_banner():
    """打印欢迎横幅"""
    print("\n" + "=" * 60)
    print("  CL-500W 电源控制项目 - AI 协作开发系统")
    print("=" * 60)
    print("  [架构师] 分析代码结构和问题")
    print("  [开发者] 提出代码修改方案")
    print("  [测试员] 评估方案并反馈")
    print("=" * 60)


async def run_team_discussion(topic: str) -> None:
    """运行团队讨论"""
    print(f"\n{'='*60}")
    print(f"讨论主题: {topic[:80]}...")
    print(f"{'='*60}\n")
    
    # 获取项目上下文
    context = get_project_context()
    full_topic = f"{topic}\n\n{context}"
    
    # 创建 Actions - 每个角色有自己的 action
    analyze_action = Action(
        name="Analyze",
        instruction="你是软件架构师。分析MODBUS通信代码的问题，特别是'响应数据过短'和'CRC校验失败'的可能原因。用中文给出具体分析。"
    )
    
    suggest_action = Action(
        name="Suggest",
        instruction="你是软件开发工程师。根据架构师的分析，提出具体的代码修改方案。用中文说明修改的文件和代码。"
    )
    
    # 创建角色 - 互相监听对方的 action
    # 注意: watch 参数需要使用 Action 类或实例
    # Alex 需要监听 UserRequirement 来接收初始消息，以及 suggest_action 来听开发者
    architect = Role(
        name="Alex",
        profile="Architect", 
        goal="分析代码问题",
        actions=[analyze_action],
        watch=[UserRequirement, suggest_action]  # 监听用户需求和开发者
    )
    
    developer = Role(
        name="Bob",
        profile="Developer",
        goal="提出修改方案",
        actions=[suggest_action],
        watch=[analyze_action]  # 监听架构师
    )
    
    # 创建环境和团队
    env = Environment(desc="CL-500W电源控制器开发讨论")
    team = Team(
        investment=10.0,
        env=env,
        roles=[architect, developer],
        use_mgx=False
    )
    
    # 运行讨论
    print("开始 AI 协作讨论...\n")
    print("-" * 40)
    try:
        print("正在调用 LLM API (请等待)...\n")
        # 通过 team.run() 发布初始消息并运行
        result = await team.run(idea=full_topic, n_round=4)
        
        print(f"\n运行完成, 结果类型: {type(result)}")
        
        # 从 env 获取历史消息
        if hasattr(team, 'env') and hasattr(team.env, 'history'):
            history = team.env.history
            print(f"\n对话历史 ({len(history.storage) if hasattr(history, 'storage') else 0} 条消息):")
            print("=" * 40)
            
            if hasattr(history, 'storage'):
                for msg in history.storage:
                    if hasattr(msg, 'content'):
                        role_name = getattr(msg, 'sent_from', getattr(msg, 'role', 'Unknown'))
                        content = msg.content
                        # 截断太长的内容
                        if len(content) > 1000:
                            content = content[:1000] + "..."
                        print(f"\n[{role_name}]:\n{content}\n")
                        print("-" * 40)
                        
    except Exception as e:
        logger.error(f"讨论出错: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 60)
    print("讨论完成")
    print("=" * 60)


async def interactive_mode():
    """交互模式"""
    print_banner()
    
    default_topic = "请分析CL-500W电源控制器项目，找出可能导致通信错误的问题，并提出改进方案。"
    
    while True:
        print("\n" + "-" * 40)
        print("选项:")
        print("  1. 运行 AI 团队讨论（分析代码问题）")
        print("  2. 只运行测试")
        print("  3. 查看文件")
        print("  q. 退出")
        print("-" * 40)
        
        try:
            choice = input("\n请选择 [1/2/3/q]: ").strip()
        except EOFError:
            break
        
        if choice == 'q':
            print("\n退出程序。")
            break
        elif choice == '1':
            await run_team_discussion(default_topic)
        elif choice == '2':
            print("\n运行测试...\n")
            output = run_tests()
            print(output)
        elif choice == '3':
            try:
                filepath = input("\n输入文件路径 (如 src/drivers/modbus_rtu.py): ").strip()
                if filepath:
                    content = read_file_content(filepath)
                    print(f"\n{content[:2000]}")
            except EOFError:
                break
        else:
            print("无效选择")


async def main():
    """主入口"""
    if len(sys.argv) > 1 and sys.argv[1] == '--auto':
        # 自动模式
        topic = "请分析CL-500W电源控制器项目的MODBUS通信代码，特别是modbus_rtu.py中的错误处理机制，找出可能导致'响应数据过短'和'CRC校验失败'的问题，并提出具体的改进方案。"
        await run_team_discussion(topic)
    else:
        # 交互模式
        await interactive_mode()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n\n用户中断，退出程序。")
    except Exception as e:
        logger.error(f"程序异常: {e}")
        import traceback
        traceback.print_exc()
