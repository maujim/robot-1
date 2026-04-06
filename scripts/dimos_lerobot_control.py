#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""
DimOS-backed LeRobot motor control.

This script wraps the existing SO100/SO101 Feetech motor bus in a DimOS skill
module so an agent can control the arm through chat or MCP tool calls.

Typical usage:
  python scripts/dimos_lerobot_control.py --port /dev/tty.usbmodemXXXX

If you want to expose it from a DimOS checkout, import this file and use the
`lerobot_agentic_mcp` blueprint variable or `build_blueprint(...)`.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import threading
from collections.abc import Sequence
from pathlib import Path
from typing import Any

# Allow local source import without editable install.
REPO_ROOT = Path(__file__).resolve().parents[1]
LOCAL_SRC = REPO_ROOT / "lerobot" / "src"
try:
    if LOCAL_SRC.exists() and str(LOCAL_SRC) not in sys.path:
        sys.path.insert(0, str(LOCAL_SRC))
except OSError:
    pass

try:
    from dimos.agents.annotation import skill
    from dimos.agents.mcp.mcp_client import mcp_client
    from dimos.agents.mcp.mcp_server import McpServer
    from dimos.core.blueprints import autoconnect
    from dimos.core.core import rpc
    from dimos.core.module import Module
    from dimos.core.stream import In, Out
    from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
    from langchain_core.messages.base import BaseMessage
except ImportError as exc:  # pragma: no cover - direct runtime guidance
    raise SystemExit(
        "DimOS is required for this script. Install it first, for example:\n"
        "  uv pip install 'dimos[base]'\n"
        "Then rerun this script."
    ) from exc

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

MOTOR_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "gripper",
    "wrist_roll",
]

LEROBOT_SYSTEM_PROMPT = f"""
You are a cautious motor-control assistant for a 6-axis LeRobot arm connected through DimOS MCP tools.

# SAFETY
- Prioritize human safety and hardware safety.
- Do not move more than necessary.
- Prefer reading state before moving if the user request is ambiguous.
- If the user asks for a large or unclear motion, ask a short clarifying question first.
- Use `stop_all_motion` if the user asks you to stop, freeze, or hold position.
- Never claim a movement happened unless a motor tool returned success.

# ROBOT CONTEXT
- Available joints: {", ".join(MOTOR_NAMES)}.
- The tools operate in raw servo ticks, not Cartesian space.
- `jog_joint` is the safest default for small moves.
- `set_joint_goal` should be used only when the user gives a clear absolute target.
- The gripper is the `gripper` joint.

# RESPONSE STYLE
- Be concise and action-oriented.
- Briefly explain what tool you are about to use when it helps the user trust the action.
"""


def make_bus(port: str) -> FeetechMotorsBus:
    return FeetechMotorsBus(
        port=port,
        motors={
            "shoulder_pan": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "shoulder_lift": Motor(2, "sts3215", MotorNormMode.DEGREES),
            "elbow_flex": Motor(3, "sts3215", MotorNormMode.DEGREES),
            "wrist_flex": Motor(4, "sts3215", MotorNormMode.DEGREES),
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.DEGREES),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
    )


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


class LeRobotMotorSkills(Module):
    """DimOS skill container for safe LeRobot motor control."""

    def __init__(
        self,
        port: str,
        disable_torque_on_exit: bool = False,
        max_jog_ticks: int = 50,
        max_goal_step_ticks: int = 250,
    ) -> None:
        super().__init__()
        self.port = port
        self.disable_torque_on_exit = disable_torque_on_exit
        self.max_jog_ticks = max_jog_ticks
        self.max_goal_step_ticks = max_goal_step_ticks
        self.bus = make_bus(port)
        self._lock = threading.Lock()
        self._mins: dict[str, int] = {}
        self._maxes: dict[str, int] = {}
        self._targets: dict[str, int] = {}

    def _require_motor(self, motor: str) -> str:
        if motor not in MOTOR_NAMES:
            raise ValueError(f"Unknown motor '{motor}'. Valid motors: {', '.join(MOTOR_NAMES)}")
        return motor

    def _read_limits(self) -> None:
        self._mins = {m: int(self.bus.read("Min_Position_Limit", m, normalize=False)) for m in MOTOR_NAMES}
        self._maxes = {m: int(self.bus.read("Max_Position_Limit", m, normalize=False)) for m in MOTOR_NAMES}

    def _read_positions(self) -> dict[str, int]:
        return {m: int(v) for m, v in self.bus.sync_read("Present_Position").items()}

    def _json(self, payload: dict[str, Any]) -> str:
        return json.dumps(payload, sort_keys=True)

    @rpc
    def start(self) -> None:
        super().start()
        with self._lock:
            self.bus._connect(handshake=False)  # noqa: SLF001
            with self.bus.torque_disabled():
                for motor in MOTOR_NAMES:
                    self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            self.bus.enable_torque()
            self._read_limits()
            self._targets = self._read_positions()

    @rpc
    def stop(self) -> None:
        with self._lock:
            if self.bus.is_connected:
                self.bus.disconnect(disable_torque=self.disable_torque_on_exit)
        super().stop()

    @skill
    def list_joints(self) -> str:
        """List all controllable joint names."""
        return self._json({"joints": MOTOR_NAMES})

    @skill
    def get_joint_positions(self) -> str:
        """Read the current raw tick position of every joint."""
        with self._lock:
            positions = self._read_positions()
        return self._json({"positions": positions})

    @skill
    def get_joint_limits(self) -> str:
        """Read the configured raw tick limits for every joint."""
        with self._lock:
            if not self._mins or not self._maxes:
                self._read_limits()
            limits = {
                motor: {"min": self._mins[motor], "max": self._maxes[motor]}
                for motor in MOTOR_NAMES
            }
        return self._json({"limits": limits})

    @skill
    def jog_joint(self, motor: str, delta_ticks: int) -> str:
        """Jog a joint by a small relative amount.

        Args:
            motor: Joint name to move.
            delta_ticks: Relative move in raw ticks. Positive and negative are allowed.
        """
        self._require_motor(motor)
        if abs(delta_ticks) > self.max_jog_ticks:
            return self._json(
                {
                    "ok": False,
                    "error": (
                        f"Requested jog {delta_ticks} exceeds max_jog_ticks={self.max_jog_ticks}. "
                        "Use a smaller jog or update the safety limit."
                    ),
                }
            )

        with self._lock:
            current = int(self.bus.read("Present_Position", motor, normalize=False))
            goal = clamp(current + delta_ticks, self._mins[motor], self._maxes[motor])
            self.bus.write("Goal_Position", motor, goal, normalize=False)
            self._targets[motor] = goal
        return self._json({"ok": True, "motor": motor, "goal_ticks": goal, "mode": "relative"})

    @skill
    def set_joint_goal(self, motor: str, goal_ticks: int) -> str:
        """Set a joint to an absolute raw tick goal with guardrails.

        Args:
            motor: Joint name to move.
            goal_ticks: Absolute raw tick target. Values outside limits are clamped.
        """
        self._require_motor(motor)
        with self._lock:
            current = int(self.bus.read("Present_Position", motor, normalize=False))
            requested_step = abs(goal_ticks - current)
            if requested_step > self.max_goal_step_ticks:
                return self._json(
                    {
                        "ok": False,
                        "error": (
                            f"Requested move step {requested_step} exceeds "
                            f"max_goal_step_ticks={self.max_goal_step_ticks}. "
                            "Use jog_joint for incremental motion or raise the limit intentionally."
                        ),
                    }
                )
            goal = clamp(goal_ticks, self._mins[motor], self._maxes[motor])
            self.bus.write("Goal_Position", motor, goal, normalize=False)
            self._targets[motor] = goal
        return self._json({"ok": True, "motor": motor, "goal_ticks": goal, "mode": "absolute"})

    @skill
    def stop_all_motion(self) -> str:
        """Stop commanded motion by setting each joint goal to its current position."""
        with self._lock:
            positions = self._read_positions()
            for motor, position in positions.items():
                self.bus.write("Goal_Position", motor, position, normalize=False)
            self._targets = positions.copy()
        return self._json({"ok": True, "goal_ticks": positions})

    @skill
    def enable_torque(self) -> str:
        """Enable torque for all motors."""
        with self._lock:
            self.bus.enable_torque()
        return self._json({"ok": True, "torque_enabled": True})

    @skill
    def disable_torque(self) -> str:
        """Disable torque for all motors."""
        with self._lock:
            self.bus.disable_torque()
        return self._json({"ok": True, "torque_enabled": False})


class TerminalChatBridge(Module):
    """Simple terminal chat bridge for direct natural-language control."""

    human_input: Out[str]
    agent: In[BaseMessage]

    def __init__(self) -> None:
        super().__init__()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._stdin_loop,
            name=f"{self.__class__.__name__}-stdin",
            daemon=True,
        )

    @rpc
    def start(self) -> None:
        super().start()

        def _on_agent_message(message: BaseMessage) -> None:
            rendered = self._render_message(message)
            if rendered:
                print(rendered, flush=True)

        self._disposables.add(self.agent.subscribe(_on_agent_message))

        if not sys.stdin.isatty():
            print(
                "[chat] stdin is not a TTY; skipping interactive chat bridge. "
                "Use `dimos agent-send` or another MCP client instead.",
                flush=True,
            )
            return

        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=1.0)
        super().stop()

    def _stdin_loop(self) -> None:
        print(
            "[chat] natural-language control is ready. Type commands, or `/quit` to exit.",
            flush=True,
        )
        while not self._stop_event.is_set():
            try:
                line = input("you> ").strip()
            except EOFError:
                break
            except KeyboardInterrupt:
                print("", flush=True)
                break

            if not line:
                continue
            if line.lower() in {"/quit", "quit", "exit"}:
                print("[chat] exiting interactive prompt. Press Ctrl-C if the process should stop.", flush=True)
                break

            self.human_input.publish(line)

    def _render_message(self, message: BaseMessage) -> str | None:
        if isinstance(message, HumanMessage):
            return None

        text = _message_content_to_text(message.content)
        if not text:
            return None

        if isinstance(message, AIMessage):
            return f"agent> {text}"
        if isinstance(message, ToolMessage):
            name = getattr(message, "name", None) or "tool"
            return f"{name}> {text}"
        return f"{message.__class__.__name__}> {text}"


def _message_content_to_text(content: Any) -> str:
    if isinstance(content, str):
        return content.strip()

    if isinstance(content, Sequence):
        parts: list[str] = []
        for item in content:
            if isinstance(item, str):
                if item.strip():
                    parts.append(item.strip())
                continue
            if isinstance(item, dict) and item.get("type") == "text":
                text = str(item.get("text", "")).strip()
                if text:
                    parts.append(text)
        return "\n".join(parts).strip()

    return str(content).strip()


def _resolve_mcp_server_url(host: str, port: int) -> str:
    client_host = "127.0.0.1" if host in {"0.0.0.0", "::"} else host
    return f"http://{client_host}:{port}/mcp"


def build_blueprint(
    port: str,
    agent_model: str = "gpt-4o",
    mcp_server_url: str = "http://127.0.0.1:9990/mcp",
    include_terminal_chat: bool = False,
    disable_torque_on_exit: bool = False,
    max_jog_ticks: int = 50,
    max_goal_step_ticks: int = 250,
):
    modules = [
        LeRobotMotorSkills.blueprint(
            port=port,
            disable_torque_on_exit=disable_torque_on_exit,
            max_jog_ticks=max_jog_ticks,
            max_goal_step_ticks=max_goal_step_ticks,
        ),
        McpServer.blueprint(),
        mcp_client(
            model=agent_model,
            system_prompt=LEROBOT_SYSTEM_PROMPT,
            mcp_server_url=mcp_server_url,
        ),
    ]

    if include_terminal_chat:
        modules.append(TerminalChatBridge.blueprint())

    return autoconnect(*modules)


lerobot_agentic_mcp = build_blueprint(
    port=os.environ.get("LEROBOT_PORT", "/dev/tty.usbmodemXXXX"),
    agent_model=os.environ.get("LEROBOT_AGENT_MODEL", "gpt-4o"),
    mcp_server_url=_resolve_mcp_server_url(
        os.environ.get("DIMOS_MCP_HOST", "127.0.0.1"),
        int(os.environ.get("DIMOS_MCP_PORT", "9990")),
    ),
    disable_torque_on_exit=os.environ.get("LEROBOT_DISABLE_TORQUE_ON_EXIT", "").lower() in {"1", "true", "yes"},
    max_jog_ticks=int(os.environ.get("LEROBOT_MAX_JOG_TICKS", "50")),
    max_goal_step_ticks=int(os.environ.get("LEROBOT_MAX_GOAL_STEP_TICKS", "250")),
)

lerobot_agentic_chat_mcp = build_blueprint(
    port=os.environ.get("LEROBOT_PORT", "/dev/tty.usbmodemXXXX"),
    agent_model=os.environ.get("LEROBOT_AGENT_MODEL", "gpt-4o"),
    mcp_server_url=_resolve_mcp_server_url(
        os.environ.get("DIMOS_MCP_HOST", "127.0.0.1"),
        int(os.environ.get("DIMOS_MCP_PORT", "9990")),
    ),
    include_terminal_chat=True,
    disable_torque_on_exit=os.environ.get("LEROBOT_DISABLE_TORQUE_ON_EXIT", "").lower() in {"1", "true", "yes"},
    max_jog_ticks=int(os.environ.get("LEROBOT_MAX_JOG_TICKS", "50")),
    max_goal_step_ticks=int(os.environ.get("LEROBOT_MAX_GOAL_STEP_TICKS", "250")),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="DimOS chat and MCP control for LeRobot motors")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/tty.usbmodemXXXX")
    parser.add_argument(
        "--agent-model",
        default=os.environ.get("LEROBOT_AGENT_MODEL", "gpt-4o"),
        help="Hosted or local LLM model name for the DimOS agent, e.g. gpt-4o or ollama:llama3.1",
    )
    parser.add_argument("--mcp-port", type=int, default=9990, help="MCP HTTP port for McpServer")
    parser.add_argument("--mcp-host", default="127.0.0.1", help="MCP bind host")
    parser.add_argument(
        "--no-chat",
        action="store_true",
        help="Disable the built-in terminal chat bridge and expose only MCP/DimOS endpoints",
    )
    parser.add_argument(
        "--disable-torque-on-exit",
        action="store_true",
        help="Disable torque when shutting down the DimOS module",
    )
    parser.add_argument(
        "--max-jog-ticks",
        type=int,
        default=50,
        help="Maximum allowed delta for jog_joint",
    )
    parser.add_argument(
        "--max-goal-step-ticks",
        type=int,
        default=250,
        help="Maximum allowed one-shot step for set_joint_goal",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    os.environ["DIMOS_MCP_PORT"] = str(args.mcp_port)
    os.environ["DIMOS_MCP_HOST"] = args.mcp_host

    if not args.agent_model.startswith("ollama:") and not os.environ.get("OPENAI_API_KEY"):
        raise SystemExit(
            "OPENAI_API_KEY is required for hosted natural-language control. "
            "Set it in the environment or use a local model such as `--agent-model ollama:llama3.1`."
        )

    mcp_server_url = _resolve_mcp_server_url(args.mcp_host, args.mcp_port)

    blueprint = build_blueprint(
        port=args.port,
        agent_model=args.agent_model,
        mcp_server_url=mcp_server_url,
        include_terminal_chat=not args.no_chat,
        disable_torque_on_exit=args.disable_torque_on_exit,
        max_jog_ticks=args.max_jog_ticks,
        max_goal_step_ticks=args.max_goal_step_ticks,
    )
    print(
        f"[startup] MCP server: {mcp_server_url}\n"
        f"[startup] agent model: {args.agent_model}\n"
        "[startup] You can chat in this terminal or use `dimos mcp call ...` from another shell.",
        flush=True,
    )
    blueprint.build().loop()


if __name__ == "__main__":
    main()
