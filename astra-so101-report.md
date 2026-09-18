# GPT-6 Astra & open-source LLM→robot-arm control: actionable findings for an SO-101 plugin

*Compiled 2026-09-18. Facts marked ✅ verified from primary source; ⚠️ unverified/claimed; ❌ disproved.*

## 1. What GPT-6 Astra actually offers for robotics

✅ **No realtime/streaming robot API. No robotics SDK. No "robot arm control" interface.** From the official model page ([developers.openai.com/api/docs/models/gpt-6-astra](https://developers.openai.com/api/docs/models/gpt-6-astra) / [`.md`](https://developers.openai.com/api/docs/models/gpt-6-astra.md)):

| Property | Value |
|---|---|
| Model ID | `gpt-6-astra` (single snapshot, no dated variants) |
| Input / output modalities | **text, image → text only** (no audio/video out) |
| Context | 1,050,000 tokens (922K max input), 128K max output |
| Endpoints | Chat Completions ✅, Responses ✅, Batch ✅; **Realtime ❌, Live `v1/live/sessions` ❌, Fine-tuning ❌, Embeddings ❌, Audio/vision-generation ❌** |
| Features | streaming, structured_outputs, **function_calling**, image_input, web_search, prompt_caching |
| Hosted tools | web_search, file_search, image_generation, code_interpreter, hosted_shell, apply_patch, skills, **computer_use**, mcp, tool_search |
| Price | $10 / $50 per 1M in/out; 2× input above 272K |
| Rate limits | Tier 1: 500 RPM / 500K TPM |
| Knowledge cutoff | Apr 30, 2026 |

**How to actually drive a robot with it:** the *only* supported path is `v1/responses` (or Chat Completions) with `tools` + `image_input`. Two supported transports reduce round-trip cost for agentic loops: SSE streaming, and ✅ **WebSocket mode on `/v1/responses`** ([docs](https://developers.openai.com/api/docs/guides/websocket-mode)), which the docs claim gives *"up to roughly 40% faster end-to-end execution"* for rollouts with 20+ tool calls. That is a **tool-call-latency** claim, not a control-rate one, and there is no published per-call latency figure. Note WebSocket mode is a *Responses*-API multiplexing feature, distinct from the unsupported `v1/realtime`.

⚠️ **Hype to discount:** `dev.to` posts titled "controla brazos robóticos en tiempo real" / "Build a Real-Time Multimodal Robot Arm with GPT-6 Astra in Hours" and n1n.ai's "Deploying GPT-6 Astra Class Multimodal Models on Robotic Arms" are vendor/community content with no latency data and no SDK. Treat "realtime robot arm control" as marketing for "agentic tool-calling loop."
⚠️ The often-cited claim that "Altman confirmed a self-built humanoid robot" ([nbd.com.cn](http://www.nbd.com.cn/articles/2026-09-04/4572291.html)) is a Chinese media round-up, not an OpenAI doc — I could not open `openai.com/index/gpt-6-astra.md` (403), so I flag it unverified.
⚠️ `deploymentsafety.openai.com/gpt-6-astra/*` exists (system card / runtime kill-switch discussion) but I could not verify robotics-specific safety content.

**Practical verdict:** Astra is a ~0.5–3 Hz *high-level planner*. It cannot close a servo loop. Budget a local controller for 20–100 Hz.

## 2. Architecture patterns

**(a) VLM as task planner → skill/pose calls (the dominant pattern).**
- ✅ [cheng-haha/GPT-Policy](https://github.com/cheng-haha/GPT-Policy) (arXiv 2609.19138, paper 2026-09-16): "context compiler" → VLM emits **one structured request per step** → adapter **validates and executes**, returns fresh observations. Adapters do Cartesian targets + waypoint sequences + **per-sample sequential IK checks** + gripper + "backend-specific timing". Camera/state in, tool call out; loop cadence = one VLM round-trip per waypoint.
- ✅ [IliaLarchenko/robot_MCP](https://github.com/IliaLarchenko/robot_MCP): LLM picks semantic deltas every round-trip; local controller interpolates at **100 Hz** (`DEGREES_PER_STEP=1.5`, `STEP_DELAY_SECONDS=0.01`, `MAX_INTERPOLATION_STEPS=150`).
- LLM sits **outside** the servo loop in both.

**(b) VLM emitting low-level joint/EE deltas at N Hz.** ❌ **Not found in any working project.** [zjwzcx/Awesome-Astra-Embodied-AI](https://github.com/zjwzcx/Awesome-Astra-Embodied-AI) (updated 2026-09-17) is the best evidence: of ~35 curated cases, nearly all are **simulation or kinematic replay** with "disclosed ideal-grasp assumptions", "demonstrate specific planned motions **rather than an online policy**", or "trajectory planned by Astra, tracked by a low-level controller (GEAR-SONIC / whole-body controller)". A few real-world cases exist (e.g. learning keyboard typing "through visual feedback in about 40 minutes") but none publishes a control frequency.

**(c) VLA policies (pi0 / SO-101 fine-tunes).** ✅ [D-Robotics/openpi_runtime](https://github.com/D-Robotics/openpi_runtime): quantized pi0 client-server VLA. Published latencies: preprocessing 3.2 ms, **inference 192.5 ms**, action exec 700.5 ms for a 50-step chunk (~70 timesteps) → ~1.4 Hz replanning. Target hardware is the **S600 arm, Ubuntu 24.04 + ROS 2 Jazzy + RDK S600 NPU** — not an SO-101. ✅ [strands-labs/robots](https://github.com/strands-labs/robots) wraps **LeRobot (ACT/Pi0/SmolVLA/Diffusion), GR00T, Cosmos 3, MolmoAct2** behind one policy ABC.

**(d) MCP server exposing robot primitives.** See §4.

## 3. Usable today for an SO-101 + wrist + overhead depth camera

| Project | License | Last commit | SO-101? | Leader arm needed? |
|---|---|---|---|---|
| [IliaLarchenko/robot_MCP](https://github.com/IliaLarchenko/robot_MCP) | ⚠️ **no LICENSE file** | 2025-08-12 (**stale**) | ✅ `so100`/`so101`/`lekiwi` | ❌ no |
| [strands-labs/robots](https://github.com/strands-labs/robots) (`strands-robots`) | ✅ Apache-2.0 | 2026-09-18 (**active**) | ✅ `Robot("so101", mode="real")` | ❌ no; teleop optional |
| [windht/xlerobot-mcp](https://github.com/windht/xlerobot-mcp) | ⚠️ no license found | 2026-04-01 | ❌ **XLeRobot only** | ❌ |
| [Grigorij-Dudnik/RoboCrew](https://github.com/Grigorij-Dudnik/RoboCrew) (`robocrew`) | ⚠️ not verified | 2026-09-16 | ❌ XLeRobot/LeKiwi | ❌ |
| [cheng-haha/GPT-Policy](https://github.com/cheng-haha/GPT-Policy) | ⚠️ **"license intentionally pending"** — no redistribution | 2026-09-17 | ❌ **ARX X5, I2RT/YAM only** | ❌ |
| [dimentary/llm-robotics-playground](https://github.com/dimentary/llm-robotics-playground) | ✅ MIT | 2026-09-17 | ❌ **MuJoCo only** | ❌ |
| [D-Robotics/openpi_runtime](https://github.com/D-Robotics/openpi_runtime) | ⚠️ not verified | 2026-03-12 | ❌ S600 | ❌ |

**Best fit: `strands-robots`.** `Robot("so101", mode="real", port=...)`, Python 3.12+, `uv pip install "strands-robots[sim-mujoco]"` then the `lerobot` extra for hardware; `mode="real"` is an explicit opt-in. It has a LeRobot `Teleoperator()` factory (`so101_leader`, gamepad, keyboard) so a **leader arm is optional**. Note `Robot()` deliberately *refuses* `so101_leader` names.
**`robot_MCP` is the closest thing to a drop-in SO-101 MCP plugin** and its `config.py` already expects three cameras (`front` idx0, `wrist` idx1, `top` idx2 @ 640×360/30fps) — exactly your wrist + overhead layout — but it is a year stale and its README warns LeRobot updates broke compatibility (pin [release v0.0.1](https://github.com/IliaLarchenko/robot_MCP/tree/v0.0.1)).
⚠️ I did **not** verify a working SO-101 depth-camera path in any project; all use OpenCV RGB (`OpenCVCameraConfig`). Depth is your integration work.

## 4. Existing "robot arm as MCP tools" projects

- ✅ **[IliaLarchenko/robot_MCP](https://github.com/IliaLarchenko/robot_MCP)** — FastMCP, stdio/SSE/streamable-HTTP, port 3001. Tools: `get_initial_instructions`, `get_robot_state` (returns JSON state **plus JPEG images from all cameras**), `move_robot(move_gripper_up_mm, move_gripper_forward_mm, tilt_gripper_down_angle, rotate_gripper_counterclockwise_angle, rotate_robot_left_angle)` — all optional floats, semantic Cartesian deltas, and `control_gripper(gripper_openness_pct)` (0–100). Motions are interpolated then images re-captured after a **1.0 s settle delay**.
- ✅ **[windht/xlerobot-mcp](https://github.com/windht/xlerobot-mcp)** — `get_camera_image` (MCP `image` payload + metadata), `move_forward/backward`, `turn_left/right`, `strafe_*`, `stop_wheels`, `turn_head_yaw/pitch`, `set_arm_position`, `read_arm_present_position`, `save_arm_position`, `set_saved_position`, `enable_torque`, `disable_torque`, `get_robot_state`, plus a `robot://state` resource. **XLeRobot only.**
- ⚠️ **[SO101 Robot MCP Control Protocol](https://www.mcpworld.com/en/detail/a236a3188dc4cb6df194fbc4d7afa94e)** — listed on a Chinese MCP directory; I could not retrieve its tool schema or a source repo. **Unverified; treat as a listing, not a usable artifact.**
- ⚠️ [abdrah3Em/Robot-MCP](https://github.com/abdrah3Em/Robot-MCP) — surfaced in search, contents not verified.

## 5. Safety/interlock practices actually implemented

- **Dry-run / read-only mode:** ✅ `robot_MCP` has `self.read_only` gating every motion (`_execute_interpolated_move` raises `"Cannot move robot in read-only mode"`), plus `check_positions.py` to inspect state without actuating. ✅ `xlerobot-mcp` defaults to `--backend mock` "so the server is safe to start on any machine."
- **Velocity/step clamp:** ✅ `robot_MCP` per-step interpolation (`1.5°/step` @ 100 Hz) with **per-step normalization-range validation that aborts mid-trajectory** ("would exceed range limits, stopping interpolation").
- **Workspace limits:** ⚠️ **weak everywhere.** `robot_MCP` enforces only per-joint normalized ranges, not Cartesian workspace bounds; it relies on **prompt-level** guardrails ("Max forward reach ~250 mm… Stay >25 cm above ground… Never move with gripper near the ground").
- **Settle-before-observe:** ✅ 1.0 s sleep before re-capturing images.
- **E-stop:** ❌ **no project I found implements one.** `GPT-Policy` explicitly disclaims it: *"`Ctrl+C` requests software cancellation and cleanup; it does not replace a hardware emergency stop."* `strands-robots` comes closest with a **mesh broadcast E-STOP** over its Zenoh peer network, plus a `trust_remote_code` gate.
- **Watchdog:** ❌ none found. **You must add** a host-side heartbeat/lease that torque-disables on client silence.

**Bottom line for your plugin:** use Astra for planning only, behind an OpenAI `mcp`/function-calling tool schema; put a local 20–100 Hz interpolator, joint-limit clamp, workspace box, heartbeat watchdog and hardware e-stop between the model and the SO-101. Ship `strands-robots` (Apache-2.0, actively maintained) as the driver, and mine `robot_MCP` for its tool schema and read-only pattern — not as a dependency, given its staleness and missing license.
