# DS101 首跑手册（Runbook）

按顺序做。每一步都有验证命令和预期输出。**不要跳过验证**——这一步的输出就是下一步的输入。

---

## 第 0 步：先跑两个离线自检（不需要硬件，1 分钟）

```powershell
cd D:\vscode\DS101
.\.venv\Scripts\python.exe calib\selftest.py     # 标定链路
.\.venv\Scripts\python.exe calib\loop_test.py    # 闭环（感知→IK→运动→抓取反馈）
```

预期两行都是 `ALL CHECKS PASSED` / `CLOSED LOOP VERIFIED`。

`loop_test.py` 尤其重要：它用**仿真舵机 + 合成相机**驱动**同一套生产代码**
（`ArmDriver` 注入 `SimServoBus`，不是平行实现），验证：

- 两个方块的位置（40 mm 内）、高度（8 mm 内）被正确恢复
- 抓取点在**斜视修正后**落在真实中心 12 mm 以内（未修正时偏 15.6 mm）
- 腕部到位误差 < 3 mm
- 超出工作区的目标被 IK **拒绝**，而不是半途尝试
- 越限关节被夹紧并如实报告 `clamped`
- 夹爪夹空气 vs 夹到物体的结论不同（这是无力量传感时**唯一的抓取反馈**）

如果这两个不通过，先修代码，别去动硬件。

**想看 Agent 在零硬件下驾驶机械臂**（会启动仿真 bridge，不碰 COM10）：
```powershell
.\.venv\Scripts\python.exe bridge\server.py --simulate
```
然后让 DSH 依次调 `arm_status` / `arm_scene` / `arm_plan` / `arm_move_to` / `arm_gripper`。

---

## ⚠️ 最重要的实测发现：板子必须朝相机倾斜

这一条是本轮最有价值的结论，也是你最容易踩的坑。

顶视相机在桌面之上 610 mm、俯角 16°，垂直视场只有 43.4°，所以它只能看到桌面上一条约 **460–700 mm 深**的带。
而一块**平放**在桌上的板子，相对相机是 **74° 斜入射**，画面里被压缩 **3.63 倍**——
角点检测在这种角度下基本失效（真实相机也一样，不是仿真问题）。

实测验证（`framing_check` 的数值输出）：

| 板子 | 摆放 | 结果 |
|---|---|---|
| 200×275 mm（8×11 @25 mm） | 平放，x=460 mm | ❌ 像素框 x[-170, 81] y[193, 515]，出画 |
| 200×275 mm | **倾斜 50°**，x=120 mm | ✅ 像素框 x[270, 412] y[149, 399]，完美入画 |
| 300×390 mm（你手上那块） | 平放 | ❌ **怎么摆都装不进画幅** |

**所以：**
1. **内参拍摄时把板子立起来、朝相机倾斜**（约 40–60°），别平放。
2. 300×390 那块板太大，建议**另打一张小的**——`calib/boards/charuco_wrist_8x11_20mm.png` 就是为此准备的，
   或者接受把相机抬高（抬高后要重新量高度）。
3. `calib/extrinsics.py` 现在会**先做画幅预检，不合适就直接拒跑**，不会再浪费一次拍摄：
```
REFUSING to capture: the board does not fit in the frame. Fix the framing first
```

---

## 现在立刻要做的一件事

**拔掉 CH343 串口线（机械臂那根 USB），等 3 秒，插回。**（如果 COM10 报 PermissionError 才需要）

> 本轮已实测：**拔插后 COM10 恢复正常，D435 也换到了 USB 3.2**，两个问题都解决了。
> 以后**永远不要强杀 bridge**，用 `POST /shutdown`。

验证：
```powershell
cd D:\vscode\DS101
.\.venv\Scripts\python.exe bridge\servo_bus.py --port COM10
```
预期：6 个舵机在线，`preflight: no blocking problems`。

---

## 阶段 0：物理准备（人类）

### 0.1 装急停开关 ⚠️ 先做这个
把物理开关串在**舵机电源**上。所有开源方案都没有急停，GPT-Policy 自己声明
"Ctrl+C 不是急停"。在装好之前不要跑任何自主动作。

### 0.2 ~~D435 换 USB 3.0~~ ✅ 已完成
已实测 `usb_type_descriptor = 3.2`，1080p@30 可用。

### 0.3 量 3 个尺寸，写在纸上
| 量什么 | 记号 | 容差 |
|---|---|---|
| 镜头中心 → 桌面（垂直） | `H_cam` | ±5 mm |
| 臂底座竖直轴 → 相机（前后） | `D_cam` | ±5 mm |
| 肩部关节中心 → 桌面 | `H_shoulder` | ±2 mm |

这三个值直接喂给外参脚本的 `--camera-forward-mm / --camera-height-mm / --table-z-mm`。

### 0.4 标定板
```powershell
cd D:\vscode\DS101
.\.venv\Scripts\python.exe calib\make_boards.py
```
已生成两张（254 DPI，100% 缩放打印，**不要选"适合页面"**）：
- `calib/boards/charuco_table_10x13_30mm.png` — 300×390 mm（**注意：这块太大，见上面的警告**）
- `calib/boards/charuco_wrist_8x11_20mm.png` — 160×220 mm（**推荐用于顶视**）

打完后卡尺复核方格，然后**裱在亚克力/铝板上**。

### 0.5 确认哪个是腕部相机
三个 UVC 相机都能被 OpenCV 打开（索引 0/1/3）。**必须靠"动一下机械臂看哪张图在动"确认**。
bridge 默认 `--wrist-index 3`。

### 0.6 清空桌面
实测桌面拟合内点率只有 **0.50**（合成理想情况是 0.967），说明视野里有大量非桌面内容。
标定前清空。

---

## 阶段 1：关节零位标定（阻塞一切）

```powershell
cd D:\vscode\DS101
.\.venv\Scripts\python.exe calib\arm_zero.py --port COM10
```

工具会打印每个关节的**行程中点角度**，你需要手动把臂摆到那个姿态：

| 关节 | 目标角度 |
|---|---|
| shoulder_pan | 179.9° |
| shoulder_lift | 180.0° |
| elbow_flex | 175.0° |
| wrist_flex | 180.0° |
| wrist_roll | 182.8° |
| gripper | 230.0° |

> 力矩是关的，臂可自由扳动。有主臂的话，**两只臂摆成同一姿态**，以后遥操作才对得上。

**验证（必做）**：把臂扳到两个不同姿态，用卷尺量夹爪尖到肩部轴线的距离，
和 FK 预测对比，200 mm 行程内应差几毫米。若某关节符号反了（`direction_sign`），
现在就得发现，不能等到抓取时。

---

## 阶段 2：内参标定（板子要倾斜！）

拍 20–30 张，覆盖画面四角、带 ±30° 俯仰**和板子倾角变化**，存到 `calib/captures/intrinsics/`。

```powershell
.\.venv\Scripts\python.exe calib\intrinsics.py calib\captures\intrinsics --board table --out calib\out\overhead_intrinsics.json
```

预期：`RMS reprojection error < 0.3 px`。**> 1.0 px 就是失败**，重拍。

> 拍摄时必须关掉 D435 红外投射器（IR 散斑毁角点检测）。`extrinsics.py` 默认已关。
> **深度变化也必须拍**：所有照片都在同一距离时，焦距和距离会互相抵消，
> 自检里就实测到过这种退化——焦距错了 89% 而 RMS 仍然小于 0.5 px。

---

## 阶段 3：手眼外参（含俯角）

```powershell
.\.venv\Scripts\python.exe calib\extrinsics.py `
  --intrinsics calib\out\overhead_intrinsics.json `
  --board-x-mm <X> --board-y-mm <Y> --board-yaw-deg <角度> --board-pitch-deg 50 `
  --table-z-mm -127.4 `
  --camera-forward-mm 400 --camera-height-mm 610 --camera-pitch-deg 16
```

脚本会**先做画幅预检**，不合适直接拒跑（返回 exit 2）并打印可见桌面带的范围，
告诉你要把板子挪到哪。预检通过后扫 25 帧，反解相机安装参数。

**和阶段 0.3 的实测值对比**：
- `pitch_from_nadir_deg` 应接近 16°
- `camera_height_above_table_mm` 应接近 `H_cam`

**差超过 10 mm 或 2° 就说明板子摆位（T_base_board）填错了**，不是相机的问题。

---

## 阶段 4：跑桥接服务

```powershell
.\.venv\Scripts\python.exe bridge\server.py --allow-motion
```

不加 `--allow-motion` 就是**只读模式**（先这样跑一遍更稳）。它启动前会自己检查 COM10。

停：
```powershell
curl.exe -X POST http://127.0.0.1:8731/shutdown
```

---

## 阶段 5：DSH 工具自检

| 工具 | 期望 | 本轮状态 |
|---|---|---|
| `arm_status` | 关节角、工具位姿、电压、温度 | ✅ 已实测通过 |
| `arm_scene` | 桌面平面 + 物体列表 | ✅ 已实测通过（返回 8 个区域） |
| `arm_plan` `target_mm=[200,0,-50]` | `ok: true`，残留 < 1 mm（**不动作**） | ✅ 实测 0.42 mm |
| `arm_move_to` | 需要 `--allow-motion` | ⏸ 待零位标定 |
| `arm_gripper` | 夹爪开合 | ⏸ |
| `arm_estop` | 立刻断力矩 | ⏸ |

---

## 排障速查

| 现象 | 原因 | 解法 |
|---|---|---|
| `COM10 PermissionError(13)` 且没有进程在跑 | 内核里泄漏了句柄 | **拔插 USB 串口线**（或重启） |
| `bridge unreachable` | bridge 没起 | 起 bridge |
| `REFUSING to capture: board does not fit` | 板子出画（最常见） | 按提示**把板子倾斜 40–60°** 并挪到可见带内 |
| 内参 RMS 好但焦距离谱 | 所有照片同一距离（退化） | 加**不同距离**的照片 |
| `no valid depth at that pixel` | 该像素是镜面/黑色/超量程 | 换相邻像素，或用 `arm_scene` 的质心 |
| IK `residual` 大 | 目标超出工作区（半径 50–433 mm） | 用 `arm_plan` 先试 |
| 标定后 FK 和实测差很多 | 零位的 `direction_sign` 反了 | 重跑 `arm_zero.py` |
| 桌面拟合内点率低（<0.7） | 视野里有杂物 | 清空桌面 |
| 关节拒绝动作 | 触发限位/步长/温度/电压预检 | 看返回的 `error`，别调大阈值绕过 |
