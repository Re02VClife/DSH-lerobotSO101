# DS101 实施进展与准备清单

> 目标定位（2026-09-18 与操作者确认）：**让 DSH 里的 LLM 充当 VLA 的 "L"——机械臂的驾驶员**。
> 不是只做规则抓取，也不是先做模仿学习；是 LLM 看图 → 决策 → 直接驱动机械臂闭环。

## 当前状态（第 3 轮结束时）

| 环节 | 状态 |
|---|---|
| 硬件通信（COM10 / 6 舵机 / D435） | ✅ 实测通过 |
| USB 3 升级 | ✅ `usb_type_descriptor = 3.2` |
| **标定几何（真实尺寸）** | ✅ 相机 625 mm 高 / 前方 440 mm / 俯角 16°；桌面 z = **−115 mm**（肩轴高 115 mm） |
| **标定板（真实尺寸）** | ✅ 重新生成 **12×7 格 × 30 mm = 360×210 mm**，66 内角点；用户告知实际板为 12 竖 7 横 |
| **画幅可行性** | ✅ 新尺寸下可见桌面带 = **720 mm 深**（x ∈ [−150, 570]），360×210 板**平放也能入画** |
| 运动学 FK + IK | ✅ 36/36 目标收敛，最差 1.37 mm |
| 内参标定链路 | ✅ 合成数据恢复误差 **0.0000%** |
| 手眼外参链路 | ✅ 合成数据平移 **0.469 mm**、旋转 **0.0000°** |
| 结构化感知 | ✅ 方块高度精确；**footprint 斜视修正 15.6→3 mm** |
| **闭环（感知→决策→IK→运动→抓取反馈）** | ✅ `calib/loop_test.py` 全部通过，无硬件、无人工 |
| **正式 host composition 插件** | ✅ `plugins/dsh-so101-arm`，`test/load-check.mjs` 10/10 工具通过真 `defineTool` 校验 |
| **操作经验 skill** | ✅ `.dsh/skills/dsh-so101-arm/SKILL.md`（目录已生效） |
| **视觉能力** | ⚠️ 已诊断：**不是配置错误**，DeepSeek 适配器硬编码 text-only；修法与探测器见 `docs/VISION-配置.md` |
| bridge 生命周期 | ✅ `POST /shutdown` 优雅退出，COM10 干净释放 |
| **关节零位标定** | ❌ **仍是唯一阻塞项**，需要你配合摆位 |
| 手眼外参（真实数据） | ❌ 待零位标定后执行 |

**现在可以立刻跑（都不需要硬件）：**
```powershell
cd D:\vscode\DS101
.\.venv\Scripts\python.exe calib\selftest.py        # 标定链路自检
.\.venv\Scripts\python.exe calib\loop_test.py       # 闭环自检
.\.venv\Scripts\python.exe calib\bench_geometry.py  # 用你的真实尺寸算画幅/摆板位置
node plugins\dsh-so101-arm\test\load-check.mjs      # 插件能否被 DSH 加载
.\.venv\Scripts\python.exe bridge\server.py --simulate   # 零硬件跑 DSH 工具闭环
```

---

## 第 3 轮新增

| 文件 | 作用 |
|---|---|
| `plugins/dsh-so101-arm/` | **正式 host composition 插件包**（可开源）：`lib/index.js` + `cordis.patch.yml` + `README.md` + 内置 skill + `test/load-check.mjs` |
| `calib/bench_geometry.py` | 用真实尺寸算可见桌面带 + 板子摆放可行性（改尺寸后先跑这个） |
| `calib/vision_probe.py` | 视觉能力探测器（2×2 已知颜色 PNG，区分"真支持"/"接受但忽略"） |
| `docs/VISION-配置.md` | 视觉诊断与配置步骤 |
| `.dsh/skills/dsh-so101-arm/SKILL.md` | **操作经验 skill** |

### 关键发现：板子尺寸改对了，画幅问题消失

原来按 300×400 板、相机 610 mm 高算，可见桌面带只有 460 mm 深，300 mm 宽的板怎么摆都出画。
用你的真实尺寸（625 mm 高、440 mm 前）重算后：**可见带 720 mm 深**，而且你告知的板实际是
**12×7 格 × 30 mm = 360×210 mm**（不是我之前假设的 10×13）。两者一叠加，**平放也能入画**。

> 所以"必须倾斜板子"这条结论**降级**了：倾斜仍然更好（74° 斜入射的角点定位精度差），
> 但不再是"必须"。`calib/bench_geometry.py` 会直接列出哪些摆法 `fits=True`。

### 关于"视觉"的诊断结论

DSH 的 DeepSeek 适配器（`@deepseek-ai/dsh-llm-deepseek`）**硬编码**了纯文本：

```
lib/index.js:41   if (contentHasImage(blocks)) throw new LlmError(
                    "The DeepSeek chat-completions adapter does not support image content.",
                    "UNSUPPORTED_CONTENT")
lib/index.js:442  inputModalities: ["text"]
```

所以在这个 provider 上，`read_image` **必然**失败——**与模型本身有没有视觉无关**，
DSH 在读模态声明时就把图挡掉了，根本不会发出去。

可修的路线是 `@deepseek-ai/dsh-llm-pi-ai`（已在依赖里，且**改配置立即生效、无需重启**），
它支持每个路由 `defaultInput` 和每个模型 `input`。但**不能盲配**：README 明确写了过度声明的后果是
"图片被持久化进会话日志 → 该模型每轮重发 → 模型选择拒绝切回纯文本模型"。
所以先用 `calib/vision_probe.py` 验证端点真的能读图。

### 关于"操作经验凝练为 skill"的方法论

skill 是**过程性知识**的正确载体（怎么开车）。但它不该承载**状态性知识**。三类知识分开：

| 知识类型 | 载体 | 为什么 |
|---|---|---|
| 过程性：流程、判据、故障模式 | **skill**（`SKILL.md`） | 按需加载，随时间打磨 |
| 状态性：本台标定值、相机内参、工作区 | **工具运行时读文件**（bridge 读 `calib/out/*.json`） | 只能有一个真源；写进 prompt 会腐化 |
| 结构性：能做什么、参数约束 | **工具 schema + 描述** | 每次都在上下文里，且**编辑 schema 比编辑记忆可靠** |

关键取舍：**不要把经验塞进 system prompt**。prompt 里的数字会随会话腐化、随版本漂移、
且上下文变长后会被压缩掉；而工具每次调用都重新读文件，永远是真的。
所以设计原则是——**凡是我可能记错的，都让工具替我记住。**
本轮所有几何常量（`table_z_mm`、相机位置、板尺寸）都走这条路。

---

## 一、已建成并实测通过的部分（可直接运行）

| 文件 | 作用 | 实测结果 |
|---|---|---|
| `bridge/servo_bus.py` | 舵机总线，**默认只读**，含限位/步长/温度/电压预检 | 6 舵机在线，preflight 无告警 |
| `bridge/cameras.py` | D435 对齐 RGB-D + 腕部相机 + 反投影 | 内参读取正确 |
| `bridge/kinematics.py` | 官方 URDF 的 FK + 阻尼最小二乘 IK | **36/36 桌面目标收敛，最差 1.37 mm** |
| `bridge/driver.py` | 运动底座：tick↔度、限位夹紧、舵机内置轨迹、心跳看门狗 | 到位残差 0.0°，三步可达 |
| `bridge/server.py` | 本地 bridge 服务（HTTP，标准库） | 全端点通过；含端口预检 + `/shutdown` + `--simulate` |
| `bridge/sim.py` | 仿真舵机总线（含轨迹积分、抓取失速、温升） | 闭环测试通过 |
| `bridge/fake_camera.py` | 仿真顶视相机（逐像素解析求交） | 三个方块位置/高度全对 |
| `calib/perception.py` | RANSAC 桌面拟合 + 物体提取 + 抓取点计算 | 高度精确，斜视修正后平面误差 0.4 mm |
| `calib/intrinsics.py` | ChArUco 内参标定 | 自检验证 |
| `calib/extrinsics.py` | 手眼外参 + 画幅预检 | 自检验证 |
| `calib/arm_zero.py` | 关节零位标定（soft / homing） | 流程与文件生成通过 |
| `calib/make_boards.py` | 生成可打印 ChArUco 板 | 2 张 254 DPI PNG |
| `calib/scene_diag.py` | 场景几何诊断 | 已出报告 |
| `calib/selftest.py` | 离线端到端标定链路自检 | **全部通过** |
| `calib/loop_test.py` | **离线闭环自检（感知→IK→运动→抓取反馈）** | **全部通过** |

### 闭环测试当场抓出的 6 个真 bug

`calib/loop_test.py` 第一次跑就暴露了 6 个问题，全是真的：

1. **`set_gripper` 只发一条被限速的指令** —— 与 `goto_joints` 同一个 bug。夹爪只走 15° 就返回，
   于是"夹到东西会失速"这个唯一的抓取反馈信号完全失效。现在走同一条多步路径。
2. **`goto_joints` 只发一条指令却返回 `ok: true`** —— 15°/条限速下，`go_home()` 走了 24 条指令还在半路，
   但报告成功。现在迭代到收敛，并明确区分 `reached` / `clamped` / `budget_exhausted`。
3. **`tool_target_for_grasp` 米/毫米混用** —— 产出小 1000 倍的腕部目标，IK "成功"求解，
   只有机械臂没走到位才看得出来。
4. **仿真舵机不置 Moving 标志** —— `wait_until_settled` 立刻返回，仿真**永远不会动**。
5. **合成场景射线未归一化** —— 重建点不在桌面上，把平面看成斜面，凭空造出两个幻影障碍。
6. **斜视顶面质心偏差** —— 相机 74° 斜视时，看到的顶面质心比物体真实中心**朝相机偏 15.6 mm**。
   夹爪会夹在物体中心后面。新增 `footprint_centre_base_mm()` 修正到 **0.4 mm**。
7. **`look_at` 与 `invert()` 帧约定不一致** —— 配对的矩阵不再是互逆的
   （`R_bc @ eye + t_cb ≠ 0`），但**往返仍然精确**（误差在两个错配矩阵间抵消），
   所以所有组合路径的一致性检查都通过，而中间量是错的——制造了 70 mm 的位置误差。

### 关键实现决策

1. **不用 `cv2.calibrateHandEye`**。本机 OpenCV 5.0.0 里它**不存在**。
2. **LLM 绝不进伺服环**。
3. **不用 VLM 做几何**。感知走确定性算法 → 直接给毫米级数字。
4. **URDF 的 `baseframe_frame` 不是臂底座**，偏了 `(-287, -336, +127) mm`，已归正。
5. **bridge 是硬件唯一所有者**，用 `/shutdown` 优雅退出。
6. **仿真复用生产代码路径**（`ArmDriver` 通过注入 `bus` 使用 `SimServoBus`），
   不是平行实现——所以仿真通过意味着逻辑正确，剩下的风险只剩机械（重复性、回差、台面水平度）。

---

## 二、实测发现的硬约束（必须处理）

| 发现 | 数值 | 影响 |
|---|---|---|
| ~~D435 在 USB 2.1~~ | **已升级到 USB 3.2** ✅ | 1080p@30 可用 |
| 顶视相机俯角 | 桌面法向量 → **16.03°** 离天顶（你的确认一致） | 标定已按斜视建模 |
| **相机看板子的入射角** | **74°**（离垂直入射），画面被压缩 **3.63×** | ⚠️ **平板放在桌上拍不好内参**，必须把板**朝相机倾斜**（`board_placement` 的 `pitch_deg`） |
| 300×390 板能否进画幅 | **装不下**（600 mm 高时可视带约 460 mm 深） | 需换小板或抬高相机 |
| 可达工作区 | 半径 50–433 mm | 相机在 +400 mm 前方 |
| 机械臂从未标定 | 无 lerobot 标定文件 | **阻塞一切** |
| 供电 | 4.4–4.5 V | 4.2 V 附近会掉力矩 |
| 桌面拟合内点率 | 实测 0.50–0.51（合成测试 0.967） | ⚠️ 视野里有非桌面内容，标定前需清空 |

---

## 三、必须由你（人类）完成的物理步骤

### 步骤 A：断电急停开关 —— **先做这个**
串在舵机电源上的物理开关。所有开源方案都没有，GPT-Policy 自己声明 "Ctrl+C 不是急停"。
在装好之前不要跑任何自主动作。

### 步骤 B：D435 换到 USB 3.0
插蓝色口 + 用随相机附带的 USB-C/USB3 线。验证：
```powershell
cd D:\vscode\DS101; .\.venv\Scripts\python.exe -c "import pyrealsense2 as rs; [print(d.get_info(rs.camera_info.usb_type_descriptor)) for d in rs.context().query_devices()]"
```
出现 `3.2` 或 `3.1` 即成功。这一步能把彩色从 640×480@15 提到 1280×720@30。

### 步骤 C：量 3 个尺寸并记下来（标定必需）
1. **相机到桌面的垂直高度**（镜头中心 → 桌面），卷尺量，±5 mm。
2. **臂底座竖直轴线到相机的水平距离**（前后方向），±5 mm。
3. **肩部旋转关节中心到桌面的高度**（用于把桌面 z 归一化），±2 mm。

> 反推估算：URDF 里肩关节比底座低 127.4 mm，所以若桌面就在底座平面，桌面在臂坐标系里是 z = -127.4 mm。
> 但**必须实测**，我不用推算值做标定。

### 步骤 D：标定板
1. 打印 `calib/boards/charuco_table_10x13_30mm.png`（**100% 缩放，不要"适合页面"**）。
   已生成：10×13 格 × 30 mm = 300×390 mm 板面，108 个内角点，254 DPI。
2. 卡尺复核方格真的是 30.00 mm。
3. **裱在硬板上**（亚克力/铝板）。纸贴在桌上会翘，翘曲直接进误差。
4. 顺带打一张腕部用的 `charuco_wrist_8x11_20mm.png`。
5. ⚠️ 你的板是 300×400；30 mm 方格在 400 mm 方向上**只能排 13 格（390 mm）**，余 10 mm。
   这是正常的，但要确认打印出来后是这个布局。

### 步骤 E：确认哪个是腕部相机
现在有三个 UVC 相机可被 OpenCV 打开（索引 0/1/3）。**必须靠"动一下机械臂看哪张图在动"来确认**，
不能靠名字。桥接服务默认用索引 3，需要你确认或改掉。

### 步骤 F：清空桌面
视野里 1646 mm 处有东西，且桌面拟合残差 p95=6.3 mm。标定和工作时视野内不要有杂物。

---

## 四、标定执行顺序（我操作，你配合摆位）

```
D → A → B → C
        ↓
1. calib/arm_zero.py            手动把臂摆到关节行程中点 → 记录 tick → 生成标定文件
        ↓                        （可选 --mode homing 写入舵机 EEPROM，兼容 lerobot 生态）
2. 验证零位                     把臂摆到 2 个姿态，FK 预测 vs 卷尺实测，200 mm 行程内差几 mm
        ↓
3. calib/intrinsics.py          拍 20–30 张板子照片 → 目标 RMS < 0.3 px
        ↓                        与 D435 出厂内参对比，偏差 >0.5% 就用自标定值
4. calib/extrinsics.py          板子固定在桌面已知位置 → 20–50 帧 → T_base_cam
        ↓                        describe_mounting() 反解出的俯角/高度必须和步骤 C 的量测吻合
5. calib/perception.py          结构化感知自检：桌面 z 是否落在预期高度
        ↓
6. 触碰探测交叉验证             手动把夹爪尖按到桌面若干点 → 对比 FK 与标定预测（应 3–8 mm）
        ↓
7. 闭环联调                     空中点位 → 桌面点位 → 抓取 → 放置
```

---

## 五、Bridge API（LLM 最终通过这些调用机械臂）

服务：`.venv\Scripts\python.exe bridge\server.py --allow-motion`（默认**只读**）

| 端点 | 作用 | 安全 |
|---|---|---|
| `GET /health` | 服务状态、是否允许运动 | — |
| `GET /state` | 关节角、工具位姿、电压、温度 | — |
| `POST /scene` | **结构化感知**：桌面、物体列表（base 坐标质心/包围盒/高度/颜色/可达性） | — |
| `POST /look` | 存图（顶视/腕部），供人或 VLM 看 | — |
| `POST /pixel_to_base` | 像素+深度 → base 坐标 | 需外参 |
| `POST /plan` | **干跑**：IK 求解但不动作 | 只读 |
| `POST /move_to` | 末端到 base 坐标目标点 | 需 `--allow-motion` + 心跳 |
| `POST /move_joints` | 关节空间目标 | 同上 |
| `POST /gripper` | 夹爪 0–100% | 同上 |
| `POST /home` | 回 URDF 中间位姿 | 同上 |
| `POST /release` `/estop` `/enable` | 断电矩 / 急停 / 恢复 | — |

看门狗：客户端静默 > 5 s 自动断力矩。

---

## 六、待你拍板

1. **DSH 插件的持久化形态**：
   - **动态 Cordis 插件**（`cordis_define`）：本会话内可用，进程重启即消失，适合先验证闭环。
   - **正式 host composition 插件**：写进组合文件，所有会话永久可用，需要改 DSH 部署配置。
2. **零位标定模式**：`soft`（只写本项目文件，完全可逆）还是 `homing`（写舵机 EEPROM，兼容 lerobot 生态与主从遥操作）。
   你选了"轻量自建 bridge"且"有主臂还没接上"，考虑到以后要遥操作采数据，**建议 `homing`**。
3. **任务线**：第一个要跑通的具体任务是什么？（例如"把红色积木放进盘子"）
