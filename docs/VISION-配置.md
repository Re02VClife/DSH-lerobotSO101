# 让我能看见：视觉能力诊断与配置

**结论先说**：我现在看不见图，**不是 DSH 配错了，也不是你的模型没有视觉**——是 DSH 的
DeepSeek 适配器**硬编码**了纯文本，而你当前走的正是这条路线。

---

## 一、诊断（已核实）

你的配置（`C:\Users\XU\.dsh\settings.yaml`）：

```yaml
agent-default-model:
  provider: deepseek-official
  model: deepseek-v4-flash
```

这个 provider 由 `@deepseek-ai/dsh-llm-deepseek` 提供。我读了它的源码：

```
lib/index.js:14   核心图像块被显式拒绝，因为这条 wire route 是纯文本的
lib/index.js:41   if (contentHasImage(blocks)) throw new LlmError(
                    "The DeepSeek chat-completions adapter does not support image content.",
                    "UNSUPPORTED_CONTENT")
lib/index.js:442  inputModalities: ["text"]
lib/index.js:514  inputModalities: ["text"]
```

所以在这条路线上，`read_image` 一定失败，报的错就是你会看到的
`model "..." does not declare image input`。**模型本身有没有视觉与这里无关**——
DSH 在读模型声明的模态时就把图挡掉了，根本不会发出去。

## 二、可修的路线：`llm-pi-ai` 适配器

DSH 里有一个**模态可配置**的适配器：`@deepseek-ai/dsh-llm-pi-ai`。
它已经在你的 profile 依赖里（`^0.1.1-rc.2`），并且**从 settings.yaml 动态读取配置，改完立即生效、不用重启**。

它支持的 profile 字段（摘自其 README）：

| 字段 | 作用 |
|---|---|
| `apiKeyEnv` | 凭据**引用**（不是明文），每次请求解析 |
| `api` / `baseURL` | 协议与端点 |
| `models` | 该路由提供的模型列表（**替换**内置目录，不是追加） |
| `modelOverrides` | 只改单个模型，其余保留 |
| **`defaultInput`** | **该路由未声明模型的默认模态** |
| **`input`** | **单个模型的模态**（每个 `models` 条目可写） |

模态解析顺序：**模型 `input` → 内置目录条目 → 路由 `defaultInput`（默认 `[text]`）**。

### ⚠️ 一个必须注意的坑（README 明确写了）

> "一个模态声明是**不被验证**的，而过度声明的后果会活过这一轮……harness 会把用户消息
> **先持久化再构建请求**，所以被拒绝的图片会留在会话日志里：那个模型会一直重发它，
> 而且模型选择会拒绝切换到任何纯文本模型。恢复办法是换一个能看图的模型、在图片之前分叉、
> 或者开新会话。"

翻译成操作建议：**不要在没验证的情况下把 `[text, image]` 加上去。**
如果网关其实不支持，你会把一张图片写进会话日志，然后这个会话就废了。

**所以先跑探测器**（我写了 `calib/vision_probe.py`，它发一个 2×2 已知颜色的 PNG，
看模型能不能说出"红、蓝"——而不是只回 200）：

```powershell
cd D:\vscode\DS101
# 方式一：直接把 key 传进去（不会写盘）
.\.venv\Scripts\python.exe calib\vision_probe.py `
  --base-url https://你的网关/v1 --model deepseek-v4-flash --api-key sk-...

# 方式二：key 放文件里，避免进 shell 历史和进程列表
.\.venv\Scripts\python.exe calib\vision_probe.py `
  --base-url https://你的网关/v1 --model deepseek-v4-flash --api-key-file D:\key.txt
```

三种结果：

| 结果 | 含义 | 该怎么办 |
|---|---|---|
| 说出 red + blue | **真支持图片** | 按下面加配置 |
| HTTP 400/422 | 端点不接受图片字段 | 保持 text-only，别加 |
| 200 但答非所问 | **接受了字段但忽略了图** | **绝对别加**——这比拒绝更糟 |

### 验证通过后，往 `settings.yaml` 加这一段

```yaml
llm-pi-ai:
  providers:
    你的路由名:
      displayName: 你的网关
      api: openai-completions
      baseURL: https://你的网关/v1
      apiKeyEnv: YOUR_GATEWAY_API_KEY
      defaultInput: [text, image]      # ← 关键
      models:
        - id: deepseek-v4-flash
          name: DeepSeek V4 Flash
          contextWindow: 1000000
          maxTokens: 384000
          input: [text, image]         # ← 也可以写在每个模型上
```

然后：

1. **新开一个会话**（当前会话的模型路线已冻结，不会中途切换）。
2. 在新会话里选这个 provider/模型。
3. 让我调 `read_image` 读一张 `calib/captures/` 里的图验证。
4. 生效后，`arm_scene` 返回的 `color_png` / `annotated_png` 我就能直接看了。

> 注意 `apiKeyEnv` 是**引用**：真正的 key 要么放在进程环境变量里，
> 要么通过 DSH 的凭据界面存。`OPENCODE_GO_API_KEY` / `GLM_API_KEY` 目前都**没有设置**，
> 所以 settings.yaml 里已有的那两个 pi-ai 路由暂时是"配置了但拿不到 key"的状态。

---

## 三、如果暂时配不通视觉，方案也不会退化

即使我完全看不见图，这套系统的设计本来就是**不依赖 VLM 做几何**的：
`arm_scene` 把画面压成结构化数字（`centroid_base_mm` / `extent_mm` /
`height_above_table_mm` / `colour_bgr`），我基于数字驾驶。

看见图会给我的是**语义能力**，不是几何能力：

| 看得见之后能做 | 现在靠什么 |
|---|---|
| "把那个像杯子的东西挪开" | 只能按颜色/尺寸猜，或让你指定 |
| 判断夹爪是不是真的夹住了（看腕部图） | 只能用失速信号 |
| 发现场景里有我没预料到的东西 | `arm_scene` 会列出未识别区域 |
| 你自己看一眼 `annotated.png` 就知道我看到的是什么 | 目前只能你手动打开图 |

最后一条其实最实用：**只要你在我给判断前瞄一眼 `calib/captures/bridge/annotated_*.png`，
你就是在替我做视觉**，而且比任何自动方案都准。这个分工现在就能用。
