# 论文翻译：Zotero 插件使用指南

----
本文档基于这台机器上已经跑通的组合整理：

- `server` 目录：`/home/japluto/zotero-pdf2zh/server`
- 翻译引擎：`pdf2zh_next`
- 翻译服务：`SiliconFlow`
- 模型：`deepseek-ai/DeepSeek-V3`
- 本地服务地址：`http://127.0.0.1:8890`

另外，这份本地 `server.py` 已经修过一处配置持久化问题：重启服务时不会再把现有 `config/config.toml` 覆盖掉。

## 1. 一次性安装

### 1.1 进入目录

```bash
cd /home/japluto/zotero-pdf2zh/server
```

### 1.2 安装 Python 依赖

```bash
python3 -m pip install --user -r requirements.txt
```

### 1.3 创建并准备 `pdf2zh_next` 的 Conda 环境

```bash
conda create -n zotero-pdf2zh-next-venv python=3.12 -y
conda run -n zotero-pdf2zh-next-venv python -m pip install --index-url https://mirrors.ustc.edu.cn/pypi/simple pdf2zh_next flask
```

说明：

- 这里显式使用 `conda`，因为这台机器已经用这个方式跑通了。
- `pdf2zh_next` 实际翻译时会走这个独立环境。

----
## 2. 配置 SiliconFlow

### 2.1 打开配置文件

```bash
cd /home/japluto/zotero-pdf2zh/server
nano config/config.toml
```

### 2.2 至少确认下面这些字段

```toml
[translation]
lang_in = "en"
lang_out = "zh-CN"

[siliconflow_detail]
siliconflow_base_url = "https://api.siliconflow.cn/v1"
siliconflow_model = "deepseek-ai/DeepSeek-V3"
siliconflow_api_key = "<你的_SiliconFlow_API_Key>"
```

说明：

- `siliconflow_api_key` 不要写进公开仓库。
- 如果你只想通过 Zotero 插件界面填 API Key，也可以；但服务端这份配置保留好更稳。

### 2.3 用命令行快速核对配置

```bash
cd /home/japluto/zotero-pdf2zh/server
rg -n 'lang_in|lang_out|siliconflow_base_url|siliconflow_model|siliconflow_api_key' config/config.toml
```

----
## 3. 启动本地翻译服务

### 3.1 前台启动

```bash
cd /home/japluto/zotero-pdf2zh/server
env -u ALL_PROXY -u all_proxy python3 -u server.py --env_tool=conda --check_update=False --port=8890
```

这一步很关键：

- 这台机器环境里有 `ALL_PROXY=socks://127.0.0.1:7890/`。
- `pdf2zh_next` 里使用的 `httpx` 在当前组合下会被这个 `socks://` 代理干扰。
- 所以启动时要显式去掉 `ALL_PROXY` 和 `all_proxy`。

### 3.2 后台启动

如果你希望服务挂在后台跑：

```bash
cd /home/japluto/zotero-pdf2zh/server
nohup env -u ALL_PROXY -u all_proxy python3 -u server.py --env_tool=conda --check_update=False --port=8890 >/tmp/zotero-pdf2zh.log 2>&1 &
```

### 3.3 查看日志

```bash
tail -f /tmp/zotero-pdf2zh.log
```

### 3.4 检查服务是否正常

```bash
curl http://127.0.0.1:8890/health
```

正常时应返回类似 JSON，包含 `status: ok`。

另外，浏览器里也可以直接打开：

- `http://127.0.0.1:8890`

这个页面可以看翻译进度、历史记录和生成文件。

----
## 4. 安装 Zotero 插件

这一部分主要是 Zotero 图形界面操作，没有纯命令行方案。

### 4.1 插件包来源

当前这个本地目录里只有 `server` 端，没有现成的 `.xpi` 安装包。插件端请用下面两种方式之一：

1. 在 Zotero 里打开 `工具 -> 插件`，检查更新。
2. 从项目发布页下载 `.xpi`，再拖进 `工具 -> 插件` 页面安装。

如果拖入后功能没有生效，重启一次 Zotero。

----
## 5. Zotero 侧如何配置

打开 Zotero 插件设置后，按下面顺序做。

### 5.1 连接本地 Python 服务

在插件设置里填写：

- `Python Server IP`：`127.0.0.1`
- 端口：`8890`
- 引擎：`pdf2zh_next`

然后点击：

- `检查连接`

如果连接失败，优先检查两件事：

1. `server.py` 是否真的在运行。
2. 端口是不是仍然是 `8890`。

### 5.2 配置翻译服务

这一步要分成两小步，少一步都不行。

第一步，在 `LLM API配置管理` 里点击 `新增`，填写：

- 服务名：优先选 `silicon`
- 如果你的插件版本显示的是 `siliconflow`，选它也可以
- URL：`https://api.siliconflow.cn/v1`
- Model：`deepseek-ai/DeepSeek-V3`
- API Key：你自己的 SiliconFlow Key
- `qps`：建议先填 `6`
- `pool size`：填 `0`

第二步，在插件设置页面顶部的 `翻译服务` 下拉框里，选中刚才配置好的服务。

说明：

- README 对 SiliconFlow 的建议并发大约是 `6` 左右，先保守设置更稳。
- 如果你不知道怎么配，优先只改 `qps`，`pool size` 保持 `0`。
- 某些版本插件里服务名写作 `silicon`，服务端内部会兼容映射到 `siliconflow`。

### 5.3 关于术语表提取

如果你使用的是 `pdf2zh_next`，并且更关心速度和稳定性，建议在插件设置里把“提取术语表”关掉。

原因：

- 术语表自动提取会增加一次额外处理。
- 在长论文上，这一步经常拖慢速度，甚至引入额外失败点。

----
## 6. 在 Zotero 里翻译论文

### 6.1 单篇翻译

在 Zotero 中对论文条目或 PDF 附件右键：

- 选择 `PDF2zh-翻译选项`
- 再选择 `翻译PDF`

插件会按照你当前设置生成默认输出文件。

### 6.2 批量翻译

如果一次选中多个条目或多个 PDF，同样可以右键后批量翻译。

### 6.3 结果会出现在哪里

翻译完成后，一般有两种查看方式：

1. 在 Zotero 条目下自动出现新附件。
2. 在 `http://127.0.0.1:8890` 的进度页面里查看和下载。

通常会生成两类 PDF：

- `*.mono.pdf`：中文版
- `*.dual.pdf`：双语对照版

----
## 7. 命令行下的常用检查

### 7.1 看服务进程

```bash
ps -ef | rg 'server.py --env_tool=conda'
```

### 7.2 看 8890 端口是否在监听

```bash
ss -ltnp | rg ':8890'
```

### 7.3 看当前 Zotero 某篇论文目录里有没有翻译结果

示例：

```bash
ls -lh ~/Zotero/storage/H5Q7UVS3/*zh-CN*.pdf
```

---
## 8. 常见问题

### 8.1 `401 invalid` 或 API 调用失败

优先检查：

1. API Key 是否正确。
2. URL 是否精确写成 `https://api.siliconflow.cn/v1`。
3. 模型名是否精确写成 `deepseek-ai/DeepSeek-V3`。

不要把 URL 写成带 `/chat/completions` 等后缀的形式。

### 8.2 点了“检查连接”但连不上

优先执行：

```bash
ps -ef | rg 'server.py --env_tool=conda'
curl http://127.0.0.1:8890/health
```

如果没有返回正常结果，就先重启服务。

### 8.3 服务启动了，但翻译时报代理错误

请确认你是这样启动的：

```bash
env -u ALL_PROXY -u all_proxy python3 -u server.py --env_tool=conda --check_update=False --port=8890
```

不要直接裸跑 `python3 server.py`，否则很容易被当前机器上的 `socks://127.0.0.1:7890/` 代理环境变量干扰。

### 8.4 配置会不会在重启后丢失

这台机器上的本地 `server.py` 已经修过，现有 `config/config.toml` 不会在重启时被示例文件覆盖。

----
## 9. 我推荐的最短使用路径

如果你已经装好了依赖，平时只需要做这几步：

1. 启动服务

```bash
cd /home/japluto/zotero-pdf2zh/server
env -u ALL_PROXY -u all_proxy python3 -u server.py --env_tool=conda --check_update=False --port=8890
```

2. 打开 Zotero，确认：

- `Python Server IP = 127.0.0.1`
- `端口 = 8890`
- `引擎 = pdf2zh_next`
- `翻译服务 = silicon` 或 `siliconflow`
- `模型 = deepseek-ai/DeepSeek-V3`

3. 右键论文条目或 PDF：

- `PDF2zh-翻译选项 -> 翻译PDF`

4. 去 `http://127.0.0.1:8890` 看进度，翻译完成后在 Zotero 里看新附件。

---
## 10. 如何停止后台翻译服务

如果你之前是用后台方式启动的服务，最稳妥的关闭方法是：先查 PID，再结束进程。

### 10.1 查出服务 PID

```bash
ps -ef | rg 'python3 -u server.py --env_tool=conda --check_update=False --port=8890'
```

会看到类似：

```text
japluto   947133  ... python3 -u server.py --env_tool=conda --check_update=False --port=8890
```

这里的 `947133` 就是 PID。

### 10.2 停掉这个服务

```bash
kill 947133
```

把 `947133` 换成你实际查到的 PID。

### 10.3 验证是否已经停掉

```bash
ps -ef | rg 'python3 -u server.py --env_tool=conda --check_update=False --port=8890'
curl -s http://127.0.0.1:8890/health
```

判断方式：

- 如果第一条命令查不到服务进程，说明进程已经结束。
- 如果第二条命令连不上，说明 `8890` 上的服务已经停止。

### 10.4 一条命令直接停止

如果你确认只有这一条服务在跑，也可以直接用：

```bash
pkill -f 'python3 -u server.py --env_tool=conda --check_update=False --port=8890'
```

这个写法更快，但不如“先查 PID 再 kill”直观。日常更推荐上一种。
