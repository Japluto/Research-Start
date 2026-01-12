## 配置
---
> 这里主要是写一下怎么启动，担心后面忘记了
### 几个常用的命令
* load_isaac_ros —— 启动isaacsim适配的3.11py的ros2
* isaacsim
---
根据你提供的 `.bashrc` 配置，启动 Isaac Sim 并配合你编译好的**专用 ROS 2 环境 (Python 3.11/3.12)** 需要分为两个步骤，通常建议使用**两个独立的终端窗口**来进行操作。

这是标准的操作流程：

### 第一步：启动 Isaac Sim 仿真器 (终端 1)

这个终端只负责运行仿真软件本身的 GUI 界面。

1. 打开一个新的终端。
2. 输入你配置好的别名命令：
```bash
isaacsim

```


> **原理解析**：根据你的配置，这条命令会临时添加 Isaac Sim 所需的 `LD_LIBRARY_PATH` 库路径并启动软件。它不会受到你后续加载的自定义 ROS 环境影响，保证仿真器本身能稳定运行。



---

### 第二步：加载适配的 ROS 2 环境 (终端 2)

这个终端用于运行你的 Python 脚本、ROS 2 节点，或者进行 `ros2 topic list` 等操作。**这是你使用 Python 3.11/3.12 的地方。**

1. 打开一个新的终端（默认会加载系统的 ROS Humble）。
2. 执行你编写的环境切换函数：
```bash
load_isaac_ros

```


3. **验证环境是否成功加载**：
执行完上述命令后，你应该会看到 `🔄 正在切换至 Isaac Sim 专用 ROS 2 环境...` 的提示。
此时，你可以输入以下命令确认 Python 版本和 ROS 路径是否正确：
```bash
python3 --version
# 输出应该是 Python 3.11.x 或 3.12.x (取决于你编译时的环境)

which python3
# 输出应该指向你的 Conda 环境或自定义环境，而不是 /usr/bin/python3

printenv ROS_DISTRO
# 确认依然是 humble

```



---

### 第三步：如何进行测试？

现在你已经准备就绪：

* **终端 1** 跑着 Isaac Sim。
* **终端 2** 跑着适配版 ROS 2。

**测试流程：**

1. 在 **终端 1 (Isaac Sim)** 中：
* 点击菜单栏 `Window` -> `Extensions`。
* 搜索 `ROS2 Bridge` 并确保它已启用 (Enabled)。
* 点击 `Play` 按钮开始仿真（或者加载一个带有 ROS2 Bridge 的示例场景，例如 `Isaac Examples -> ROS2 -> Navigation`）。


2. 在 **终端 2 (Custom ROS Env)** 中：
* 尝试列出话题，看能否通过新环境看到仿真器发出的消息：
```bash
ros2 topic list

```


* 如果你能看到类似 `/tf`, `/scan`, `/rgb` 等话题，说明 **新的 Python 环境与 Isaac Sim 通信成功**。



### 总结

* 启动仿真器用：`isaacsim`
* 运行代码/ROS命令用：`load_isaac_ros`

不要在同一个终端里混用这两个命令，否则环境变量冲突可能会导致报错 (例如 `GLIBCXX not found` 或 `rmw_implementation` 错误)。