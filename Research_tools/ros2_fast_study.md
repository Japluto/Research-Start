## 目前的目标是一周内拿下ros2的基本使用
---
#### 一、一些前置步骤：
---
**1. 激活环境（设置的别名）：ros2_on**
* 一些基本的检查 —— 
* （1）printenv | grep ROS
* （2）ros2 doctor
**2. turtlesim 验证ros2是否正常工作**

---
#### 二、linux命令行操作
---
**1. 文件与目录操作 (基础核心)**

* **`pwd`** (Print Working Directory)
* **作用：** 显示你当前所在的完整路径。
</br>

* **`ls`** (List)
* **作用：** 列出当前目录下的文件。
* **常用参数：**
* `ls`：仅列出非隐藏文件。
* **`ls -A`** (或 `ls -a`)：列出所有文件，包括以 `.` 开头的隐藏文件。
</br>

* **`cd`** (Change Directory)
* **作用：** 切换目录。
* **常用技巧：**
* `cd test/`：进入 test 文件夹。
* **`cd ..`**：返回上一级目录（非常有用的操作）。
* `cd ~` 或直接 `cd`：快速回到用户的家目录 (`/home/用户名`)。
</br>

* **`mkdir`** (Make Directory)
* **作用：** 创建一个新的文件夹。
* *例子：* `mkdir test` 创建了一个名为 test 的目录。
</br>

* **`touch`**
* **作用：** 创建一个空的普通文件（如果文件已存在，则更新其时间戳）。
* *例子：* `touch read.txt`。
</br>


**2. 删除操作 (重点注意)**
* **`rm`** (Remove)
* **作用：** 删除文件或目录。
* **关键区别（你在截图中遇到的坑）：**
* `rm read.txt`：直接删除普通文件。
</br>

* **`rm -R`** (或 `-r`, Recursive)：**递归删除**。如果你想删除一个**文件夹**（如 `test/`），必须加这个参数，否则会报错 "Is a directory"（如截图 1 第一行所示）。

**3. 软件管理与权限 (进阶)**
* **`sudo`** (SuperUser Do)
* **作用：** 以管理员（root）权限执行命令。安装软件或修改系统配置时必须使用。
</br>

* **`apt install`**
* **作用：** Ubuntu 的软件包管理器，用于安装软件。
</br>

* **Tab 键补全 (高效技巧)**



---

#### 三、ros2命令行操作

---
**1. 节点 (Node) 相关操作**

* **查看节点列表：** `ros2 node list`
* **查看节点具体信息：** `ros2 node info <node_name>`
* *注：* 可以看到节点发布/订阅的话题、服务和动作。


* **运行一个节点：** `ros2 run <package_name> <executable_name>`
* *例子：* `ros2 run turtlesim turtlesim_node`



**2. 话题 (Topic) 相关操作**

* **查看话题列表：** `ros2 topic list`
* 加上 `-t` 参数可显示消息类型：`ros2 topic list -t`


* **监听话题内容 (Echo)：** `ros2 topic echo <topic_name>`
* *作用：* 实时打印该话题的数据，用于调试。


* **查看话题详情：** `ros2 topic info <topic_name>`
* *作用：* 查看发布者和订阅者的数量。


* **手动发布消息：** `ros2 topic pub <topic_name> <msg_type> "<args>"`
* *例子（让海龟动起来）：*
`ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`



**3. 服务 (Service) 相关操作**

* **查看服务列表：** `ros2 service list`
* **查看服务类型：** `ros2 service type <service_name>`
* **手动调用服务：** `ros2 service call <service_name> <service_type> "<args>"`
* *例子（生成新海龟）：* `ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"`



**4. 常用可视化与调试工具**

* **rqt_graph：**
* 在终端输入 `rqt_graph`，会弹出一个窗口，直观展示当前所有节点和话题的连接关系图（非常重要）。


* **ros2 bag (录包工具)：**
* `ros2 bag record -a`：记录所有话题数据。
* `ros2 bag play <bag_file>`：回放数据。

