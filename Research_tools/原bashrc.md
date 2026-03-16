bashrc_settings_summary() {
    echo "=== 你的 Bash 关键设置 ==="
    echo "1) 历史记录: HISTCONTROL=ignoreboth, HISTSIZE=1000, HISTFILESIZE=2000, histappend=on"
    echo "2) 终端体验: checkwinsize=on, 彩色提示符与 xterm 标题已启用"
    echo "3) 常用别名: ll/la/l, grep/fgrep/egrep 带颜色, alert"
    echo "4) Conda: 已初始化 (/home/japluto/anaconda3)"
    echo "5) ROS2 默认环境: /opt/ros/humble + ~/ROS2_study/dev_ws (若存在)"
    echo "6) Isaac Sim 变量: ISAACSIM_PATH=$HOME/Research/isaac-sim, ISAACSIM_PYTHON=$ISAACSIM_PATH/python.sh"
    echo "7) Pegasus 变量: PEGASUS_SIMULATOR_PATH=$HOME/isaac-projects/PegasusSimulator/PegasusSimulator"
    echo "8) 你自定义的快捷命令: isaacsim, load_isaac_env, load_isaac_ws, ros2_reset, ros2_on, run_pegasus, qgc"
    echo "9) 代理例外: no_proxy / NO_PROXY = localhost,127.0.0.1,::1"
}
alias my_bash_settings='bashrc_settings_summary'
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
    # We have color support; assume it's compliant with Ecma-48
    # (ISO/IEC-6429). (Lack of such support is extremely rare, and such
    # a case would tend to support setf rather than setaf.)
    color_prompt=yes
    else
    color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi


# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/home/japluto/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/japluto/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/home/japluto/anaconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/japluto/anaconda3/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<

# =======================================================
#               ROS 2 & Isaac Sim 环境配置 (修复版)
# =======================================================

# --- 1. 默认加载纯净的 ROS 2 环境 ---
# 这样你打开终端就能直接跑 ros2 run，不会报错
source /opt/ros/humble/setup.bash
if [ -f ~/ROS2_study/dev_ws/install/local_setup.sh ]; then
    source ~/ROS2_study/dev_ws/install/local_setup.sh
fi

# --- 2. Isaac Sim 路径变量 ---
export ISAACSIM_PATH="${HOME}/Research/isaac-sim"
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"

# ✅ 【新增】Pegasus 路径变量 (从官方指南里“偷”来的)
# 这样以后有些脚本可以直接用 $PEGASUS_PATH 找到插件位置
export PEGASUS_SIMULATOR_PATH="${HOME}/isaac-projects/PegasusSimulator/PegasusSimulator"


# --- 3. 启动 Isaac Sim 的别名 (环境隔离) ---
# 当你输入 'isaacsim' 时，命令会临时加上 Isaac Sim 的库路径去运行
# 运行结束后，当前终端依然保持纯净，不影响普通 ROS2 使用
alias isaacsim='LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib" RMW_IMPLEMENTATION=rmw_fastrtps_cpp ${ISAACSIM_PATH}/isaac-sim.sh'


# 只有当你需要在这个终端里编译 Isaac Sim 相关的 ROS 包时，才输入这个命令
alias load_isaac_env='export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:${ISAACSIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib" && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && echo ">> Isaac Sim 环境已加载 (注意：此终端可能无法运行普通ROS2节点)"'

# --- 5. 其他工作空间快捷指令 ---
alias load_isaac_ws='source ~/IsaacSim-ros_workspaces/build_ws/humble/humble_ws/install/local_setup.bash'

# --- 6. 应急工具 ---
# 如果环境乱了，输入 ros2_reset 重置
alias ros2_reset='export LD_LIBRARY_PATH="" && source /opt/ros/humble/setup.bash && echo ">> ROS2 环境已重置 (LD_LIBRARY_PATH 已清空)"'

alias ros2_on='export LD_LIBRARY_PATH="" && conda deactivate 2>/dev/null; source /opt/ros/humble/setup.bash && if [ -f ~/ROS2_study/dev_ws/install/local_setup.sh ]; then source ~/ROS2_study/dev_ws/install/local_setup.sh; fi && echo "✅ ROS2 环境已强制重置 (Conda已退出 + 工作空间已加载)"'

# 定义一个函数来切换环境
function load_isaac_ros() {
    echo "🔄 正在切换至 Isaac Sim 专用 ROS 2 环境 (Python 3.11)..."

    # 1. 【核心步骤】清理原本环境 (关掉原有的ROS2)
    # 因为你的 .bashrc 开头自动加载了系统 ROS，所以这里必须先 unset 掉
    unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROS_LOCALHOST_ONLY ROS_VERSION
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
    unset PYTHONPATH
    
    # 清空 LD_LIBRARY_PATH (防止与系统库冲突)
    export LD_LIBRARY_PATH=""

    # 2. 加载我们刚编译好的新环境
    # 使用绝对路径，确保不出错
    local WS_ROOT=~/IsaacSim-ros_workspaces/build_ws/humble
    
    if [ -f "$WS_ROOT/humble_ws/install/setup.bash" ]; then
        source "$WS_ROOT/humble_ws/install/setup.bash"
        source "$WS_ROOT/isaac_sim_ros_ws/install/setup.bash"
        
        echo "🚀 Isaac Sim Custom ROS 2 (Python 3.11) 已加载!"
        echo "   |-- 核心库路径: $WS_ROOT"
        echo "   |-- 注意: 此环境专用于 Isaac Sim 通信，'$ ISAACSIM_PYTHON' 运行脚本"
    else
        echo "❌ 错误: 找不到构建文件，请检查 ~/IsaacSim-ros_workspaces/build_ws/humble 是否存在。"
    fi
}

# Pegasus 专用启动命令 (自动加载环境 + 启动仿真 + 挂载插件)
alias run_pegasus='load_isaac_ros && isaacsim --ext-folder ~/isaac-projects/PegasusSimulator/PegasusSimulator/extensions'
export no_proxy="localhost,127.0.0.1,::1"
export NO_PROXY="localhost,127.0.0.1,::1"

alias qgc="$HOME/QGC/QGC.AppImage"

export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion
