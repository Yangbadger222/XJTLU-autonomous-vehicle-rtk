# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc) for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# ==========================================
# 1. BASH HISTORY & DISPLAY DEFAULTS
# ==========================================
HISTCONTROL=ignoreboth
shopt -s histappend
HISTSIZE=1000
HISTFILESIZE=2000
shopt -s checkwinsize

[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
    color_prompt=yes
else
    color_prompt=
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# ==========================================
# 2. ALIASES & ENVIRONMENT VARIABLES
# ==========================================
# Standard Aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Additional Handy Aliases for Devs
alias rc='vi ~/.bashrc'
alias s1='source ~/.bashrc'
alias gs='git status'
alias cw='cd ~/XJTLU-autonomous-vehicle/'
alias ss="source /opt/ros/humble/setup.bash; if [ -f install/setup.bash ]; then source install/setup.bash; echo 'ROS 2 Humble and Workspace sourced!'; else echo 'ROS 2 Humble sourced. No install/setup.bash found.'; fi"

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

# Custom Make wrapper for ROS 2
mbuild() {
    make "$@"
    if [ $? -eq 0 ] && [ -f install/setup.bash ]; then
        source install/setup.bash
        echo -e "\n\033[0;32m>>> Build successful! You may run 'make launch-...'\033[0m"
    fi
}

# Environment Variables
export PATH="$HOME/.local/bin:$PATH"
export HF_ENDPOINT=https://hf-mirror.com

# 🟢 Permanent NTRIP Export (No need to export manually anymore)
export FYP_RTK_PARAMS_FILE=/tmp/um982_cors.yaml


# ==========================================
# 3. ROS 2 & WORKSPACE SETUP
# ==========================================
# Source core ROS 2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Navigate to the workspace securely
cd ~/XJTLU-autonomous-vehicle/ || echo "Warning: Workspace ~/XJTLU-autonomous-vehicle/ not found!"

# Source local workspace if built
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
fi

# Source XJTLU Tailscale ROS 2 environment
if [ -f "$HOME/.ros2_tailscale_env" ]; then
    source "$HOME/.ros2_tailscale_env"
fi


# ==========================================
# 4. FROG CAR SSH DASHBOARD
# ==========================================
# Print Header (with color codes for flair)
echo -e "\e[1;32m================================================\e[0m"
echo -e "\e[1;32m                🐸 FROG CAR 🐸                \e[0m"
echo -e "\e[1;32m                🐸 牛蛙小车 🐸                \e[0m"
echo -e "\e[1;32m================================================\e[0m"

# Show System Info
sys_load=$(uptime | awk -F'load average:' '{ print $2 }' | xargs)
sys_ip=$(hostname -I | awk '{print $1}')
echo -e "\e[1;36m💻 System:\e[0m IP: $sys_ip | Load: $sys_load"

# Show current Git Branch
current_branch=$(git branch --show-current 2>/dev/null)
if [ -n "$current_branch" ]; then
    echo -e "\e[1;34m🌿 Branch:\e[0m $current_branch"
else
    echo -e "\e[1;34m🌿 Branch:\e[0m Not a git repository"
fi

# Check for active robot modes
echo -e "\n\e[1;33m⚙️  Active Robot Modes:\e[0m"
found_mode=false

for mode in slam explore indoor-nav corridor travel explore-gps nav-gps tightly-coupled rtk-basic; do
    mode_underscore=$(echo "$mode" | tr '-' '_')
    if pgrep -f "make launch-$mode" >/dev/null || pgrep -f "$mode_underscore" >/dev/null; then
        echo "   🟢 Running: $mode"
        found_mode=true
    fi
done

if [ "$found_mode" = false ]; then
    echo "   🔴 None (Robot is idle)"
fi

# Show NTRIP Status
echo -e "\n\e[1;35m📡 NTRIP Status:\e[0m"
if [ -f "scripts/setup_ntrip.py" ]; then
    # timeout 1s enforces a strict 1-second limit to prevent SSH hangs
    ntrip_output=$(timeout 1s python3 scripts/setup_ntrip.py --status 2>&1)
    exit_code=$?
    
    # Exit code 124 means the 'timeout' command forcibly killed the script
    if [ $exit_code -eq 124 ]; then
        echo "   ⚠️  Status check timed out (Server took > 1s)."
    else
        echo "$ntrip_output" | sed 's/^/   /'
    fi
else
    echo "   ⚠️  scripts/setup_ntrip.py not found in current directory."
fi

echo -e "\e[1;32m================================================\e[0m\n"