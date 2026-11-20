# ~/.bashrc: executed by bash(1) for non-login shells.

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# 'task box' runs from 'devbox shell' and Distrobox passes that environment through:
# Nix compilers (CC, CXX, ...), Nix build flags and Python 3.13 paths. Drop them so the
# box uses its own Ubuntu toolchain and Python, which ROS 2 is built for.
if [ -n "${DEVBOX_PROJECT_ROOT:-}${IN_NIX_SHELL:-}" ]; then
    unset AR AS CC CXX LD NM OBJCOPY OBJDUMP RANLIB READELF SIZE STRINGS STRIP \
        CONFIG_SHELL DETERMINISTIC_BUILD HOST_PATH IN_NIX_SHELL \
        PYTHONPATH PYTHONHASHSEED PYTHONNOUSERSITE _PYTHON_HOST_PLATFORM _PYTHON_SYSCONFIGDATA_NAME
    for var in $(compgen -e | grep -E '^(NIX_|DEVBOX_|__DEVBOX)'); do
        unset "$var"
    done
    PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v -e '^/nix/' -e '/\.devbox/' | paste -sd: -)"
    XDG_DATA_DIRS="$(printf '%s' "${XDG_DATA_DIRS:-}" | tr ':' '\n' | grep -v -e '^/nix/' -e '/\.devbox/' | paste -sd: -)"
    export PATH XDG_DATA_DIRS
fi

# Append to the history file, don't overwrite it
shopt -s histappend

# Check the window size after each command and, if necessary, Update the values of LINES and COLUMNS.
shopt -s checkwinsize

export PROMPT_COMMAND="history -a;$PROMPT_COMMAND"
export HISTCONTROL=erasedups
export HISTSIZE=10000000
export HISTFILESIZE=10000000

export WORKDIR="${WORKDIR:-/home/workdir}"
export BANNER="$HOME/.motd"
export PS1='🚀 distrobox:\[\033[01;34m\]\W\[\033[00m\]> '

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

if [ -f $BANNER ]; then
    cat $BANNER
fi

cd $WORKDIR

# ROS 2 Jazzy and the UGV workspace (if built)
source /opt/ros/jazzy/setup.bash
if [ -f "$WORKDIR/install/setup.bash" ]; then
    source "$WORKDIR/install/setup.bash"
fi
