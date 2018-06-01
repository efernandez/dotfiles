# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
[ -z "$PS1" ] && return

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth
# ... or force ignoredups and ignorespace
#HISTCONTROL=ignoredups:ignorespace

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=10000
HISTFILESIZE=20000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "$debian_chroot" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color) color_prompt=yes;;
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

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.
for file in $(ls ~/.bash_aliases*); do
    if [ -f $file ]; then
        source $file
    fi
done

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
    . /etc/bash_completion
fi

#SSH_AUTH_SOCK=`ss -xl | grep -o '/run/user/enrique/keyring-.*/ssh'`
#[ -z "$SSH_AUTH_SOCK" ] || export SSH_AUTH_SOCK

# Support for ssh-agent with screen sessions:
# https://gist.github.com/martijnvermaat/8070533
# http://superuser.com/questions/180148/how-do-you-get-screen-to-automatically-connect-to-the-current-ssh-agent-when-re/424588#424588
if [[ -S "$SSH_AUTH_SOCK" && ! -h "$SSH_AUTH_SOCK" ]]; then
    ln -sf "$SSH_AUTH_SOCK" ~/.ssh/ssh_auth_sock;
fi
export SSH_AUTH_SOCK=~/.ssh/ssh_auth_sock;

# Search history with arrows (like in Matlab):
bind '"\e[A": history-search-backward'
bind '"\e[B": history-search-forward'

# Prompt:
if [[ "$PS1" != "" ]]; then
    function _ps1_master_uri() {
        host=$(echo $ROS_MASTER_URI | cut -d'/' -f3 | cut -d':' -f1)
        if [[ "$host" != "localhost" ]]; then
            echo "{$host}"
        fi
    }
    function _ps1_git_branch() {
        branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
        if [[ "$branch" != "" ]]; then
            echo "<$branch>"
        fi
    }
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[1;35m\][${ROS_DISTRO}]\[\033[00m\]\[\033[1;30m\]`_ps1_master_uri`\[\033[00m\]\[\033[1;31m\]`_ps1_git_branch`\[\033[00m\]\n\[\033[01;34m\]\w\[\033[00m\]\$ '
    case "$TERM" in
    xterm*|rxvt*)
        PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
        ;;
    *)
        ;;
    esac
fi

shopt -s autocd

source /opt/ros/indigo/setup.bash
export ROBOT_SIMULATION=true
export IS_GAZEBO=true
export ROS_IP=127.0.0.1

#eval `ssh-agent -s`
added_keys=`ssh-add -l`
if [ ! $(echo $added_keys | grep -o -e bitbucket) ]; then
   ssh-add "$HOME/.ssh/id_rsa_bitbucket"
fi
if [ ! $(echo $added_keys | grep -o -e gitlab) ]; then
   ssh-add "$HOME/.ssh/id_rsa_gitlab"
fi
if [ ! $(echo $added_keys | grep -o -e github) ]; then
   ssh-add "$HOME/.ssh/id_rsa_github"
fi
if [ ! $(echo $added_keys | grep -o -e robot) ]; then
   ssh-add "$HOME/.ssh/id_rsa_robot"
fi

ulimit -c unlimited

export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:$HOME/stacks"
export PATH=/usr/lib/ccache:${PATH}
