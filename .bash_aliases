
alias rosmake='ROS_PARALLEL_JOBS=-j6 nice rosmake'

function guri()
{
  if [[ $# < 1 ]]
  then
    echo $GAZEBO_MASTER_URI
  else
    if [[ $# < 2 ]]
    then
      PORT=11345
    else
      PORT=$2
    fi
    export GAZEBO_MASTER_URI=http://$1:$PORT
  fi
}

function rosip()
{
  if [[ $# < 1 ]]
  then
    INTERFACE=wlan0
  else
    INTERFACE=$1
  fi

  export ROS_IP=$(ifconfig $INTERFACE | grep "inet addr" | cut -d ':' -f 2 | cut -d ' ' -f 1)
}

function uri()
{
  if [[ $# < 1 ]]
  then
    echo $ROS_MASTER_URI
  else
    if [[ $1 == test ]]
    then
      export ROS_MASTER_URI=http://localhost:22422
    else
      if [[ $# < 2 ]]
      then
        PORT=11311
      else
        PORT=$2
      fi
      export ROS_MASTER_URI=http://$1:$PORT
    fi
  fi
}

function log()
{
  if [[ $# < 1 ]]
  then
    rosservice list | grep get_loggers | cut -d/ -f 2
  elif [[ $# < 2 ]]
  then
    rosservice call $1/get_loggers
  elif [[ $# < 3 ]]
  then
    rosservice call $1/set_logger_level "{logger: ros, level: $2}"
  else
    rosservice call $1/set_logger_level "{logger: $2, level: $3}"
  fi
}

function uln()
{
  TARGET=`ls -l $1 | awk '{print $11}'`
  unlink $1
  cp $TARGET $1
}

rviz()
{
  if [[ $# < 1 ]]
  then
    rosrun rviz rviz
  elif [[ $# < 2 ]]
  then
    rosrun rviz rviz -d ${1}
  else
    rosrun rviz rviz -d $(rospack find ${1})/config/${2}.rviz
  fi
}

function release()
{
  if [[ $# < 1 ]]
  then
    ROS_DISTRO=indigo
  else
    ROS_DISTRO=$1
  fi

  catkin_generate_changelog
  git ci -am "Update changelog"
  catkin_prepare_release -y
  bloom-release -y $(basename $(pwd)) --track $ROS_DISTRO --rosdistro $ROS_DISTRO
}

function term()
{
  if [[ $# > 0 ]]
  then
    gnome-terminal -x bash -ci "$*; bash"
  else
    gnome-terminal -x "bash"
  fi
}

function termn()
{
  if [[ $# > 0 ]]; then
    N=$1
    shift
  else
    N=1
  fi

  for i in $(seq 1 $N); do
    term $*
  done
}

function cake()
{
  ARGS="-j8 -DCMAKE_BUILD_TYPE=RelWithDebInfo"

  if [[ $# > 0 ]]; then
    ARGS="$ARGS --only-pkg-with-deps $*"
  fi

  catkin_make $ARGS
}

function cm()
{
  WS_DIR=($(echo $CMAKE_PREFIX_PATH | tr ':' '\n'))
  WS_DIR=${WS_DIR[0]}
  WS_DIR=${WS_DIR/\/devel/}

  if [[ $# < 1 ]]
  then
    PKG=$(basename $(pwd))
  else
    PKG=$1
  fi

  cd $WS_DIR
  catkin_make -j8 --only-pkg-with-deps $PKG
  cd -
}

#alias psgrep='pgrep '
alias psgrep='ps aux | grep -i '
alias lgrep='ls | grep -i '
alias fgrep='find | grep -i '
alias dgrep='dpkg -l | grep -i '
alias egrep='env | grep -i '
alias rosdepgrep='rosdep db | grep -i '
alias envgrep='env | grep -i '
alias hgrep='history | grep -i '

alias roscorebg='roscore >/dev/null 2>&1 &'

# misspelling aliases:
alias rosonde='rosnode '
alias rosndoe='rosnode '

alias joystick='rosrun joy joy_node _dev:=/dev/input/js1 '

function roskill()
{
  killall -q gazebo gzclient

  #pkill -9 ros
  #pgrep ros | ...
  psgrep ros | grep -v 'vim\|sublime\|g++\|gcc\|c++\|rosmake\|catkin\|grep\|chrome' | awk '{print $2}' | xargs -I{} kill {}
  psgrep ros | grep -v 'vim\|sublime\|g++\|gcc\|c++\|rosmake\|catkin\|grep\|chrome' | awk '{print $2}' | xargs -I{} kill -9 {}

  psgrep rqt | grep -v 'vim\|sublime\|g++\|gcc\|c++\|rosmake\|catkin\|grep\|chrome' | awk '{print $2}' | xargs -I{} kill {}
  psgrep rqt | grep -v 'vim\|sublime\|g++\|gcc\|c++\|rosmake\|catkin\|grep\|chrome' | awk '{print $2}' | xargs -I{} kill -9 {}

  roscorebg
}

function rosbagfilter()
{
  INPUT=$1
  FRAME=$2

  OUTPUT=${INPUT//.bag/_filtered.bag}

  rosbag filter $INPUT $OUTPUT "topic != '/tf' or m.transforms[0].header.frame_id != '$FRAME'"
}

alias deps='rosdep install --from-paths src -iy '

function f()
{
  findcmd='find | grep -v "\.svn" | grep -v "\.git"'
  if [ $# -gt 0 ]; then
    result=`eval $findcmd | grep --color=always $* | tee /dev/stderr`
    if [ -z "$result" ]; then
      eval $findcmd | grep --color=always -i $*
    fi
  else
    eval $findcmd
  fi
}

alias a='ack-grep -i '

# Pipe to vim
function v()
{
  $@ | vim -R -
}

alias vi='vim '

alias ws='source devel/setup.bash'

# ls aliases
alias ll='ls -rthal'
alias la='ls -A'
alias l='ls -CF'

# other aliases
alias o='xdg-open'
alias k='pkill -9'
alias c='clear'
alias x='exit'
alias q='exit'
alias ..='cd ..'
alias ...='cd ../..'
alias ....='cd ../../..'
alias .....='cd ../../../..'
alias ......='cd ../../../../..'
alias .......='cd ../../../../../..'
alias -- -='cd -'

alias rosnodegrep='rosnode list | grep -i '
alias rostopicgrep='rostopic list | grep -i '
alias rosservicegrep='rosservice list | grep -i '
alias rosparamgrep='rosparam list | grep -i '

function rosnodeinfogrep()
{
  if [[ $# < 1 ]]
  then
    PATTERN="unknown"
  else
    PATTERN=$1
  fi

  for node in $(rosnode list)
  do
    rosnode info | grep -i $PATTERN
  done
}

alias sudo='sudo '

alias gg='git log --oneline --abbrev-commit --all --graph --decorate --color'

alias ud='udisks --detach'

alias log-out='gnome-session-quit'
alias x-start='service lightdm restart'

alias off='xset dpms force off'

alias cbuild='catkin build --cmake-args -DCMAKE_C_FLAGS="-Wall -W -Wno-unused-parameter -Werror" -DCMAKE_CXX_FLAGS="-Wall -W -Wno-unused-parameter -Werror" -DCMAKE_BUILD_TYPE=Release -- --no-notify '
alias cbuildnowarn='catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release -- --no-notify '
alias cdebug='catkin build --cmake-args -DCMAKE_C_FLAGS="-Wall -W -Wno-unused-parameter -Werror" -DCMAKE_CXX_FLAGS="-Wall -W -Wno-unused-parameter -Werror" -DCMAKE_BUILD_TYPE=RelWithDebInfo -- --no-notify '
#alias cbuild='catkin build --cmake-args -DCMAKE_C_FLAGS="-Wall -W -Wno-unused-parameter -Werror" -DCMAKE_CXX_FLAGS="-Wall -W -Wno-unused-parameter -Werror" -DCMAKE_BUILD_TYPE=Debug -- --no-notify '
alias cclean='catkin clean --deinit -y '
alias cinstall='catkin build --no-notify --catkin-make-args install -- '
alias ctest='catkin run_tests --no-deps --this --no-notify '
alias cconfig='catkin config --install --isolate-install --isolate-devel '

# Current year calendar
alias cal='cal -y'

# matlab in awesome
# see http://awesome.naquadah.org/wiki/Problems_with_Java
function matlab()
{
  wmname LG3D
  if [[ $# < 1 ]]
  then
    MATLAB_VERSION=R2013a
  else
    MATLAB_VERSION=$1
  fi
  /usr/local/MATLAB/$MATLAB_VERSION/bin/matlab -desktop
}

# set operations between files
function set_union()
{
  cat "$1" "$2" | sort | uniq
}

function set_difference()
{
  cat "$1" "$2" "$2" | sort | uniq -u
}

# repeat
function repeat()
{
  n=$1
  shift
  while [ $(( n -= 1 )) -ge 0 ]
  do
      "$@"
  done
}

# A shortcut function that simplifies usage of xclip.
# - Accepts input from either stdin (pipe), or params.
# Taken from: http://madebynathan.com/2011/10/04/a-nicer-way-to-use-xclip/
# ------------------------------------------------
cb() {
  local _scs_col="\e[0;32m"; local _wrn_col='\e[1;31m'; local _trn_col='\e[0;33m'
  # Check that xclip is installed.
  if ! type xclip > /dev/null 2>&1; then
    echo -e "$_wrn_col""You must have the 'xclip' program installed.\e[0m"
  # Check user is not root (root doesn't have access to user xorg server)
  elif [[ "$USER" == "root" ]]; then
    echo -e "$_wrn_col""Must be regular user (not root) to copy a file to the clipboard.\e[0m"
  else
    # If no tty, data should be available on stdin
    if ! [[ "$( tty )" == /dev/* ]]; then
      input="$(< /dev/stdin)"
    # Else, fetch input from params
    else
      input="$*"
    fi
    if [ -z "$input" ]; then  # If no input, print usage message.
      echo "Copies a string to the clipboard."
      echo "Usage: cb <string>"
      echo "       echo <string> | cb"
    else
      # Copy input to clipboard
      echo -n "$input" | xclip -selection c
      # Truncate text for status
      if [ ${#input} -gt 80 ]; then input="$(echo $input | cut -c1-80)$_trn_col...\e[0m"; fi
      # Print status.
      echo -e "$_scs_col""Copied to clipboard:\e[0m $input"
    fi
  fi
}

# Aliases / functions leveraging the cb() function
# ------------------------------------------------
# Copy contents of a file
function cbf() { cat "$1" | cb; }

# Copy SSH public key
alias cbssh="cbf ~/.ssh/id_rsa.pub"

# Copy current working directory
alias cbwd="pwd | cb"

# Copy most recent command in bash history
alias cbhs="cat $HISTFILE | tail -n 1 | cb"
# Paste
alias cbp="xclip -o -selection clipboard"

# VTune
alias vtune='amplxe-cl -collect hotspots -run-pass-thru=--no-altstack -result-dir vtune_result_dir_$(date +%Y-%m-%d-%H-%M-%S) -- '
alias vtune-gui='amplxe-gui '

# Battery status
alias battery='upower -i /org/freedesktop/UPower/devices/battery_BAT0 | grep -E "state|to\ full|percentage"'

# Anti Virus:
function virus()
{
  if [[ $# -lt 1 ]]
  then
    DIR=/
  else
    DIR=$1
  fi

  clamscan -r --bell -i $DIR
}
