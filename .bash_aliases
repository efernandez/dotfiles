
alias rosmake='ROS_PARALLEL_JOBS=-j6 nice rosmake'

guri(){
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

rosip()
{
  if [[ $# < 1 ]]
  then
    INTERFACE=wlan0
  else
    INTERFACE=$1
  fi

  export ROS_IP=$(ifconfig $INTERFACE | grep "inet addr" | cut -d ':' -f 2 | cut -d ' ' -f 1)
}

uri(){
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

uln(){
  TARGET=`ls -l $1 | awk '{print $11}'`
  unlink $1
  cp $TARGET $1
}

rviz(){
  if [[ $# < 1 ]]
  then
    rosrun rviz rviz
  else
    #rosrun rviz rviz -d `rospack find ${1}_2dnav`/config/rviz/navigation.rviz
    rosrun rviz rviz -d `rospack find ${1}_viz`/configs/autonomy.rviz
  fi
}

release(){
  if [[ $# < 1 ]]
  then
    ROS_DISTRO=`git br | grep '*' | awk '{print $2}' | cut -d'-' -f 1`
  else
    ROS_DISTRO=$1
  fi

  catkin_generate_changelog
  git ci -am "Update changelog"
  git push
  catkin_prepare_release
  bloom-release -y `basename `pwd`` --track $ROS_DISTRO --rosdistro $ROS_DISTRO
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

alias catkin_make='catkin_make -j8 -DCMAKE_BUILD_TYPE=RelWithDebInfo '

cm(){
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

alias psgrep='ps aux | grep -i '
alias lgrep='ls | grep -i '
alias fgrep='find | grep -i '
alias dgrep='dpkg -l | grep -i '
alias egrep='env | grep -i '
alias rosdepgrep='rosdep db | grep -i '
alias envgrep='env | grep -i '

alias roscore='roscore >/dev/null 2>&1 &'

# misspelling aliases:
alias rosonde='rosnode '
alias rosndoe='rosnode '

alias joystick='rosrun joy joy_node _dev:=/dev/input/js1 '

roskill() {
  killall -q gazebo gzclient
  psgrep ros | grep -v 'vim\|sublime\|g++\|gcc\|c++\|rosmake\|catkin\|grep' | awk '{print $2}' | xargs -I{} kill {}
  psgrep ros | grep -v 'vim\|sublime\|g++\|gcc\|c++\|rosmake\|catkin\|grep' | awk '{print $2}' | xargs -I{} kill -9 {}
  roscore
}

rosbagfilter() {
  INPUT=$1
  FRAME=$2

  OUTPUT=${INPUT//.bag/_filtered.bag}

  rosbag filter $INPUT $OUTPUT "topic != '/tf' or m.transforms[0].header.frame_id != '$FRAME'"
}

alias deps='rosdep install --from-paths src -iy '

f() {
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

a() {
  ack-grep -ai $*
}

# Pipe to vim
v() {
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

alias sudo='sudo '

alias gg='git log --oneline --abbrev-commit --all --graph --decorate --color'

alias ud='udisks --detach'

alias log-out='gnome-session-quit'
alias x-start='service lightdm restart'

alias off='xset dpms force off'

alias cbuild='catkin build --no-notify '
alias cclean='catkin clean --no-notify '
alias cinstall='catkin build --no-notify --catkin-make-args install '

# Current year calendar
alias cal='cal -y'

# matlab in awesome
# see http://awesome.naquadah.org/wiki/Problems_with_Java
matlab() {
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
set_union() {
  cat "$1" "$2" | sort | uniq
}

set_difference() {
  cat "$1" "$2" "$2" | sort | uniq -u
}

# repeat
repeat() {
  n=$1
  shift
  while [ $(( n -= 1 )) -ge 0 ]
  do
      "$@"
  done
}

