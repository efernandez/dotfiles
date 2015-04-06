
alias rosmake='ROS_PARALLEL_JOBS=-j6 nice rosmake'

function uri(){
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
        export ROS_MASTER_URI=http://$1:11311
      else
        export ROS_MASTER_URI=http://$1:$2
      fi
    fi
  fi
}

function uln(){
  TARGET=`ls -l $1 | awk '{print $11}'`
  unlink $1
  cp $TARGET $1
}

function rviz(){
  if [[ $# < 1 ]]
  then
    rosrun rviz rviz
  else
    rosrun rviz rviz -d `rospack find ${1}_2dnav`/config/rviz/navigation.rviz
  fi
}

function release(){
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

function cm(){
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

roskill() {
  killall gazebo gzclient
  psgrep ros | grep -v vim | grep -v sublime | grep -v 'g++' | grep -v rosmake | cut -d' ' -f1 | xargs -I{} kill {}
  psgrep ros | grep -v vim | grep -v sublime | grep -v 'g++' | grep -v rosmake | cut -d' ' -f1 | xargs -I{} kill -9 {}
  roscore >/dev/null 2>&1 &
}

function f() {
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

function a() {
  ack-grep -ai $*
}

# Pipe to vim
function v() {
  $@ | vim -R -
}

alias ws='source devel/setup.bash'

# ls aliases
alias ll='ls -rthal'
alias la='ls -A'
alias l='ls -CF'
alias o='xdg-open'
alias k='pkill -9'
alias c='clear'
alias x='exit'
alias q='exit'
alias ..='cd ..'
alias ...='cd ../..'
alias .....='cd ../../..'
alias ......='cd ../../../..'
alias .......='cd ../../../../..'
alias -- -='cd -'

alias grepnode='rosnode list | grep'
alias greptopic='rostopic list | grep'
alias grepservice='rosservice list | grep'
alias grepparam='rosparam list | grep'

alias sudo='sudo '

alias gg='git log --oneline --abbrev-commit --all --graph --decorate --color'

alias ud='udisks --detach'

alias log-out='gnome-session-quit'
alias x-start='service lightdm restart'

# matlab in awesome
# see http://awesome.naquadah.org/wiki/Problems_with_Java
function matlab() {
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

