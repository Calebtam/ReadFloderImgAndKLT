# >>> fishros initialize >>>
echo "ros choose : foxy:1    noetic:everthing ?"
read choose
case $choose in
1) source  /opt/ros/foxy/setup.bash;;
2) source  /opt/ros/noetic/setup.bash;;
*) source  /opt/ros/noetic/setup.bash;;
esac
# <<< fishros initialize <<<