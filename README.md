
add third_3_party/CMakeLists.txt
solve problem 
<!-- 
CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message):
  Unable to find either executable 'empy' or Python module 'em'...  try
  installing the package 'python3-empy'
Call Stack (most recent call first):
  /opt/ros/noetic/share/catkin/cmake/all.cmake:164 (include)
  /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:20 (include)
  CMakeLists.txt:31 (find_package)
 -->


<!-- # 
sudo update-alternatives --list python

sudo update-alternatives --install /usr/bin/python python /usr/bin/python2.7 1
# update-alternatives: using /usr/bin/python2.7 to provide /usr/bin/python (python) in auto mode
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.5 2
# update-alternatives: using /usr/bin/python3.4 to provide /usr/bin/python (python) in auto mode

sudo update-alternatives --config python



# conda and ros python change
# 1 check your installed python version 
ls /usr/bin/python*

# 2 install the python manage pyenv
curl https://pyenv.run | bash

# 3 add the path to your .bashrc 
echo 'export PATH="$HOME/.pyenv/bin:$
export PATH="$HOME/.pyenv/bin:$PATH" eval "$ (pyenv init -)" eval "$ (pyenv virtualenv-init -)"

# 4 reload ~/.bashrc
source ~/.bashrc

# 5 install the python version you want
pyenv install 3.8.1

# 6 change the python
pyenv global <version> -->