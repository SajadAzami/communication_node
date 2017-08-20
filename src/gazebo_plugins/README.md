export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/workspaces/communication_node/devel/lib  
 note that communication_node is a catkin workspace

""""""""""
 import sys;
 sys.path.append("/home/user/workspaces/communication_node/src/sosvr_gazebo_plugins/scripts");
 from getInfo import  GetInfo;
""""""""""""
add the above script to top of your python file so you can instantiate GetInfo
the object you have created has 1 method called Request
it take 3 args
first one is command ---- it should be "walls" or "distance"
second and third are robot1 and robot2 ---- they are string and can take any value



-------------
also add this to the world file
  <plugin name='sosvr_gazebo_plugin_simple' filename='libsosvr_gazebo_plugin_simple.so'/>
  ----------------------------------------------------------
