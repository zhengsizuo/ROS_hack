### 把工作空间写进bachrc
-写入bash文件：echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc
### 创建工作空间
'''
    $ mkdir -p ~/catkin_ws/src  //创建文件夹
    $ cd ~/catkin_ws/src        //定位到src
    $ catkin_init_workspace     //对工作空间进行初始化
    编译
    $ cd ~/catkin_ws/  
    $ catkin_make
'''
### 创建功能包
 -catkin_create_pkg my_demo roscpp rospy std_msgs

### 有关话题
-rostopic list
-rostopic info [TOPIC_NAME]:查看话题的发布者、订阅者以及类型
-rostopic ehco [TOPIC_NAME]:打印话题所包含的消息的内容

**查看服务与消息类似**
-rosservice list

### 有关消息
-查看消息的具体格式：
rosmsg show bmirobot_msg/Robot_ctr.msg
-查看服务也类似：
rsosrv show [SERVICE_NAME]

**发布消息：**
-publish speed topic:
rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'

### 可视化话题之间的关系:
-rqt_graph
-rosrun rqt_graph rqt_graph

### 打开turtlesim节点：
-rosrun turtlesim turtlesim_node

### 有关编译
-catkin_make
-catkin build

