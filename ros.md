# 关于ros使用

## 创建节点

文件格式

- `${workspace}/src/`
  - `${PACKAGE}`
    - `src`
      - `${NODE}.cpp`
    - `CMakeList.txt`
    - `package.xml`

入口代码结构

```cpp
#include <ros/ros.h>
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "sensor_node");
    // code for setup
    while (ros::ok())
    {
        // code for loop
    }
    // code for end
    return 0;
}

```

1. `catkin_create_pkg` 创建软件包
   1. 路径一般为 `${workspace}/src/`
   2. 标准命令为 `catkin_create_pkg ${PACKAGE_NAME} ${RELA_PACKAGE_NAME}`
   3. 常见依赖项为: `rospy` `roscpp` `std_msgs` , 最后一项为标准信息包
2. 在 `${PACKAGE_NAME}/src` 中创建节点
   1. 环境引用 `#include<ROS.h>`
   2. 命令行参数 `int argc, char *argv[]`
   3. ros 环境初始化 `ros::init(argc, argv, "${NODE_NAME}");`
   4. 循环标记 `ros::ok()`
3. 在 `${PACKAGE_NAME}/CMakeList.txt` 中
   1. 编译规则在其中说明
   2. 在 `Build` 小节中 `target_link_libraries(${NODE_NAME} ${catkin_LIBRARIES})`
   3. 在 `Build` 小节中 `add_executable(${NODE_NAME} src/${NODE_NAME}.cpp)` 添加编译项
4. 在 `${PACKAGE_NAME}/xml` 中
   1. 是 `catkin` 软件包的通用配置文件
5. 运行 Node 节点
   1. 启动 ROS 核心,在终端中 `roscore`
   2. 将包引入工作空间 `source ${workspace}/devel/setup.bash`
   3. 执行 Node 节点 `rosrun ${PACKAGE_NAME} ${NODE_NAME}`

## 通讯 Topic_Message 模式

文件结构

- `${workspace}/src/`
  - `${Publisher_PACKAGE}`
  - `${Subsciber_PACKAGE}`

1. Publisher
   1. 在 `${NODE_NAME},cpp` 中添加节点句柄 `ros::NodeHandle ${NODE_HANDLE_NAME};`
   2. 在 `${NODE_NAME},cpp` 中添加发布者对象 `ros::Publisher ${PUBLISHER_NAME}`
      1. `ros::Publisher ${PUBLISHER_NAME} = ${NODE_HANDLE}.advertise<std_msgs::${MASSAGE_TYPE}>(${TOPIC_NAME},${TOPIC_BUFFER});`
   3. 创建消息对象 `std_msgs::${MASSAGE_TYPE} ${MASSAGE_NAME};`
   4. 发布消息 `${PUBLISHER}.publish(${MASSAGE})`
2. Subsciber
   1. 在 `${NODE_NAME},cpp` 中添加节点句柄 `ros::NodeHandle ${NODE_HANDLE_NAME};`
   2. 在 `${NODE_NAME},cpp` 中添加接受者对象 `ros::Subsciber ${PUBLISHER_NAME}`
      1. 声明回调函数，形参为 `std_msgs::${MASSAGE_TYPE} ${MASSAGE_NAME}`,返回 `void`
      2. `ros::Publisher ${SUBSCIBER_NAME} = ${NODE_HANDLE}.subscibe(${TOPIC_NAME},${TOPIC_BUFFER},${CALLBACK_FUNC});`
   3. 消息查询探针 `ros::spinOnce();`
3. 管理话题
   1. 查看当前 ROS 中的话题 `rostopic list`
   2. 以图像形式查看当前 ROS 中的话题 `rqt_graph`
   3. 查看 ROS 中话题中的消息 `rostopic echo ${TOPIC}`
   4. 查看 ROS 中话题的频率 `rostopic hz ${TOPIC}`
   5. 一个阻塞式控制循环频率的方法 `ros::Rate ${Rate_NAME}` + `${Rate}.sleep()`

## launch 启动方式

文件结构

- `${workspace}/src/`
  - `${PACKAGE}`
    - `launch/${LAUNCH_NAME}.launch`

代码结构

```xml
<launch>
   <node pkg=${PACKAGE_NAME} type=${NODE_NAME} name=${NAME}>
</launch>
```

运行方式 `roslaunch ${LAUNCH}`

## moveit相关内容

教程网址 <https://ros-planning.github.io/moveit_tutorials/>

1. 演示
   1. 启动演示
      1. `roslaunch panda_moveit_config demo.launch`
      2. `roslaunch moveit_tutorials move_group_interface_tutorial.launch`
2. 代码结构
   1. 机器人模型及状态
      1. `RobotModel` 类
         1. 连接和关节关系、约束。
         2. SRDF？<https://ros-planning.github.io/moveit_tutorials/doc/urdf_srdf/urdf_srdf_tutorial.html>
         3. 在ros参数服务器上下载机器人参数. `RobotModelLoader` 类
            1. `robot_model_loader::RobotModelLoader robot_model_loader("robot_description");` ==这里的 robot_description 是枚举型参数还是描述性参数不太明白==
            2. `const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();`
         4. 模型参数可以输出为字符串 `kinematic_model->getModelFrame().c_str()`
      2. `RobotState` 类
         1. 机器人在某个时间点的信息
            1. 关节位置向量
            2. 速度
            3. 加速度
            4. 姿态 `Eigen::Isometry3d`
               1. 平移 `translation()`
               2. 旋转 `rotation()`
         2. 根据末端执行器位置反接手臂位置和笛卡尔轨迹相关函数……
         3. 从机器人模型中获取初始姿态
            1. 派生机器人姿态类`moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));`
            2. 设置为默认姿态`kinematic_state->setToDefaultValues();`
            3. 获取默认姿态的当前关节表`const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");` 在这里会指定机器人类型
            4. 获取默认姿态的当前关节名称`const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();`
         4. 从机器人状态中获取当前姿态
            1. `kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);`关节名称来自上一小节
         5. 设置关节角度 `setJointGroupPositions（）`,可以用 `kinematic_state->enforceBounds();` 增加关节限制约束
         6. 获取雅可比
         7. 命令行获取系统 `rosrun moveit_ros_planning moveit_print_planning_model_info`
   2. 运动学
      1. 正向运动学
         1. 给定关节参数`kinematic_state->setToRandomPositions(joint_model_group);`
         2. 得到末端姿态`const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");`
      2. 逆运动学 IK 我们似乎用不到……
   3. 显示相关
      1. 规划组关节集名称 `static const std::string PLANNING_GROUP = "panda_arm";`
      2. `MoveGroupInterface` 类
         1. 用于控制和规划
         2. 创建对象 `moveit::planning_interface::MoveGroupInterface ${MOVE_GROUP_NAME}(PLANNING_GROUP);`
      3. `PlanningSceneInterface` 类
         1. 用于管理碰撞对象
         2. 创建对象 `moveit::planning_interface::PlanningSceneInterface ${PLANNING_SCENE_NAME};`
      4. `JointModelGroup` 类
         1. 存储计划组指针
         2. `const moveit::core::JointModelGroup* joint_model_group = ${MOVE_GROUP}.getCurrentState()->getJointModelGroup(PLANNING_GROUP);`
