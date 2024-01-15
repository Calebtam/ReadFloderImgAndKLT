// https://wiki.ros.org/dynamic_reconfigure/Tutorials/SettingUpDynamicReconfigureForANode%28cpp%29
#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_tutorials/TutorialsConfig.h>
// 在这里，我们只包含节点所需的头文件。请注意 dynamic_tutorials/TutorialConfig.h，这是 dynamic_reconfigure 从我们的配置文件生成的头文件。

// 这是在向dynamic_reconfigure服务器发送新配置时将调用的回调。它需要两个参数，第一个是新配置。第二个是电平，它是将所有已更改参数的电平值或在一起的结果。
// 你想用等级值做什么完全取决于你，现在没有必要。我们在回调中要做的就是打印出配置。
void callback(dynamic_tutorials::TutorialsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
            config.int_param, config.double_param, 
            config.str_param.c_str(), 
            config.bool_param?"True":"False", 
            config.size);
}

// 在我们的主函数中，我们只需初始化节点并定义dynamic_reconfigure服务器，将我们的配置类型传递给它。只要服务器存在（在本例中直到 main（） 结束），节点就会侦听重新配置请求。
int main(int argc, char **argv) {
  // 初始化节点
  ros::init(argc, argv, "dynamic_tutorials");

  // 定义dynamic_reconfigure服务器
  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig> server;
  dynamic_reconfigure::Server<dynamic_tutorials::TutorialsConfig>::CallbackType f;
   
  // bind 是一组重载的函数模板.用来向一个函数(或函数对象)绑定某些参数.bind的返回值是一个函数对象.
  f = boost::bind(&callback, _1, _2);
  // 将我们的配置类型传递给它
  server.setCallback(f);
  // 接下来，我们定义一个变量来表示我们的回调，然后将其发送到服务器。现在，当服务器收到重新配置请求时，它将调用我们的回调函数。
  // 注意：如果类的成员函数的回调使用 f = boost：：bind（&callback， x， _1， _2），其中 x 是类的实例（如果从类内部调用，则为 this）。

  ROS_INFO("Spinning node");
  ros::spin();
  return 0;
}
