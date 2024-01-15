import rospy
# 我们首先导入 rospy 和 dynamic_reconfigure.client。
import dynamic_reconfigure.client

# 然后我们定义一个回调，它将打印服务器返回的配置。此回调与服务器之间有两个主要区别，一是它不需要返回更新的配置对象，二是它没有“level”参数。此回调是可选的。
def callback(config):
    rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))

# 最后，我们初始化 ROS 和我们的客户端。我们的主循环每十秒运行一次，每次都在客户端上调用update_configuration。请注意，您不需要使用完整配置，也可以传入仅包含其中一个参数的字典。
if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback)

    r = rospy.Rate(0.1)
    x = 0
    b = False
    while not rospy.is_shutdown():
        x = x+1
        if x>10:
            x=0
        b = not b
        client.update_configuration({"int_param":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
        r.sleep()