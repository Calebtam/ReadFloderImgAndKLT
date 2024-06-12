#! /usr/bin/env python3

import os
import argparse
import rospy 
from rospy import Time
import rosbag
import logging
from datetime import datetime, timedelta

# from message_filters import TimeSynchronizer, Subscriber
# import rospy
# from rospy.time import Time, Duration

logging.basicConfig(level=logging.DEBUG)
# topic_name = '/Pvtsln'
s_topic = ['/Pvtsln', '/rtk_fix', 'sub_string_3']

class BagFilter(object):
  def __init__(self, input_bag, output_bag, start_offset, end_offset, topics):
    self.__input_bag = input_bag
    self.__output_bag = output_bag
    self.__start_offset = start_offset
    self.__end_offset = end_offset
    self.__topics = topics

  def filter_bag(self):
    inbag = rosbag.Bag(self.__input_bag, "r")
    start_time = inbag.get_start_time() + self.__start_offset
    end_time = inbag.get_end_time() - self.__end_offset

    with rosbag.Bag(self.__output_bag, "w") as outbag:
      # // 如果不指定 -t ，这里的self.__topics就是空，read_messages()就会把全量topic读出来，很方便

      # # 初始化ros
      # roslib.load_manifest('roslib')

      # # 获取topic的消息类型
      # msg_type = get_message_class(inbag._type_info[topic_name][0])
    
      # 遍历input bag中的消息
      # for topic, msg, t in inbag.read_messages():
          # 增加8小时
          # msg.header.stamp = msg.header.stamp + roslib.rostime.Duration(3600) * 8
          # 将修改后的消息写入到output bag

      for topic, msg, t in inbag.read_messages(topics=self.__topics, raw=False):
        # if start_time <= t.to_sec() <= end_time:
          # if topic_name in topic:
          # contains_all = all(stopic in topic for stopic in s_topic)
          # if contains_all:
          if topic in s_topic:
            print(msg.header.stamp)
            print(t)
            # print("       ")
            # msg.header.stamp = msg.header.stamp + roslib.rostime.Duration(3600) * 8
            # msg.header.stamp = rospy.Time(msg.header.stamp.to_sec() + 8.0)
          
            # # 将消息的时间戳转换为datetime对象
            # msg_time = rospy.Time.to_sec(msg.header.stamp)
            # msg_datetime = datetime.fromtimestamp(msg_time)
            # # 增加八个小时
            # new_datetime = msg_datetime + timedelta(hours=8)
            # # 将新的时间转换为ROS时间
            # new_time = rospy.Time.from_sec(new_datetime.timestamp())
            # msg.header.stamp = new_time

            msg.header.stamp = msg.header.stamp + rospy.Duration(3600) * 8# - rospy.Duration(3)
            t = t + rospy.Duration.from_sec(8 * 3600)
            # print(msg.header.stamp)
            # print(t)
            # print("       ")
            # outbag.write(topic, msg, t)
          else:
            # print(msg.header.stamp)
            # print(topic)
            # print("       ")
            outbag.write(topic, msg, t)
            

      logging.info("------------[%s summary]------------" %self.__output_bag)
      logging.info("time span %f [origin %f]" %(end_time-start_time, inbag.get_end_time()-inbag.get_start_time()))
      logging.info("time span %f [now %f]" %(end_time-start_time, outbag.get_end_time()-outbag.get_start_time()))
      topic_list = [topic for topic in outbag.get_type_and_topic_info()[1].keys()]
      logging.info("topic list %s" %topic_list)

    inbag.close() 
    outbag.close()

def main():
  # // 支持前后时间偏移截取，支持提取指定topic
  parser = argparse.ArgumentParser(description="filter rosbag depends time offset and topics")
  parser.add_argument("-i", "--input_bag", type=str, required=True, help="specify input rosbag")
  parser.add_argument("-o", "--output_bag", type=str, required=True, help="specify output rosbag")
  parser.add_argument("-s", "--start_offset", type=int, default=0, help="specify start offset time")
  parser.add_argument("-e", "--end_offset", type=int, default=0, help="specify end offset time")
  parser.add_argument("-t", "--topics", nargs="+", type=str, help="specify topic")
  parser.add_argument('extra_args', nargs='*', help=argparse.SUPPRESS)
  args=parser.parse_args()

  if not os.path.isfile(args.input_bag):
    logging.error("%s is no found!" %args.input_bag)
    return

  filter = BagFilter(args.input_bag, args.output_bag, args.start_offset, args.end_offset, args.topics)
  filter.filter_bag()

if __name__ == "__main__":
    main()
