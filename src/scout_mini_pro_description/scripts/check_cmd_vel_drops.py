#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
检查 /scout_mini_pro/cmd_vel 是否有丢包或明显延迟。
订阅该话题，统计接收频率和「长时间间隔」次数（间隔 > 0.1s 视为可能丢包或卡顿）。
用法：导航运行时在另一终端执行
  rosrun scout_mini_pro_description check_cmd_vel_drops.py
可选参数：cmd_vel_topic 默认为 /scout_mini_pro/cmd_vel
  rosrun scout_mini_pro_description check_cmd_vel_drops.py _cmd_vel_topic:=/scout_mini_pro/cmd_vel
"""

import rospy
from geometry_msgs.msg import Twist

# 纯追踪控制周期约 0.05 s (20 Hz)，间隔超过此阈值 2 倍视为异常
EXPECTED_PERIOD = 0.05   # s
GAP_THRESHOLD = 0.1      # s，超过则认为可能丢包或严重延迟


def main():
    rospy.init_node("check_cmd_vel_drops", anonymous=True)
    topic = rospy.get_param("~cmd_vel_topic", "/scout_mini_pro/cmd_vel")

    last_time = [None]  # 用 list 以便在 callback 里修改
    count = [0]
    gap_count = [0]
    window_start = [None]
    window_count = [0]

    def cb(msg):
        t = rospy.Time.now()
        if last_time[0] is not None:
            dt = (t - last_time[0]).to_sec()
            if dt > GAP_THRESHOLD:
                gap_count[0] += 1
        last_time[0] = t
        count[0] += 1
        window_count[0] += 1

    rospy.Subscriber(topic, Twist, cb, queue_size=10)
    rospy.loginfo("check_cmd_vel_drops: 订阅 %s，每 2 秒打印一次统计（间隔>%.2fs 计为可能丢包）", topic, GAP_THRESHOLD)

    r = rospy.Rate(0.5)  # 每 2 秒
    while not rospy.is_shutdown():
        r.sleep()
        if window_start[0] is None and last_time[0] is not None:
            window_start[0] = last_time[0]
        if window_start[0] is None:
            rospy.loginfo_throttle(2.0, "check_cmd_vel_drops: 尚未收到消息，请确认有节点在发布 %s", topic)
            continue
        elapsed = (rospy.Time.now() - window_start[0]).to_sec()
        if elapsed < 0.01:
            continue
        rate = window_count[0] / elapsed
        hint = " (预期约 20 Hz，若明显偏低或长间隔很多可能有丢包/卡顿)" if rate < 15 or gap_count[0] > 0 else ""
        rospy.loginfo(
            "cmd_vel 统计: 近 2s 接收 %.1f msg/s, 本窗口消息数 %d, 长间隔(>%.2fs)次数 %d%s",
            rate, window_count[0], GAP_THRESHOLD, gap_count[0], hint
        )
        # 重置窗口，便于看近期速率
        window_start[0] = rospy.Time.now()
        window_count[0] = 0


if __name__ == "__main__":
    main()
