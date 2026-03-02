#!/usr/bin/env python3
import rospy
import socket
import subprocess
import os
import signal
import time
from geometry_msgs.msg import PoseStamped

# === 动态查找 rosbag record 并优雅终止 ===
def stop_rosbag_record():
    try:
        # 查找 rosbag record 的 PID
        result = subprocess.check_output("ps -ef | grep 'rosbag record' | grep -v grep", shell=True).decode()
        lines = result.strip().split('\n')
        for line in lines:
            if 'rosbag record' in line:
                pid = int(line.split()[1])
                rospy.loginfo(f"正在结束 rosbag record 进程 PID={pid}")
                os.kill(pid, signal.SIGINT)  # 发送 SIGINT（模拟 Ctrl+C）

                # 等待其完全退出
                for i in range(10):
                    time.sleep(1)
                    if not process_alive(pid):
                        rospy.loginfo("rosbag record 已退出并完成 .bag 写入")
                        return
                rospy.logwarn("等待超时 rosbag 可能未完整写入")
    except Exception as e:
        rospy.logwarn(f"终止 rosbag 失败: {e}")

# 检查某个进程是否还活着
def process_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False

# === 清理 tmux 会话 ===
def cleanup_tmux():
    rospy.loginfo("Cleaning up tmux session 'sim_auto' ...")
    try:
        subprocess.run(["tmux", "kill-session", "-t", "sim_auto"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        rospy.logwarn(f"tmux 清理失败: {e}")

# === Ctrl+C 信号处理 ===
def signal_handler(sig, frame):
    rospy.loginfo("Caught Ctrl+C, shutting down...")
    stop_rosbag_record()
    cleanup_tmux()
    rospy.signal_shutdown("KeyboardInterrupt")
    exit(0)

# === 主程序 ===
def start_server():
    rospy.init_node("multi_pose_socket_server", anonymous=True)

    publishers = {}
    launched = False
    host = "0.0.0.0"
    port = 5566

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port))
    server.listen(1)

    rospy.loginfo(f"📡 Waiting for Isaac Sim on port {port}...")
    conn, addr = server.accept()
    rospy.loginfo(f"Connected to Isaac Sim: {addr}")
    conn.settimeout(1.0)

    while not rospy.is_shutdown():
        try:
            data = conn.recv(1024)
            if not data:
                continue

            lines = data.decode("utf-8").strip().split("\n")
            for line in lines:
                parts = line.strip().split()
                if len(parts) != 8:
                    rospy.logwarn("Invalid message format")
                    continue

                name = parts[0]
                x, y, z = map(float, parts[1:4])
                qx, qy, qz, qw = map(float, parts[4:8])

                if name not in publishers:
                    topic_name = f"/target_pose/{name}"
                    publishers[name] = rospy.Publisher(topic_name, PoseStamped, queue_size=10)
                    rospy.loginfo(f"Created publisher for {topic_name}")

                if not launched:
                    rospy.loginfo("First data received, launching auto_start_system.sh ...")
                    script_dir = os.path.dirname(os.path.abspath(__file__))
                    auto_start_script = os.path.join(script_dir, "auto_start_system.sh")
                    subprocess.Popen([auto_start_script])
                    launched = True

                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "map"
                msg.pose.position.x = x
                msg.pose.position.y = y
                msg.pose.position.z = z
                msg.pose.orientation.x = qx
                msg.pose.orientation.y = qy
                msg.pose.orientation.z = qz
                msg.pose.orientation.w = qw

                publishers[name].publish(msg)
                rospy.loginfo(f"🛰 Published {name} to /target_pose/{name}: ({x:.2f}, {y:.2f}, {z:.2f})")

        except socket.timeout:
            continue
        except Exception as e:
            rospy.logerr(f"Socket error: {e}")
            break

# === 注册 Ctrl+C 信号处理器并运行 ===
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    try:
        start_server()
    except rospy.ROSInterruptException:
        pass

