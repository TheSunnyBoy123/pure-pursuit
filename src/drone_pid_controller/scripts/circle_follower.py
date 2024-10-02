# circle follower script for PID controller and ROS integration
import rospy
from pymavlink import mavutil
import math
from std_msgs.msg import Float64

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt 
        output = sel.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
    

# send a velocity command to the drone
def send_velocity_command(mavlink_connection, vx, vy, yaw_rate):
    mavlink_connection.mav.set_position_target_local_ned_send(
        0,
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111, # Position type mask
        0, 0, 0,
        vx, vy, 0,
        0, 0, 0,
        yaw_rate, 0
    )

def follow_circle(radius, velocity, pid_x, pid_y, mavlink_connection, dt = 0.1):
    start_time = rospy.Time.now().to_sec()
    rate = rospy.Rate(1/dt)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec()
        time_passed = current_time - start_time

        x_des = radius * math.cos(time_passed * velocity)
        y_des = radius * math.sin(time_passed * velocity)

        x_current, y_current = get_current_position()

        error_x = x_des - x_current
        error_y = y_des - y_current

        vx = pid_x.update(error_x, dt)
        vy = pid_y.update(error_y, dt)

        send_velocity_command(mavlink_connection, vx, vy, 0)

        rate.sleep()


def get_current_position():
    return 0, 0


if __name__ == "__main__":
    try:
        rospy.init_node("circle_follower")
        mavlink_connection = mavutil.mavlink_connection("udp:127.0.0.1:14551")
        mavlink_connection.wait_heartbeat()

        pid_x = PID(1, 0.1, 0)
        pid_y = PID(1, 0.1, 0)

        radius = rospy.get_param("~radius", 5)
        velocity = rospy.get_param("~velocity", 1)
        dt = rospy.get_param("~dt", 0.1)

        follow_circle(radius, velocity, pid_x, pid_y, mavlink_connection, dt)

    except rospy.ROSInterruptException:
        pass