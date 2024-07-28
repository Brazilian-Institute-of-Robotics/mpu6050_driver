#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import yaml
import os
import time

class FlowStyleListDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(FlowStyleListDumper, self).increase_indent(flow=True)

class IMUCalibrationListener:
    def __init__(self):
        self.node_name = 'mpu_calibration_listener'
        rospy.init_node(self.node_name, anonymous=True)
        
        self.yaml_path = rospy.get_param('~mpu_settings_path', '')
        if not self.yaml_path:
            rospy.logerr("YAML file path must be specified as a private parameter.")
            exit(1)

        self.last_time_received = time.time()
        self.timeout_duration = 5  # Time in seconds to wait after the last message
        self.last_offsets = None
        
        rospy.Subscriber('/imu_offsets', Imu, self.imu_callback)

        rospy.loginfo(f"{self.node_name} started.")
        rospy.loginfo("Please ensure the IMU is placed level on a flat surface and remains motionless during calibration.")
        rospy.loginfo(f"Loading configuration from: {self.yaml_path}")

    def imu_callback(self, data):
        self.last_time_received = time.time()
        self.last_offsets = [
            data.linear_acceleration.x,
            data.linear_acceleration.y,
            data.linear_acceleration.z,
            data.angular_velocity.x,
            data.angular_velocity.y,
            data.angular_velocity.z
        ]

    def save_offsets_to_yaml(self):
        # Load the original YAML file
        with open(self.yaml_path, 'r') as file:
            data = yaml.safe_load(file)

        # Update the offsets
        data['axes_offsets'] = [int(offset) for offset in self.last_offsets]

        # Generate the path for the new YAML file
        new_yaml_path = os.path.join(os.path.dirname(self.yaml_path), 'new_mpu_settings.yaml')

        # Save the updated data in a new YAML file using the custom settings
        with open(new_yaml_path, 'w') as file:
            yaml.dump(data, file, Dumper=FlowStyleListDumper, default_flow_style=None)

        rospy.loginfo(f"Calibration complete. Offsets saved to: {new_yaml_path}")
    
    def run(self):
        rate = rospy.Rate(1)  # Check every second
        while not rospy.is_shutdown():
            current_time = time.time()
            if current_time - self.last_time_received > self.timeout_duration:
                if self.last_offsets:
                    rospy.loginfo("No new messages from the calibration node. Assuming convergence.")
                    self.save_offsets_to_yaml()
                    rospy.signal_shutdown("Calibration Converged")
                else:
                    rospy.logwarn("Timeout reached but no offsets received.")
            else:
                rospy.loginfo("Waiting for measurements to converge...")
            rate.sleep()

if __name__ == '__main__':
    imu_cal_listener = IMUCalibrationListener()
    imu_cal_listener.run()
