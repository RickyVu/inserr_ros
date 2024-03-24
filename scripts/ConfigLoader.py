#!/usr/bin/env python
import rospy
import yaml
import rospkg


def load_params_from_yaml(file_path):
    with open(file_path, 'r') as file:
        params = yaml.safe_load(file)
    return params

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('config_loader')
    rospack = rospkg.RosPack()
    # Load parameters from a YAML file
    yaml_file = rospy.get_param('~yaml_file', '')  # Get the YAML file path from the ROS parameter server
    if yaml_file[0] != '/':
        yaml_file = f"/{yaml_file}"
    

    
    current_package = rospack.get_path('inserr_ros')
    full_yaml_file = current_package + yaml_file
    if full_yaml_file:
        params = load_params_from_yaml(full_yaml_file)
        rospy.set_param('/', params)  # Set the parameters on the ROS parameter server

    print(rospy.get_param('thrusters', []))
