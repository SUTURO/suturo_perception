#!/usr/bin/env python
# rosrun object_state semantic_map_setup.py

import rospy
import os
import yaml
import tf
import rospkg
import argparse
import functools
import sys
import tf.transformations as tft

rospack = rospkg.RosPack()

opencv_data = dict(names=[])

world_offset_x = 0
world_offset_y = 0
world_offset_z = 0
world_offset_rot_z = 0
element_list = []
prefix = ""


def create_element(listener, object_info, object_type, suffix):
    """Create elements with one region."""

    rospy.loginfo("Getting TF pose of /" + prefix + "/" + suffix + "_surface_center")
    now = rospy.Time(0)
    # trans = [0, 0, 0]
    # rot = [0.000, 0.000, 0.0, 0.0]
    (trans, rot) = listener.lookupTransform("/map", "/" + prefix + "/" + suffix + "_surface_center", now)
    pose = tf_to_region_matrix(trans, rot)
    opencv_data[suffix] = to_opencv_dict(object_info, object_type, pose)
    opencv_data['names'].append(suffix)


def create_element_with_floors(listener, object_info, object_type, suffix):
    """Create elements with more that one region."""

    shelf_region_frame_map = []
    num_floors = int(object_info['number_of_floors'])
    for n in range(0, num_floors):
        shelf_region_frame_map.append([suffix + "_floor_" + str(n),
                                       "/" + prefix + "/" + suffix + "_floor_" + str(n) + "_piece"])

    rospy.loginfo("Getting TF data of " + suffix + " floor " + str(num_floors) + ".")
    for (region, frame) in shelf_region_frame_map:
        opencv_data['names'].append(region)
        now = rospy.Time(0)
        # trans = [0, 0, 0]
        # rot = [0.000, 0.000, 0.0, 0.0]
        (trans, rot) = listener.lookupTransform("/map", frame, now)
        pose = tf_to_region_matrix(trans, rot)
        opencv_data[region] = to_opencv_dict(object_info, object_type, pose)


def write_region_filter_yaml_from_manual(source, target_path):
    """Read base YAML and create YAML for the region filter."""

    rospy.loginfo("Initialsing TF listener.")
    listener = tf.TransformListener()

    # Load base YAML
    sem_map_manual_yaml_path = rospack.get_path("suturo_resources") + "/urdf/yaml/" + source
    rospy.loginfo("Reading data from " + sem_map_manual_yaml_path)
    sem_map_yaml = dict()
    if os.path.exists(sem_map_manual_yaml_path):
        with open(sem_map_manual_yaml_path, "r") as file:
            rospy.loginfo("Loading data as yaml.")
            sem_map_yaml = yaml.load(file.read())
    else:
        rospy.logwarn("The file " + sem_map_manual_yaml_path + " does not exist.")

    # Read offsets
    global world_offset_x, world_offset_y, world_offset_rot_z
    world_offset_x = sem_map_yaml['world_offset']['X']
    world_offset_y = sem_map_yaml['world_offset']['Y']
    world_offset_rot_z = sem_map_yaml['world_offset']['zrot']

    # Create Region
    for element in element_list:
        sem_ele = sem_map_yaml[element]
        for index in range(sem_ele['amount']):
            if 'number_of_floors' in sem_ele[index]:
                create_element_with_floors(listener, sem_ele[index], element, sem_ele[index]['name'])
            else:
                create_element(listener, sem_ele[index], element, sem_ele[index]['name'])

    # Create YAML
    rospy.loginfo("Writing data to " + target_path)
    write_dict_to_ocv_yaml(opencv_data, target_path)


def tf_to_region_matrix(trans, rot):
    """Create pose for the region."""

    pose = tft.quaternion_matrix(rot)

    pose[0][3] = trans[0] + world_offset_x
    pose[1][3] = trans[1] + world_offset_y
    pose[2][3] = trans[2] + world_offset_z

    rotation = tft.quaternion_from_euler(0.0, 0.0, world_offset_rot_z)
    pose = tft.quaternion_multiply(pose, rotation)

    return pose


def to_opencv_dict(input_dict, object_type, pose):
    """Create opencv dictionary."""

    ocv_dict = dict()
    ocv_dict['type'] = object_type
    ocv_dict['width'] = input_dict['width']
    ocv_dict['height'] = input_dict['height']
    ocv_dict['depth'] = input_dict['depth']
    ocv_dict_t = dict()
    ocv_dict_t['rows'] = 4
    ocv_dict_t['cols'] = 4
    ocv_dict_t['dt'] = 'd'
    ocv_dict_t['data'] = str(functools.reduce(lambda a, b: a + b, pose.tolist()))
    ocv_dict['transform'] = ocv_dict_t
    return ocv_dict


def write_dict_to_ocv_yaml(dictionary, filename):
    with open(filename, 'w') as outfile:
        yaml.dump(dictionary, outfile, default_flow_style=False)

    opencv_yaml_flavor = "%YAML:1.0\n"
    with open(filename, 'r') as outfile:
        opencv_yaml_flavor += outfile.read().replace('\'', '').replace("transform:", "transform: !!opencv-matrix")

    with open(filename, 'w') as outfile:
        outfile.write(opencv_yaml_flavor)


def csv_list(string):
    return string.split(',')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='YAML Creator')
    parser.add_argument("--source", help="Source yaml file",
                        type=str, default="gz_sim_v1.yaml")
    parser.add_argument("--target", help="Result file name",
                        type=str, default="semantic_map.yaml")
    parser.add_argument("--prefix", help="Element prefix z.B iai_kitchen",
                        type=str, default="iai_kitchen")
    parser.add_argument("--elements", help="Elements containing the regions",
                        type=csv_list, default=["tables", "shelves"])
    args = parser.parse_args()

    rospy.init_node('semantic_map_setup', anonymous=True)

    element_list = args.elements
    prefix = args.prefix

    write_region_filter_yaml_from_manual(args.source, args.target)
