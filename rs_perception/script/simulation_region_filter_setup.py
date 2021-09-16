#!/usr/bin/env python
# rosrun object_state semantic_map_setup.py

import rospy
import os
import yaml
import tf
import rospkg
from tf.transformations import euler_matrix, quaternion_matrix
import functools
import sys

rospack = rospkg.RosPack()


def write_region_filter_yaml_from_manual(target_path='semantic_map.yaml'):
    rospy.loginfo("Initialsing TF listener.")
    listener = tf.TransformListener()

    sem_map_manual_yaml_path = rospack.get_path("suturo_urdf_publisher") + "/urdf/yaml/hsrb_sim_v1_semantic_map_manual_data.yaml"
    rospy.loginfo("Reading data from " + sem_map_manual_yaml_path)
    sem_map_yaml = dict()
    if os.path.exists(sem_map_manual_yaml_path):
        with open(sem_map_manual_yaml_path, "r") as file:
            rospy.loginfo("Loading data as yaml.")
            sem_map_yaml = yaml.load(file.read())
    else:
        rospy.logwarn("The file " + sem_map_manual_yaml_path + " does not exist.")

    opencv_data = dict(names=[])
    region_bottom_offset = sem_map_yaml['surface_bottom_offset']
    region_top_offset = sem_map_yaml['surface_top_offset']
    region_height = region_bottom_offset + region_top_offset
    
    #bed_table : 
    rospy.loginfo("Getting TF pose of /iai_kitchen/bed_table_surface_center")
    now = rospy.Time(0)
    (trans, rot) = listener.lookupTransform("/map", "/iai_kitchen/bed_table_surface_center", now)
    pose = tf_to_region_matrix(trans, rot, region_bottom_offset, region_top_offset)
    opencv_data['bed_table'] = to_opencv_dict(sem_map_yaml['bed_table'], pose, region_height)
    opencv_data['names'].append('bed_table')
    
    #table_clone : 
    rospy.loginfo("Getting TF pose of /iai_kitchen/table_clone_surface_center")
    now = rospy.Time(0)
    (trans, rot) = listener.lookupTransform("/map", "/iai_kitchen/table_clone_surface_center", now)
    pose = tf_to_region_matrix(trans, rot, region_bottom_offset, region_top_offset)
    opencv_data['table_clone'] = to_opencv_dict(sem_map_yaml['table_clone'], pose, region_height)
    opencv_data['names'].append('table_clone')
    
    #table : 
    rospy.loginfo("Getting TF pose of iai_kitchen/table_surface_center")
    now = rospy.Time(0)
    (trans, rot) = listener.lookupTransform("/map", "iai_kitchen/table_surface_center", now)
    pose = tf_to_region_matrix(trans, rot, region_bottom_offset, region_top_offset)
    opencv_data['table'] = to_opencv_dict(sem_map_yaml['table'], pose, region_height)
    opencv_data['names'].append('table')
    
    #hcr_shefl_2 : 
    shelf_region_frame_map = []
    num_floors = int(sem_map_yaml['hcr_shefl_2']['num_floors'])
    for n in range(0, num_floors):
        shelf_region_frame_map.append(["hcr_shefl_2_" + str(n), "iai_kitchen/hcr_shefl_2_floor_" + str(n) + "_piece"])
    rospy.loginfo("Getting TF data of " + str(num_floors) + " floors.")
    for (region, frame) in shelf_region_frame_map:
        opencv_data['names'].append(region)
        (trans, rot) = listener.lookupTransform("/map", frame, now)
        pose = tf_to_region_matrix(trans, rot, region_bottom_offset, region_top_offset)
        opencv_data[region] = to_opencv_dict(sem_map_yaml['hcr_shefl_2'], pose, region_height)
        
    #bookshelf_clone_0 : 
    shelf_region_frame_map = []
    num_floors = int(sem_map_yaml['bookshelf_clone_0']['num_floors'])
    for n in range(0, num_floors):
        shelf_region_frame_map.append(["bookshelf_clone_0_" + str(n), "iai_kitchen/bookshelf_clone_0_floor_" + str(n) + "_piece"])
    rospy.loginfo("Getting TF data of " + str(num_floors) + " floors.")
    for (region, frame) in shelf_region_frame_map:
        opencv_data['names'].append(region)
        (trans, rot) = listener.lookupTransform("/map", frame, now)
        pose = tf_to_region_matrix(trans, rot, region_bottom_offset, region_top_offset)
        opencv_data[region] = to_opencv_dict(sem_map_yaml['bookshelf_clone_0'], pose, region_height)
    
    #bookshelf : 
    shelf_region_frame_map = []
    num_floors = int(sem_map_yaml['bookshelf']['num_floors'])
    for n in range(0, num_floors):
        shelf_region_frame_map.append(["bookshelf_" + str(n), "iai_kitchen/bookshelf_floor_" + str(n) + "_piece"])
    rospy.loginfo("Getting TF data of " + str(num_floors) + " floors.")
    for (region, frame) in shelf_region_frame_map:
        opencv_data['names'].append(region)
        (trans, rot) = listener.lookupTransform("/map", frame, now)
        pose = tf_to_region_matrix(trans, rot, region_bottom_offset, region_top_offset)
        opencv_data[region] = to_opencv_dict(sem_map_yaml['bookshelf'], pose, region_height)
        
    rospy.loginfo("Writing data to " + target_path)
    write_dict_to_ocv_yaml(opencv_data, target_path)


def tf_to_region_matrix (trans, rot, bottom_offset, top_offset):
    pose = quaternion_matrix(rot)
    pose[0][3] = trans[0]
    pose[1][3] = trans[1]
    pose[2][3] = trans[2] + (top_offset - bottom_offset)/2
    return pose


def to_opencv_dict(input_dict, pose, region_height):
    ocv_dict = dict()
    ocv_dict['type'] = input_dict['type']
    ocv_dict['width'] = input_dict['width']
    ocv_dict['height'] = region_height
    ocv_dict['depth'] = input_dict['depth']
    ocv_dict_t = dict()
    ocv_dict_t['rows'] = 4
    ocv_dict_t['cols'] = 4
    ocv_dict_t['dt'] = 'd'
    ocv_dict_t['data'] = str(functools.reduce(lambda a, b: a + b, pose.tolist()))
    ocv_dict['transform'] = ocv_dict_t
    return ocv_dict


def write_dict_to_ocv_yaml (dictionary, filename):
    with open(filename, 'w') as outfile:
        yaml.dump(dictionary, outfile, default_flow_style=False)

    opencv_yaml_flavor = "%YAML:1.0\n"
    with open(filename, 'r') as outfile:
        opencv_yaml_flavor += outfile.read().replace('\'', '').replace("transform:", "transform: !!opencv-matrix")

    with open(filename, 'w') as outfile:
        outfile.write(opencv_yaml_flavor)


if __name__ == '__main__':
    rospy.init_node('semantic_map_setup', anonymous=True)
    if len(sys.argv) > 1:
        write_region_filter_yaml_from_manual(sys.argv[1])
    else:
        write_region_filter_yaml_from_manual()
