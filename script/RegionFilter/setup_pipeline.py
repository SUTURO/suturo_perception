#!/usr/bin/env python

import shutil
import argparse
import rospkg

rospack = rospkg.RosPack()

parser = argparse.ArgumentParser(description='Setup pipeline.')
parser.add_argument("--source", help="Source yaml file",
                    type=str, default="suturo_semantic_map.yaml")
parser.add_argument("--target", help="Target yaml file",
                    type=str, default="hsrb.yaml")

args = parser.parse_args()

path = rospack.get_path("suturo_perception")

name = path + '/config/' + args.source

# shutil.move(path + '/script/RegionFilter/' + args.source, name)

with open(name, 'r') as f:
    source_lines = f.readlines()

sem_list = []
save = False
for line in source_lines:
    if 'names:' in line:
        save = True
        continue
    if ':' in line and '  - ' not in line:
        save = False
    if save:
        sem_list.append(line)

with open(path + '/descriptors/analysis_engines/' + args.target, "r") as f:
    lines = f.readlines()

with open(path + '/descriptors/analysis_engines/' + args.target, 'w') as f:
    write = True
    for line in lines:
        if 'defaultRegions:' in line:
            f.write(line)
            for i in range(len(sem_list)):
                f.write('  ' + sem_list[i])
            write = False

        if 'semantic_map_definition' in line:
            write = True

        if write:
            f.write(line)