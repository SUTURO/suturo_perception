#!/bin/bash

#This bash file creates a .yaml file in the format that a split file for feature extraction requires.
#Copy this bash file in the directory above the one you want to extract the names out of and start it.

echo Enter your preffered file name for the .yaml file:
read filename
echo Enter the directory out of which you want to extract the class names:
read directory

touch ./$filename
printf "%%YAML:1.0\nclasses:\n" > $filename

for d in ./$directory/*;
    do [[ -d "$d" ]] && echo "  - \"${d##*/}\"" >> $filename;
done
