#!/bin/bash

#Copy this bash file in the directory above the one you want to extract the names out of and start it.

echo Enter your preffered file name for the .yaml file:
read filename
echo Enter the directory out of which you want to extract the class names:
read directory

touch ./$filename
echo -n "[" >> $filename

for d in ./$directory/*;
    do [[ -d "$d" ]] && echo -n "${d##*/}, " >> $filename;
done

echo -n "]" >> $filename
