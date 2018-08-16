#!/bin/bash

# A script to easily install packages in current workspace into geduino1 and
# geduino2 board, ignoring git files and folders.
# It must be execute from workspace directory.

echo "Scanning for packages to install..."

MAIN_PACKAGES=$(find ./src -maxdepth 1 -mindepth 1 -type d)

echo "Found packages:"
echo $MAIN_PACKAGES
echo ""

echo -n "Do you want to install them (y/n)? "
read answer

if [ "$answer" != "${answer#[Yy]}" ] ;then
    echo "Proceeding with installation...";
else
    exit;
fi

if [ -d "./TEMP" ]; then
	echo "Removing existing TEMP folder...";
	rm -Rf ./TEMP;
fi

echo "Preparing files in TEMP folder ignoring git files and folders... "

mkdir ./TEMP

find $MAIN_PACKAGES -type f -not -path */.git/* -not -name .gitignore -exec cp --parents '{}' './TEMP' \;

echo "Copying files on geduino1..."

scp -r ./TEMP/src ros@geduino1:/home/ros/catkin_ws

echo "Copying files on geduino2..."

scp -r ./TEMP/src ros@geduino2:/home/ros/catkin_ws

echo "Removing TEMP folder...";
rm -Rf ./TEMP;

echo "Installation completed!"

