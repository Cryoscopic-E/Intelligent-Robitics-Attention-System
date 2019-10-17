#! /bin/sh

yarpview --name /img_proc/th --x 100 --y 100 &
yarpview --name /img_proc/sd --x 500 --y 100 &
yarpview --name /img_proc/fd --x 100 --y 500 &
yarpview --name /img_proc/md --x 500 --y 500 &
sleep 1s
yarp connect /img_proc/threshold /img_proc/th
yarp connect /img_proc/sobel /img_proc/sd
yarp connect /img_proc/face /img_proc/fd
yarp connect /img_proc/marker /img_proc/md