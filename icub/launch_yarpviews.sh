#! /bin/sh

yarpview --name /img_proc/th --x 100 --y 100 &
yarpview --name /img_proc/sd --x 500 --y 100 &
yarpview --name /img_proc/fd --x 100 --y 500 &
yarpview --name /img_proc/md --x 500 --y 500 &
yarpview --name /img_proc/cd --x 900 --y 100
