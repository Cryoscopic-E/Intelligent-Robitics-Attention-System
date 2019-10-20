# Intelligent-Robitics-Attention-System

## ICub Instructions

1. `mkdir build`
2. `cd build`
3. `cmake ../`
4. `make`
5. `./yarpy`

> Remember to launch icub from the 'icub' folder containing the .ini file

Use the bash script to launch the yarp views and connect them to the program ports

``` bash
./launch_yarpviews
./connect_yarpviews
```

## Markers Generator Instructions

1. `mkdir build`
2. `cd build`
3. `cmake ../`
4. `make`
5. `./aruco_markers`

Use the slider to change the marker image.
Press 's' to save the current marker.
