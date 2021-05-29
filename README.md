# Research Track 1 - final assignment

This package is the result of my work on the final assignment of Reaserch Track module 1 course.

## General information

Once runned , this package will provide an interface and nodes to control a robot in a Gazebo simulation, allowing the user to chose between different behaviours for the robot to navigate the environment. In order to be able to work with this package, also the package **movebase** and **gmapping** are needed.

The bug algoritm has a bug that I'm not being able to solve: sometimes when a goal is aborted because the time expired, if relauched the bug algoritm crashes.

## How to run 

In the *launch* folder of this package there are 4 launch files, in order to set everything up please run in order :

```
roslaunch final_assignment gmapping.launch
```
```
roslaunch final ssignment movebase.launch
```
```
roslaunch final assignment main.launch
```

At this point the interface should be online

## Additional documentation

To find the complete documentation for this project go in *docs/html* folder and open **index.html**

##Rqt_Graph

![alt rqt_graph](https://github.com/BullshidoArtist/RT1_Assignment2/blob/master/rosgraph.png)
