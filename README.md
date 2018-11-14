# improved-RRT-algorithm

This repository contains the improved RRT (Rapid Random Tree) for motion planning problems. The original RRT algorithm is written by Yanjiang 
Zhao and the original code can be find [here](http://www.codeforge.cn/read/218580/pathRRT.m__html). 

The major improvement I have done is the consideration of moving obstacles. In the original only the static obstacles are considered, so the 
collision check are very easy. In order to check the collision with moving obstacles, I have used geometry overlapping method to realize this
check.

Here is a comparison of the original RRT algorithm and the improved one.

# Original RRT (only static obstacles are considered):

![](https://github.com/LihaoWang1991/improved-RRT-algorithm/blob/master/image-and-video/image1.PNG)

Improved RRT (both static and moving obstacles are taken into account)


