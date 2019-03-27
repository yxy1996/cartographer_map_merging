# cartographer_map_merging


Building
--------

Build as standard catkin packages. There are no special dependencies needed
(use rosdep to resolve dependencies in ROS). You should use brach specific for
your release i.e. `kinetic-devel` for kinetic. Master branch is for latest ROS.


WIKI
----

The original packages are documented at ROS wiki.
* [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge)



Modification
----

Modified to support cartographer, as the origin version does not work.



Launch
----

```
 	 roslaunch super_multirobot_map_merge map_merge.launch 
```



COPYRIGHT
---------

Packages are licensed under BSD license. See respective files for details.
