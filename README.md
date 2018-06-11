# Visualization
This package includes several nodes / .launch files for visualization purposes.

## Visualization of road messages
This is a very simple node which receives a topic from type RoadLine.msg and outputs a [Marker](http://docs.ros.org/api/visualization_msgs/html/msg/Marker.html) from type linestrip.


## Visualization of map
This launch files publishes a map using the [map server](http://wiki.ros.org/map_server). Currently the map is only for visualization purposes (occupancy information is not real)!

To use this visualization, make sure that the map server package is installed:

`sudo apt install ros-<distro>-map-server`

## Visualization of Obstacles
Very simple visualization of the ObstaclesArray.msg and outputs a MarkerArray.
