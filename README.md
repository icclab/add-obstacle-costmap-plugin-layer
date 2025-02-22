# Add Obstacle Costmap Plugin Layer for Nav2
1. Adds a rectangular costmap window of a configurable size as an obstacle of LETHAL_OBSTACLE cost (254).
2. The plugin can be added to the nav2 params file in the local and/or global costmap params as follows:
```yaml
  add_obstacle_layer:
    plugin: nav2_costmap_2d::AddObstacleLayer
    enabled: True
    update_window_height_m: 1.5
    update_window_width_m: 0.5
```
3. The obstacles are published as a `geometry_msgs/msg/Point` message on the `/summit/sensor_obstacles` in the `map` frame.
```bash
ros2 topic pub /summit/sensor_obstacles geometry_msgs/msg/Point "x: 1.0
y: -4.5
z: 0.0"
```
4. For having the plugin enabled we have to clone, build and source this repo.
```bash
colcon build --symlink-install --packages-select nav2_costmap_2d
. install/setup.bash
```
5. Plugin inspired from [Keepout Filter](https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html) and [Gradient Layer plugin](https://docs.nav2.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html).
6.  Add Obstacle Layer implementation can be found here: [header file](https://github.com/Gaurav-Kapoor-07/navigation2/blob/humble-costmap-update/nav2_costmap_2d/include/nav2_costmap_2d/add_obstacle_layer.hpp) and [source file](https://github.com/Gaurav-Kapoor-07/navigation2/blob/humble-costmap-update/nav2_costmap_2d/plugins/add_obstacle_layer.cpp).