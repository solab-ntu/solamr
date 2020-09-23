# fake_obstacle_layer
A costmap layer that adds obstacles to layered costmap

## Subscribed Topics

To specify obstacle Poses or empty poses
* **/move_base/global_costmap/fake_obstacle_layer/markedPoses** (geometry_msgs/PoseArray)

## Published Topics

* n/l

## Example of FakeObstacleLayer configuration

    // in global_costmap_params_multi_lidar.yaml
    # "plugins" for post-Hydro setting
    plugins:
        - {name: static_layer,          type: "costmap_2d::StaticLayer"}
        - {name: obstacle_layer,        type: "costmap_2d::ObstacleLayer"} # VoxelLayer
        - {name: fake_obstacle_layer,   type: "fake_obstacle_layer_namespace::FakeObstacleLayer"}
        - {name: inflation_layer,       type: "costmap_2d::InflationLayer"}

## Example rostopic pub command to generate two poses
    $ rostopic pub /move_base/global_costmap/fake_obstacle_layer/markedPoses geometry_msgs/PoseArray "{header: {frame_id: 'map'}, poses: [{position: {x: 2.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, {position: {x: 2.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}]}"


## Example rostopic pub command to NOT generate any poses
    $ rostopic pub /move_base/global_costmap/fake_obstacle_layer/markedPoses geometry_msgs/PoseArray "{header: {frame_id: 'map'}, poses: []}"

