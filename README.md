# Actor eStop Node

Forwards a `geometry_msgs/Twist`, or publishes all zeros at 50 hz

- Stopped and resumed via `std_srvs` services
  - `std_srvs/Empty` for `~stop` service
  - `std_srvs/Trigger` for `~resume` service
- Starts in stopped state
- Only one node has "permission" to reset estop
  - The node that resumes first gets permission
- Specify topic to forward in rosparam `~input_topic`
- Publishes output to `~out` by default
  - Can change with `~output_topic` rosparam

### Example ROS launch file

```xml
<launch>
  <node pkg="ltu_actor_estop" type="estop" name="estop">
    <param name="input_topic" value="/example/input/topic" />
    <param name="output_topic" value="/example/output/topic" />
  </node>
</launch>
```
