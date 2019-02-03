# Actor eStop Node

Forwards a `geometry_msgs/Twist`, or publishes all zeros at 50 hz

- Stopped and resumed via `std_srvs` services
  - `std_srvs/Empty` for `~stop` service
  - `std_srvs/Trigger` for `~resume` service
- Starts in stopped state
- Only one node has "permission" to reset estop
  - The node that resumes first gets permission
- Specify topic to forward in rosparam `~forwarded_topic`
- publishes output to `~out`

### Example ROS launch file

```xml
<launch>
  <node pkg="ltu_actor_estop" type="estop" name="estop">
    <param name="forwarded_topic" value="/example/topic" />
  </node>
</launch>
```
