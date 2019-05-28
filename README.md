# ROS2 SecTest

**This code is experimental and uses features from ROS2 D**
ROS2 has minimal security or limitations on node permissions by default. This package is meant
to demonstrate ways a malicious or misconfigured node could harm the operations of a ROS2 system
and demonstrate mitigations for those "attacks".
The initial attacks come from the
[ROS2 Threat Model](http://design.ros2.org/articles/ros2_threat_model.html).

More details can be found in the design doc in this [PR](https://github.com/ros2/design/pull/235).



## Example Noop Attack
To run the demo:
Create a ROS2 workspace
```bash
mkdir -p security_ws/src
cd security_ws
```

Clone the repository

```bash
cd src
git clone https://github.com/aws-robotics/ROS2-SecTest
cd ..
```

Install dependencies and build
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/local_setup.sh
```

Run the noop demo
```bash
ros2 run ros_sec_test runner __params:=`ros2 pkg prefix ros_sec_test`/share/ros_sec_test/examples/params.yaml
```

You should see something like the following output which demonstrates that
the Noop node was configured and activated.
```
[INFO] [Runner]: Initializing Runner
[INFO] [Runner]: Adding attack node 'noop'
[INFO] [Runner]: Spinning started
[INFO] [Runner]: Configuring attack node 'noop'
[INFO] [noop]: on_configure() is called.
[INFO] [Runner]: Enabling attack node 'noop'
[INFO] [noop]: on_activate() is called.
[INFO] [Runner]: Shutting-down attack node 'noop'
[INFO] [noop]: on_deactivate() is called.
[INFO] [Runner]: Spinning finished
```

To enable or disable attacks, you can create you own params.yaml.

## License

This library is licensed under the Apache 2.0 License. 
