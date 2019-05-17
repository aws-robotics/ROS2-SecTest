# ROS2 SecTest

ROS2 package that demonstrates ways to exploit vulnerabilities in ROS systems.

## Example Noop Attack
To run the demo:
Create a ROS2 Ws
```bash
mkdir -p security_ws/src
cd security_ws
```

clone the repository

```bash
cd src
git clone https://github.com/ryanewel/ros_sec_test.git
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
ros2 run ros_sec_test runner __params:=`ros2 pkg prefix ros_sec_test`/examples/params.yaml
```
You should see something like the following output which demonstrates that
the Noop node was configured and activated.
```
Starting runner
Nodes added to executor
Running
Client vector initialized
on_configure called
[INFO] [noop]: on_configure() is called.
[INFO] [attack_runner]: Transition 1 successfully triggered.
[INFO] [attack_runner]: Node ??;s has current state inactive.
Attacks configured
[INFO] [noop]: on_activate() is called.
[INFO] [attack_runner]: Transition 3 successfully triggered.
[INFO] [attack_runner]: Node ??;s has current state active.
```

To enable or disable attacks, you can create you own params.yaml.

## License

This library is licensed under the Apache 2.0 License. 
