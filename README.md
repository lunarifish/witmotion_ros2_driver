# witmotion_ros2_driver

ROS2 port of [WitStandardProtocol_JY901](https://github.com/WITMOTION/WitStandardProtocol_JY901/), currently only very basic functionality is implemented.

## CI Status

<table>
<tr>
<td>foxy</td>
<td rowspan=2>
    <a href="https://github.com/lunarifish/witmotion_ros2_driver/actions/workflows/build_eol.yaml">
        <img src="https://github.com/lunarifish/witmotion_ros2_driver/actions/workflows/build_eol.yaml/badge.svg">
    </a>
</td>
</tr>
<tr>
<td>galactic</td>
</tr>
<tr>
<td>humble</td>
<td rowspan=4>
    <a href="https://github.com/lunarifish/witmotion_ros2_driver/actions/workflows/build.yaml">
        <img src="https://github.com/lunarifish/witmotion_ros2_driver/actions/workflows/build.yaml/badge.svg">
    </a>
</td>
</tr>
<tr>
<td>iron</td>
</tr>
<tr>
<td>jazzy</td>
</tr>
<tr>
<td>rolling</td>
</tr>
</table>

## Usage

just compile & run, make sure the sensor is connected and the serial port is accessible.

imu data will be available on topic `/imu`. feel free to put the node into a namespace.

```bash
ros2 run witmotion_ros2_driver witmotion_ros2_driver --ros-args -p serial_port:=/dev/ttyUSB0

# custom parent tf frame name
ros2 run witmotion_ros2_driver witmotion_ros2_driver --ros-args -p serial_port:=/dev/ttyUSB0 -p parent_frame:=base_link

# or you just don't want to broadcast tf
ros2 run witmotion_ros2_driver witmotion_ros2_driver --ros-args -p serial_port:=/dev/ttyUSB0 -p broadcast_tf:=false
```
