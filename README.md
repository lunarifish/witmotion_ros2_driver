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

## Why reinventing the wheel?

There is already an ROS2 driver for this sensor/protocol: [ioio2995/witmotion_ros2](https://github.com/ioio2995/witmotion_ros2), but unfortunately on my [HWT606](https://wit-motion.yuque.com/wumwnr/docs/bgnf89) it doesn't work properly. So I decided to write my own one, based on [the official SDK](https://github.com/WITMOTION/WitStandardProtocol_JY901/tree/main/Linux_C).

If this version of the driver doesn't work for you, you might want to try the other one. good luck!
