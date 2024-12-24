# witmotion_ros2_driver

ROS2 implementation of [WitStandardProtocol_JY901](https://github.com/WITMOTION/WitStandardProtocol_JY901/), currently only very basic functionalities are implemented.

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

1. Install boost library if you haven't already:

   ```bash
   sudo apt-get update
   sudo apt-get install libboost-all-dev
   ```

2. Make serial port accessible:

   ```bash
   sudo chmod 666 /dev/tty...  # replace ... with the actual port name
   ```

   > [!TIP]
   > Make use of udev to do the chmod automatically. Create a file `/etc/udev/rules.d/99-witmotion-serial.rules` with the following content:
   >
   > ```bash
   > SUBSYSTEM=="tty...", MODE="0666"
   > ```
   >
   > then, execute:
   >
   > ```bash
   > sudo udevadm control --reload-rules
   > sudo udevadm trigger
   > ```

3. And just compile & run:

   ```bash
   colcon build --packages-select witmotion_ros2_driver
   ros2 run witmotion_ros2_driver witmotion_ros2_driver --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=921600
   ```

There whill be a transform from `parent_frame` to `imu_link` published, if you didn't disable it by setting `broadcast_tf` to false.

`sensor_msgs/Imu` messages are published on the `imu` topic. feel free to put the node into a namespace.

### Node parameters

- `serial_port` (string, default: "/dev/ttyUSB0"): the serial port to connect to.

- `baud_rate` (int, default: 921600): the baud rate of the serial port.

- `parent_frame` (string, default: "base_link"): the parent frame of the imu data.

- `broadcast_tf` (bool, default: true): whether to broadcast transform from `parent_frame` to `imu_link`.

## Why reinventing the wheel?

There is already an ROS2 driver for this sensor/protocol: [ioio2995/witmotion_ros2](https://github.com/ioio2995/witmotion_ros2), but unfortunately on my [HWT606](https://wit-motion.yuque.com/wumwnr/docs/bgnf89) it doesn't work properly. So I decided to write my own one, based on [the official SDK](https://github.com/WITMOTION/WitStandardProtocol_JY901/tree/main/Linux_C).

If this version of the driver doesn't work for you, you might want to try the other one. good luck!
