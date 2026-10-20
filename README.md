![Slamming Works brand lockup](assets/slw-lockup.svg)

# ISM330DHCX IMU driver

Slamming Works' ROS 2 driver for the ST ISM330DHCX 6-axis IMU, using Linux spidev.

## Demo

There is no IMU demo video or GIF in this repository yet.

## Hardware

The driver connects to an ISM330DHCX over SPI. The wiring table below documents the KVN rover's SPI0 connection. A consolidated BOM and schematic are not included here.

## Build and run

From the ROS 2 workspace root:

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to ism330dhcx_node
source install/setup.bash
```

Connect the sensor and confirm the SPI device in `config/ism330dhcx.yaml`, then launch:

```bash
ros2 launch ism330dhcx_node ism330dhcx.launch.py
ros2 topic echo /imu --once
ros2 topic echo /diagnostics
```

Run the package tests with:

```bash
colcon test --packages-select ism330dhcx_driver ism330dhcx_node
colcon test-result --verbose
```

| Package | Content |
| ------- | ------- |
| `ism330dhcx_driver` | ROS-independent register/FIFO driver, `SpiBus` (spidev, mode 3), fake bus for tests |
| `ism330dhcx_node` | Node publishing `sensor_msgs/Imu`, `sensor_msgs/Temperature` and diagnostics |

### Wiring (KVN ExPIO SPI0)

| Pi pin | GPIO | Signal | ISM330DHCX |
| ------ | ---- | ------ | ---------- |
| 19 | GPIO10 | MOSI | SDA / SDI |
| 21 | GPIO9 | MISO | SDO / SA0 |
| 23 | GPIO11 | SCLK | SCL / SPC |
| 24 | GPIO8 | CE0 | CS |
| 1/17 | - | 3V3 | VDD, VDDIO |
| GND | - | GND | GND |

INT1/INT2 are not used: the node polls the FIFO.

## How it works

- Accelerometer and gyroscope run at the same ODR and are batched into the FIFO (continuous
  mode). The node drains it every `poll_period_ms` and publishes every sample.
- Sample stamps are spaced by the factory-trimmed ODR (`INTERNAL_FREQ_FINE`) and anchored to
  the read time.
- At startup the gyro bias is averaged over `calibration.duration_s` while the robot is still;
  if the robot moves, it retries, then falls back to the `gyro_bias` parameter. Nothing is
  published while calibrating. The bias is kept across reconnects.
- No orientation is estimated (`orientation_covariance[0] = -1`).
- If the device is missing or the bus fails, the node reports ERROR on `/diagnostics` and
  retries every `reconnect_period_ms`.

Data is in the sensor's axes, stamped with `frame_id` (`imu_link`). Put the mounting
rotation in `kvn_robot_description/config/robot_config.yaml` (`imu_*`).

The sample configuration sets the requested output data rate to 208 Hz. Actual publication depends on the sensor, SPI connection and polling configuration. If the log shows `unexpected WHO_AM_I 0x00` or `0xFF`, check the wiring and power.

## Status

**Partly works.** The driver, ROS node, configuration and tests are present. Orientation is not estimated, and this repository does not include an IMU test recording or results log.

## License

MIT. See [LICENSE](LICENSE).
