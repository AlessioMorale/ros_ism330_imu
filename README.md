# ros_ism330_imu

ROS 2 driver for the ST ISM330DHCX 6-axis IMU on Linux spidev.

| Package | Content |
| ------- | ------- |
| `ism330dhcx_driver` | ROS-independent register/FIFO driver, `SpiBus` (spidev, mode 3), fake bus for tests |
| `ism330dhcx_node` | Node publishing `sensor_msgs/Imu` on `imu` and `sensor_msgs/Temperature` on `imu/temperature`, plus diagnostics |

## Wiring (kvn_expio SPI0, see `ports.md`)

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

## Bring-up

```bash
ros2 launch ism330dhcx_node ism330dhcx.launch.py
ros2 topic hz /imu                 # ~208 Hz
ros2 topic echo /imu --once        # still and level: accel z ~ +9.81, gyro ~ 0
ros2 topic echo /diagnostics
```

If the log shows `unexpected WHO_AM_I 0x00` or `0xFF`, check the wiring and power.
