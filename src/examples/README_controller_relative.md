# Controller Relative to Tracker Example

This example demonstrates how to compute and display the position of a controller relative to a tracker in real-time.

## What it does

The program:
1. Initializes the VR system
2. Detects `controller_1` and `tracker_1`
3. Continuously computes the position of the controller in the tracker's coordinate frame
4. Displays the relative position (X, Y, Z) and distance in meters

## Building

The example is built automatically with the rest of the project:

```bash
cd /root/htc_vive_tracker/build
make
```

The executable will be created at: `/root/htc_vive_tracker/bin/controller_relative_to_tracker`

## Running

```bash
cd /root/htc_vive_tracker/bin
./controller_relative_to_tracker
```

### Command-line options:
- `-v` : Verbose mode (shows detailed initialization info)
- `-h` : Display help message

## Example Output

```
VR System initialized successfully
Detected devices:
  hmd_1
  controller_1
  tracker_1
  tracking_reference_1
  tracking_reference_2

Both controller_1 and tracker_1 detected!

Press Ctrl+C to exit

Controller position relative to Tracker: X=0.234m, Y=-0.156m, Z=0.512m | Distance=0.582m
```

## Understanding the Output

The position is expressed in the tracker's coordinate frame:
- **X**: Left/Right relative to tracker (positive = right)
- **Y**: Up/Down relative to tracker (positive = up)
- **Z**: Forward/Backward relative to tracker (positive = forward)
- **Distance**: Euclidean distance between controller and tracker

## Technical Details

The program:
1. Gets the absolute poses (position + quaternion) of both devices
2. Converts them to 4x4 transformation matrices
3. Computes the relative transformation: T_tracker_controller = T_tracker_world * T_world_controller
4. Extracts the position component from the relative transformation

The transformation uses proper homogeneous coordinates and quaternion-to-matrix conversion.
