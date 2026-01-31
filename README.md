# Unity ROSbag Playback System

A Unity project for replaying underwater robot (BlueROV2) recordings. Play back your rosbag files and see what the robot saw and did.

## What You Need

- Unity (tested with 2021.3+)
- ROS 2 Humble
- Unity Robotics Hub package
- A rosbag file with BlueROV2 data

## Quick Start

### 1. Start the ROS Bridge

Open a terminal and run:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint
```

This connects Unity to ROS. Keep it running.

### 2. Set Up Unity

1. Open the project in Unity
2. Find the **RosbagController** in your scene
3. In the Inspector, set the **Rosbag Path** to your bag's folder
   - Example: `/home/usuario/Downloads/ui_localization_bags/`

### 3. Hit Play

Press Play in Unity. Use the UI controls to start playback.

## The Scripts

### Rosbag Control

**RosbagController.cs** - The main controller
- Starts/stops bag playback
- Handles looping and speed control
- Lives on an empty GameObject in your scene

**RosbagUIController.cs** - The buttons and sliders
- **Play Button** - Starts the rosbag
- **Stop Button** - Stops playback
- **Restart Button** - Stop then start (useful when you change settings)
- **Loop Toggle** - Check this to replay forever
- **Speed Slider** - Goes from 0.1x to 3.0x speed (1.0x is normal)
- **Status Text** - Shows if it's playing, the speed, and any errors

The speed and loop settings only apply when you restart. So change them, then hit restart.

### Robot Movement

**BlueRovCmdVeloRosSub.cs** - Makes the robot move based on velocity commands
- Subscribes to `/bluerov2/cmd_vel`
- Good for teleop recordings

**BlueRovVeloControl.cs** - Actually moves the robot model
- Handles ROS→Unity coordinate conversion
- Attached to your BlueROV2 model

**BlueRovOdomMirrorRosSub.cs** - Uses odometry for position
- Subscribes to `/bluerov2/global_position/local`
- More accurate than dead reckoning

**BlueRovTFPoseController.cs** - Direct pose from TF transforms
- The most accurate way to position the robot
- Disable the velocity subscriber if using this
- Set `robotFrameId` to your robot's frame (usually `base_link`)
- Set `referenceFrameId` to `odom` or `map`

**BlueRovTFPoseController_Configurable.cs** - For troubleshooting
- Try different rotation modes if the robot looks wrong
- Press 'R' during playback to cycle modes
- Has a debug overlay showing position/rotation

### TF Frames

**TFSubscriber.cs** - Shows all coordinate frames
- Creates RGB axes for each frame (like RViz)
- Red=X, Green=Y, Blue=Z

**TFFrameDetector.cs** - Finds available frames
- Run this first if you don't know your frame names
- Auto-detects for 5 seconds
- Prints results to console

### Camera

**CameraImageSubscriber.cs** - Shows the camera feed
- Needs a RawImage UI element
- Handles different image encodings (RGB, BGR, mono, etc.)

**CameraRosPublisher.cs** - Publishes Unity camera to ROS
- For recording new bags
- Set publish rate (Hz)

**CameraCapturer.cs** - Grabs frames from Unity camera
- Helper for the publisher

### ArUco Markers

**ArucoPoseArraySubscriber.cs** - Shows detected markers
- Subscribes to `/bluerov2/aruco_pool/poses`
- Creates yellow cubes at marker positions
- Good for seeing what the robot detected

## Common Issues

**"Directory not found"**
- Check your rosbag path is correct
- Make sure it ends with a `/`

**Robot not moving**
- Check the ROS bridge is running
- Verify topic names match your bag
- Use `ros2 topic list` while bag is playing

**Robot facing wrong way**
- Try the Configurable controller
- Cycle through rotation modes with 'R'
- Check your frame IDs

**Can't see camera feed**
- Assign a RawImage in the subscriber
- Check topic name matches
- Some encodings might not work

## Tips

- Start with TFFrameDetector to find your frame names
- Use TFPoseController for the most accurate playback
- Speed slider is useful for slow-motion analysis
- Loop mode is great for testing

## Coordinate Systems

ROS uses: X=forward, Y=left, Z=down (underwater)  
Unity uses: X=right, Y=up, Z=forward

The scripts handle conversion. If something looks flipped, that's probably why.
