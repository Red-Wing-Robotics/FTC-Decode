# Limelight 3A Exercise and Documentation

The goal of today's meeting will be to accomplish the following:

1. Create a new TeleOp OpMode for today's exercise.
2. Get the Robot to read the obelisk and then output the order of the balls to the Driver Hub. For example, "purple, purple, green".
3. Get the Robot's position relative to the blue wall's April Tag using MegaTag 2 and output it to the Driver Hub. *This does not need to be correct, as will have to fully measure the camera's position on the robot before this is correct.*

## References

- [Limelight 3A FTC Programming](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)

### Connecting to the Limelight Web UI

Plug the Limelight into your computer. Once connected, visit http://limelight.local:5801 . This should bring up the web UI for configuration. All configuration will be done in this UI.

### Connecting the Limelight Camera to the Robot

1. After configuration, the camera should be connected via USB to the Control Hub. It should be plugged into a USB 3.0 port (blue port). 
2. Next, you will need to use the Driver Hub. CLick Configure Robot.
3. Click the scan button. You should see an "ethernet device" appear.
4. Edit the name to be `limelight`.

## Pipeline Configuration

We will configure two pipelines today. Both will be April Tag pipelines:

- `0` : name: `Obelisk`
- `1` : name: `Goals`

### Creating a Pipeline

The following steps will be configured for each of the two pipelines:

#### Input Page

1. Set the pipline number and name.
2. Set `Pipeline Type` to `AprilTags`.
3. Set `Source Image` to `Camera`.
4. Set `Resolution` to `960x720 40fps`. *This value may be changed so that we can try to get near 60fps.*
5. Set `Black Level Offset` to `0` and set the `Sensor Gain` to `15`.
6. Set `Flicker Correction` to `60hz`.
7. Set exposure to a value where the April Tag is clearly visible.

#### Configuration Page

1. Set `Family` to `AprilTag Classic 36h11`.
2. Ensure Marker Size is set to `101.6`.
3. Set `Detector Downscale` to `4`.
4. Set `Quality Threshold` to `2`. *We will have to watch this value and adjust if we are having false positives.*
5. For `ID Filters`, you will need to enter a comma separated list of the ID's you are looking for in that pipeline. You can get these from reviewing the April Tag printouts I have provided.

#### Advanced Page

1. Be sure `Full 3D Targeting` is set to `Yes`.

## Saving the Limelight Config

For each pipline, click the download button in the header. Then, save it into a folder named, `limelight` (that you will create) at the root of our Git repository.

## Additional Notes

### Working with the Pinpoint IMU

To use MegaTag 2, you will need to update the orientation of the robot on each loop. To do this, you can get the heading from the `follower` (which gets it from the Gobilda Pinpoint).

```java
limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
```