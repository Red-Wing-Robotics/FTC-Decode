package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * This is a sample autonomous OpMode meant to illustrate what a sample OpMode
 * should look like. This particular OpMode shows how to take the robot from the
 * center position to the red box, the blue box, and then back to the center.
 */
@SuppressWarnings("unused")
@Configurable
@Autonomous(name = "Sample Auto", group = "Examples")
public class SampleAutoOpMode extends OpMode {

    // Our telemetry manager for Panels
    private TelemetryManager telemetryM;

    // The Pedro Pathing follower that we will use to run our paths
    private Follower follower;

    // The class variable we will use to track where we are at in the auto routine
    private int pathState;

    // POSES -------------------------------------------------------------

    /*

        First, we will define each of the poses we will be using. If the heading
        is not specified, then it will assume 0 degrees. Heading are always specified
        in radians, so we need to use Math.toRadians for this.

     */

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));

    private final Pose box1Pose = new Pose(38.68, 33.25, Math.toRadians(0));

    private final Pose box2Pose = new Pose(106.09, 33.25, Math.toRadians(0));

    // PATH CHAINS -------------------------------------------------------

    /*

        Next, we can define the PathChain instances which represent a "path" from one
        pose to another pose. If you just want to travel in a straight line, you will
        use the BezierLine. You will also need to set the heading interpolation.

        Linear heading interpolation usually makes sense. It will rotate the robot from
        the starting pose's heading to the ending pose's heading along the path. This does
        require that you enter those values correctly when calling setLinearHeadingInterpolation.

        We make these PathChain instances available at the class level, as we'll need them
        later in the class.

     */

    private PathChain gotoBox1, gotoBox2, returnToOrigin;

    public void buildPaths() {
        gotoBox1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, box1Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), box1Pose.getHeading())
                .build();

        gotoBox2 = follower.pathBuilder()
                .addPath(new BezierLine(box1Pose, box2Pose))
                .setLinearHeadingInterpolation(box1Pose.getHeading(), box2Pose.getHeading())
                .build();

        returnToOrigin = follower.pathBuilder()
                .addPath(new BezierLine(box2Pose, startPose))
                .setLinearHeadingInterpolation(box2Pose.getHeading(), startPose.getHeading())
                .build();
    }

    // RUNNING THE PATHS -------------------------------------------------

    /*

        Next, we need methods to help us connect multiple paths into an autonomous routine.
        The pathState private class variable allows us to track where we are at in the
        overall process, and we use the setPathState variable to update this value. The
        value is set to 0 in the start method.

        Next, we have another method, autonomousPathUpdate, which runs on every execution
        of the loop.

     */

    public void setPathState(int pState) {
        pathState = pState;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(gotoBox1);
                setPathState(1);
                break;
            case 1:

                if(!follower.isBusy()) {
                    follower.followPath(gotoBox2);
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(returnToOrigin);
                    setPathState(-1);
                }
                break;
        }
    }

    // OPMODE METHODS (OVERRIDDEN) ---------------------------------------

    /**
     * This method executes on every execution cycle for the Control Hub after the
     * OpMode is started (when you hit the play button). It handles logging out the
     * telemetry to both the Driver Hub as well as to Panels (through the telemetryM
     * class variable).
     *
     * It's main work happens through calling follower.update() and then
     * autonomousPathUpdate(). This will run the autonomous OpMode. The follower
     * should be updated prior to updating the autonomous path.
     */
    @Override
    public void loop() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        // Feedback to Panels for monitoring and debugging
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.update();
    }

    /**
     * This method is called once at the init of the OpMode (when you press the
     * Init button). This is where we initialize variables that we will be using
     * during the OpMode.
     */
    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /**
     * This method is called continuously after Init while waiting for "play". We
     * don't often use this method, but it is included here for reference (since it
     * is a method we can override).
     */
    @Override
    public void init_loop() {}

    /**
     * This method is called once at the start of the OpMode. It runs all the setup
     * actions not handled in init(). Any intensive or time consuming work should
     * be done in init() and not start() since it could delay the robot running its
     * autonomous routine.
     */
    @Override
    public void start() {
        setPathState(0);
    }

    /**
     * This method is called after stopping an OpMode. We don't often use this
     * method, but it is included here for reference (since it is a method we can
     * override).
     */
    @Override
    public void stop() {}

}
