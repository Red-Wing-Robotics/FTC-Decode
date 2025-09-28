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

@SuppressWarnings("unused")
@Configurable
@Autonomous(name = "Example Auto", group = "Examples")
public class SampleAutoOpMode extends OpMode {

    private TelemetryManager telemetryM;
    private Follower follower;
    private int pathState;

    // POSES -------------------------------------------------------------

    private final Pose startPose = new Pose(72, 72, Math.toRadians(0));

    private final Pose box1Pose = new Pose(38.68, 33.25, Math.toRadians(0));

    private final Pose box2Pose = new Pose(106.09, 33.25, Math.toRadians(0));

    // PATH CHAINS -------------------------------------------------------

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

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
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

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}
