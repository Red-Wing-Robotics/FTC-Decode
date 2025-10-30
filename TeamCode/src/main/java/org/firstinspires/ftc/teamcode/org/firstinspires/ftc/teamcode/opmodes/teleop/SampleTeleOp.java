package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@SuppressWarnings("unused")
@Configurable
@TeleOp
public class SampleTeleOp extends OpMode {

    public static boolean robotCentric = false;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> gotoShootPose, gotoLeverPose;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;



    @Override
    public void init() {
        Pose start = new Pose(17.0625/2d + 0.25,16.09375/2d, Math.toRadians(90) ); // Assumed heading is 0 since we didn't specify
        Pose shootPose = new Pose(59.5, 83.69, Math.toRadians(135));
        Pose leverSetUpPose = new Pose(22.95, 71.9, 0);
        Pose leverPose = new Pose(15.95, 71.9, 0);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start); //startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        gotoShootPose = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, shootPose )))
                .setLinearHeadingInterpolation( follower.getHeading(), shootPose.getHeading())
                .build();

        gotoLeverPose = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, leverSetUpPose )))
                .setLinearHeadingInterpolation( follower.getHeading(), leverSetUpPose.getHeading())
                .addPath(new Path(new BezierLine(leverSetUpPose, leverPose )))
                .setLinearHeadingInterpolation( leverSetUpPose.getHeading(), leverPose.getHeading())
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        telemetry.update();

        if(automatedDrive && !follower.isBusy()){
            automatedDrive = false;
            follower.startTeleopDrive();
        }

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    robotCentric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    robotCentric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed() && !follower.isBusy()) {
            follower.followPath(gotoShootPose.get());
            automatedDrive = true;
        }

        if (gamepad1.bWasPressed() && !follower.isBusy()) {
            follower.followPath(gotoLeverPose.get());
            automatedDrive = true;

        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("automatedDrive", automatedDrive);
        telemetry.addData("slowMode", slowMode);

    }
}