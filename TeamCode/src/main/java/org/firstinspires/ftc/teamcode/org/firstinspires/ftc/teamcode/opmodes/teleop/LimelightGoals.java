package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "Limelight Goals", group = "Examples")
public class LimelightGoals extends OpMode {

    Limelight3A limelight;
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));



    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(1); // Switch to pipeline number 1

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            LLResultTypes.FiducialResult fResult = result.getFiducialResults().get(0);
            int id = result.getFiducialResults().get(0).getFiducialId();
            telemetry.addData("April Tag ID", "" + id);

            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                telemetry.addData("MT2 Target:", "(" + tx + ", " + ty + ", " + ta + ")");
                double distance = getDistanceToTarget(ty);
                telemetry.addData("Distance to Goal", distance);
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.addData("PP x", follower.getPose().getX());
        telemetry.addData("PP y", follower.getPose().getY());
        telemetry.addData("PP heading", follower.getPose().getHeading());
    }

    private double getDistanceToTarget(double ty) {
        double targetHeight = 29.4375;
        double limelightHeight = 13.34375;
        double limelightAngle = 0;

        double angleToTarget = Math.toRadians(limelightAngle + ty);
        return (targetHeight - limelightHeight) / Math.tan(angleToTarget);
    }
}


