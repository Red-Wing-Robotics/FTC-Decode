package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.state.OdometryTurret;
import org.firstinspires.ftc.teamcode.util.Alliance;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "Test Odometry Turret", group = "Testing")
public class TestTeleop extends OpMode {

    public static boolean robotCentric = false;
    public static Alliance alliance = Alliance.BLUE;
    public static int limelightPipeline = 1;
    public static boolean startWithVisionEnabled = false;

    private OdometryTurret turret;
    private Follower follower;
    private Limelight3A limelight;

    private boolean visionEnabled = false;
    private boolean turretEnabled = true;
    private boolean aPressed = false;
    private boolean xPressed = false;

    @Override
    public void init() {
        Pose start = new Pose(13.653, 9.403, Math.toRadians(90));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        follower.update();

        turret = new OdometryTurret(hardwareMap, telemetry, follower, alliance);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(limelightPipeline);

        visionEnabled = startWithVisionEnabled;
        turret.setVisionEnabled(visionEnabled);
        turret.setEnabled(turretEnabled);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_x,
                -gamepad1.left_stick_x * 0.75,
                robotCentric
        );
        follower.update();

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        // Toggle vision correction with gamepad1 A
        if (gamepad1.a && !aPressed) {
            visionEnabled = !visionEnabled;
            turret.setVisionEnabled(visionEnabled);
            aPressed = true;
        } else if (!gamepad1.a) {
            aPressed = false;
        }

        // Toggle turret enabled/disabled with gamepad1 X
        if (gamepad1.x && !xPressed) {
            turretEnabled = !turretEnabled;
            turret.setEnabled(turretEnabled);
            xPressed = true;
        } else if (!gamepad1.x) {
            xPressed = false;
        }

        turret.update(result);

        telemetry.addData("Turret Enabled (X toggle)", turretEnabled);
        telemetry.addData("Vision Enabled (A toggle)", visionEnabled);
        telemetry.addData("LL Has Target", result != null && result.isValid());
        if (result != null) {
            telemetry.addData("LL tx", result.getTx());
            telemetry.addData("LL ty", result.getTy());
            telemetry.addData("LL ta", result.getTa());
        }
        telemetry.addData("PP x", follower.getPose().getX());
        telemetry.addData("PP y", follower.getPose().getY());
        telemetry.addData("PP heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Drive Mode", robotCentric ? "Robot-Centric" : "Field-Centric");
        telemetry.update();
    }

}
