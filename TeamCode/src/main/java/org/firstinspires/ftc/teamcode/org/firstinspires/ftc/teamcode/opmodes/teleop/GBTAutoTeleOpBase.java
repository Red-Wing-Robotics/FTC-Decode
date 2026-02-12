package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.state.OdometryTurret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.VelocityCalculation;
import org.firstinspires.ftc.teamcode.util.log.Logger;

import java.util.function.Supplier;

@SuppressWarnings("unused")
@Configurable
public abstract class GBTAutoTeleOpBase extends OpMode {

    private Logger logger;

    public static boolean robotCentric = true;
    private Follower follower;

    public DcMotor intake = null;
    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter = null;
    public CRServo rightFeeder = null;
    public CRServo leftFeeder = null;

    Limelight3A limelight;
    private boolean isShooterOn = false;

    public static double SHOOTER_VELOCITY_FUDGE_FACTOR = 100;

    private long flyWheelStart = 0;
    private long elapsedTime = 0;

    private Supplier<PathChain> gotoShootPoseNear, gotoShootPoseFar;

    private boolean automatedDrive = false;

    private OdometryTurret turretStateMachine;
    public static boolean enableVisionCorrection = true;

    // Abstract methods for subclasses to implement
    protected abstract Pose getStartingPose();
    protected abstract Pose getShootPoseNear();
    protected abstract Pose getShootPoseFar();
    protected abstract int getLimelightPipeline();
    protected abstract Alliance getAlliance();

    @Override
    public void init() {
        logger = new Logger(telemetry);

        Pose start = getStartingPose();
        Pose shootPoseNear = getShootPoseNear();
        Pose shootPoseFar = getShootPoseFar();

        gotoShootPoseNear = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, shootPoseNear)))
                .setLinearHeadingInterpolation(follower.getHeading(), shootPoseNear.getHeading())
                .build();

        gotoShootPoseFar = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, shootPoseFar)))
                .setLinearHeadingInterpolation(follower.getHeading(), shootPoseFar.getHeading())
                .build();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start);
        follower.update();
        turretStateMachine = new OdometryTurret(hardwareMap, telemetry, follower, getAlliance());
        turretStateMachine.setVisionEnabled(enableVisionCorrection);

        intake = hardwareMap.get(DcMotor.class, "intake");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(getLimelightPipeline());
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        // State Machine Updates
        follower.update();

        // Logging Updates
        logger.update();

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        double distanceToGoal;
        if (result != null && result.isValid()) {
            turretStateMachine.update(result);
            if (!result.getFiducialResults().isEmpty()) {
                int id = result.getFiducialResults().get(0).getFiducialId();
                logger.logData("April Tag ID", "" + id);
            }

            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            logger.logData("Tx", tx);

            double trigDistanceToGoal = DistanceCalculation.getTrigDistanceToTarget(ty);
            double taDistanceToGoal = DistanceCalculation.getAreaDistanceToTarget(ta);

            if (0.5 * (trigDistanceToGoal + taDistanceToGoal) > 74.4) {
                distanceToGoal = taDistanceToGoal;
            } else {
                distanceToGoal = trigDistanceToGoal;
            }
        } else {
            distanceToGoal = 0;
            turretStateMachine.update(result);
        }

        if (automatedDrive && (!follower.isBusy() || gamepad1.x)) {
            if(gamepad1.x) {
                follower.breakFollowing();
            }
            automatedDrive = false;
            follower.startTeleopDrive();
        }

        if (!automatedDrive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x, // Corrected from right_stick_x for strafing
                    -gamepad1.right_stick_x, // Corrected from left_stick_x for turning
                    robotCentric
            );

            if (gamepad1.a && !follower.isBusy()) {
                follower.followPath(gotoShootPoseNear.get());
                automatedDrive = true;
            }

            if (gamepad1.b && !follower.isBusy()) {
                follower.followPath(gotoShootPoseFar.get());
                automatedDrive = true;
            }
        }

        if (gamepad2.right_bumper) {
            rightFeeder.setPower(1);
        } else if (gamepad2.dpad_down) {
            rightFeeder.setPower(-1);
        } else {
            rightFeeder.setPower(0);
        }

        if (gamepad2.left_bumper) {
            leftFeeder.setPower(-1);
        } else if (gamepad2.dpad_down) {
            leftFeeder.setPower(1);
        }else {
            leftFeeder.setPower(0);
        }

        if (gamepad2.x) {
            isShooterOn = true;
            if (flyWheelStart == 0) {
                elapsedTime = 0;
                flyWheelStart = System.currentTimeMillis();
            }
        } else if (gamepad2.y) {
            isShooterOn = false;
        }

        if (gamepad2.a) {
            intake.setPower(-1.0);
        } else if (gamepad2.b) {
            intake.setPower(0);
        }else if (gamepad2.dpad_down) {
            intake.setPower(1);
        }

        double shooterVelocity;
        if (isShooterOn) {
            shooterVelocity = VelocityCalculation.getTargetVelocity(distanceToGoal);
        } else {
            shooterVelocity = 0;
        }

        if (Math.abs(shooterVelocity - rightShooter.getVelocity()) < SHOOTER_VELOCITY_FUDGE_FACTOR && elapsedTime == 0 && flyWheelStart != 0) {
            long now = System.currentTimeMillis();
            elapsedTime = now - flyWheelStart;
            flyWheelStart = 0;
        }

        if (gamepad2.dpad_down){
            shooterVelocity = -1500;
        }
        setShooterVelocity(shooterVelocity);

        logger.logData("Shooter Velocity", shooterVelocity);
        logger.logData("Right Motor Velocity", rightShooter.getVelocity());
        logger.logData("Left Motor Velocity", leftShooter.getVelocity());
        logger.logData("Distance To Goal", distanceToGoal);
        logger.logData("Driving Mode", robotCentric ? "Robot-Centric" : "Field-Centric");
        logger.logData("PP x", follower.getPose().getX());
        logger.logData("PP y", follower.getPose().getY());
        logger.logData("PP heading", Math.toDegrees(follower.getPose().getHeading()));
        if (elapsedTime > 0) {
            logger.logData("Flywheel spin-up time (ms)", elapsedTime);
        }
    }

    private void setShooterVelocity(double p) {
        rightShooter.setVelocity(-1 * p);
        leftShooter.setVelocity(p);
    }
}
