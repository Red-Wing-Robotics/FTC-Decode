package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.auto.FarSideAutoBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.state.Diverter;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.VelocityCalculation;
import org.firstinspires.ftc.teamcode.util.log.Logger;

import java.util.function.Supplier;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "GBT Blue Far Auto", group = "Examples")
public class GBTBlueFarAuto extends OpMode {

    private Logger logger;

    public static boolean robotCentric = true;
    private Follower follower;

    public DcMotor intake = null;
    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter = null;
    public CRServo rightFeeder = null;
    public CRServo leftFeeder = null;

    private Diverter diverterStateMachine;

    Limelight3A limelight;
    private Alliance alliance = Alliance.BLUE;
    private double shooterVelocity = 500;
    private boolean isShooterOn = false;

    public static double SHOOTER_DEFAULT_VELOCITY = 500;

    public static double MAX_SHOOTER_VELOCITY = (280 * 6);
    public static double MAX_SHOOTER_POWER = 1d;

    private boolean dpad_up = false;
    private boolean dpad_down = false;

    private double trigDistanceToGoal = 0;
    private double taDistanceToGoal = 0;//9

    private double distanceToGoal = 0;

    public static double SHOOTER_VELOCITY_FUDGE_FACTOR = 100;

    private long flyWheelStart = 0;
    private long elapsedTime = 0;

    private Supplier<PathChain> gotoShootPoseNear, gotoShootPoseFar;

    private boolean automatedDrive = false;

    @Override
    public void init() {
        logger = new Logger(telemetry);

        Pose start = new Pose( FarSideAutoBlue.leaveX, FarSideAutoBlue.leaveY, Math.toRadians(FarSideAutoBlue.leaveHeading) ); // Assumed heading is 0 since we didn't specify
        Pose shootPoseNear = new Pose(144 - 81.1, 92.15, Math.toRadians(135));
        Pose shootPoseFar = new Pose(67.02, 19.57, 2.037);//2.037
        //Pose leverSetUpPose = new Pose(22.95, 71.9, 0);
        //Pose leverPose = new Pose(15.95, 71.9, 0)

        gotoShootPoseNear = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, shootPoseNear )))
                .setLinearHeadingInterpolation( follower.getHeading(), shootPoseNear.getHeading())
                .build();

        gotoShootPoseFar = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, shootPoseFar )))
                .setLinearHeadingInterpolation( follower.getHeading(), shootPoseFar.getHeading())
                .build();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start); //startingPose == null ? new Pose() : startingPose);
        follower.update();

        intake = hardwareMap.get(DcMotor.class, "intake");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(1); // Switch to pipeline

        diverterStateMachine = new Diverter(hardwareMap, telemetry);
    }

    public void start() {
        follower.startTeleopDrive();
    }
    //72.1x, 75.155y,134h
    @Override
    public void loop() {
        // State Machine Updates
        follower.update();
        diverterStateMachine.update();

        // Logging Updates
        logger.update();

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            LLResultTypes.FiducialResult fResult = result.getFiducialResults().get(0);
            int id = result.getFiducialResults().get(0).getFiducialId();
            logger.logData("April Tag ID", "" + id);

            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            logger.logData( "Tx", tx);

            trigDistanceToGoal = DistanceCalculation.getTrigDistanceToTarget(ty);
            taDistanceToGoal = DistanceCalculation.getAreaDistanceToTarget(ta);

            if (0.5 * (trigDistanceToGoal + taDistanceToGoal) > 74.4) {
                distanceToGoal = taDistanceToGoal;
            } else {
                distanceToGoal = trigDistanceToGoal;
            }
        } else {
            distanceToGoal = 0;
        }

        if(automatedDrive && !follower.isBusy()){
            automatedDrive = false;
            follower.startTeleopDrive();
        }

        if( !automatedDrive ) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    -(gamepad1.left_stick_x * 0.5),
                    robotCentric
            );
        }

        if (gamepad1.aWasPressed() && !follower.isBusy()) {
            follower.followPath(gotoShootPoseNear.get());
            automatedDrive = true;
        }

        if (gamepad1.bWasPressed() && !follower.isBusy()) {
            follower.followPath(gotoShootPoseFar.get());
            automatedDrive = true;

        }

        if (gamepad2.right_bumper  ) {//&& Math.abs(rightShooter.getVelocity() - shooterVelocity) < SHOOTER_VELOCITY_FUDGE_FACTOR
            rightFeeder.setPower(1);
        } else {
            rightFeeder.setPower(0);
        }

        if (gamepad2.left_bumper && Math.abs(leftShooter.getVelocity() - shooterVelocity) < SHOOTER_VELOCITY_FUDGE_FACTOR ) {
            leftFeeder.setPower(-1);
        } else {
            leftFeeder.setPower(0);
        }

        if (gamepad2.x) {
            isShooterOn = true;
            if( flyWheelStart == 0 ){
                elapsedTime = 0;
                flyWheelStart = System.currentTimeMillis();
            }
        } else if (gamepad2.y) {
            isShooterOn = false;
        }

        if(gamepad2.a){
            intake.setPower( -1.0 );
        } else if (gamepad2.b) {
            intake.setPower( 0 );
        }

        /*if( gamepad1.dpad_up ){
            if (shooterVelocity < MAX_SHOOTER_VELOCITY && !dpad_up) {
                //ShooterVelocity = ShooterVelocity + 100;
                shooterVelocity = shooterVelocity + 10;
                setShooterVelocity(shooterVelocity);
            }
            dpad_up = true;
        } else if ( gamepad1.dpad_down ) {
            if( shooterVelocity > 0 && !dpad_down ) {
                //ShooterVelocity = ShooterVelocity - 100;
                shooterVelocity = shooterVelocity - 10;
                setShooterVelocity(shooterVelocity);

            }
            dpad_down = true;
            dpad_up = false;
        }else{
            dpad_down = false;
            dpad_up = false;
        }*/

        if( isShooterOn ){
            shooterVelocity = VelocityCalculation.getTargetVelocity( distanceToGoal );

        }else{
            shooterVelocity = 0;
        }

        if( Math.abs(shooterVelocity - rightShooter.getVelocity() ) < SHOOTER_VELOCITY_FUDGE_FACTOR && elapsedTime == 0 && flyWheelStart != 0){
            long now = System.currentTimeMillis();
            elapsedTime = now - flyWheelStart;
            flyWheelStart = 0;
        }

        setShooterVelocity( shooterVelocity );

        if(gamepad2.dpad_right || gamepad2.dpad_left){
            diverterStateMachine.toggleDiverter();
        }

        logger.logData( "Shooter Velocity", shooterVelocity);
        logger.logData( "Right Motor Velocity", rightShooter.getVelocity());
        logger.logData("Left Motor Velocity", leftShooter.getVelocity());
        logger.logData( "Distance To Goal", distanceToGoal);
        logger.logData("Driving Mode", robotCentric ? "Robot-Centric" : "Field-Centric");
        logger.logData("PP x", follower.getPose().getX());
        logger.logData("PP y", follower.getPose().getY());
        logger.logData("PP heading", Math.toDegrees(follower.getPose().getHeading()));
        if (elapsedTime > 0) {
            logger.logData( "Flywheel spin-up time (ms)", elapsedTime );
        }

    }

    private void setShooterVelocity(double p ){
        //rightShooter.setPower( -1 * p );
        //leftShooter.setPower(  p );
        rightShooter.setVelocity(-1 * p);
        leftShooter.setVelocity(p);
    }
/*
    public void setDrivePower(double frontLeft, double backLeft, double frontRight, double backRight) {
            leftFrontDrive.setPower(frontLeft);
            leftBackDrive.setPower(backLeft);
            rightFrontDrive.setPower(frontRight);
            rightBackDrive.setPower(backRight);
        }
    }*/
}

