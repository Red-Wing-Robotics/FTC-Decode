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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.state.Launcher;
import org.firstinspires.ftc.teamcode.state.SingleLauncher;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.VelocityCalculation;

import java.util.function.Supplier;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "Blue Teleop", group = "Examples")
public class BlueTeleop extends OpMode {

    public static boolean robotCentric = true;
    private Follower follower;

    public DcMotor intake = null;
    public Servo turret = null;
    public SingleLauncher launcher = null;


    Limelight3A limelight;
    private Alliance alliance = Alliance.BLUE;
    private double shooterVelocity = 500;
    private boolean isShooterOn = false;

    public static double SHOOTER_DEFAULT_VELOCITY = 500;

    public static double SHOOTER_VELOCITY = 1300;
    public static double MAX_SHOOTER_VELOCITY = (280 * 6);
    public static double MAX_SHOOTER_POWER = 1d;

    public static double INTAKE_ONE = 0;
    public static double INTAKE_TWO = 0.33;
    public static double INTAKE_THREE = 0.67;
    public static double SHOOT_FORWARD = 0.8;
    public static double FEEDER_DOWN = 0;
    public static double FEEDER_UP = 0.3;

    private boolean dpad_up = false;
    private boolean dpad_down = false;
    private boolean xPressed = false;
    private boolean yPressed = false;
    private boolean bPressed = false;
    private boolean indexing = false;
    private boolean right_bumper = false;
    private boolean left_bumper = false;


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
        Pose start = new Pose(17.0625/2d + 0.25,16.09375/2d, Math.toRadians(90) ); // Assumed heading is 0 since we didn't specify
        Pose shootPoseNear = new Pose(72.1, 75.15, Math.toRadians(135));
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

        turret = hardwareMap.get( Servo.class, "turret");
        launcher = new SingleLauncher( hardwareMap, telemetry );

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(1); // Switch to pipeline

    }

    public void start() {
        follower.startTeleopDrive();
        launcher.initializeSpindexer();
        launcher.deactivateFeeders();
        isShooterOn = true;
        turret.setPosition(SHOOT_FORWARD);
    }
    //72.1x, 75.155y,134h
    @Override
    public void loop() {
        follower.update();
        telemetry.update();
        launcher.update();

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            LLResultTypes.FiducialResult fResult = result.getFiducialResults().get(0);
            int id = result.getFiducialResults().get(0).getFiducialId();
            telemetry.addData("April Tag ID", "" + id);

            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            telemetry.addData( "Tx", tx);

            trigDistanceToGoal = DistanceCalculation.getTrigDistanceToTargetBot2(ty);
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
                    -gamepad1.left_stick_x,
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

        if ( gamepad2.x && !xPressed ) {
            launcher.shootSecond();
            xPressed = true;
        } else if(!gamepad2.x){
            xPressed = false;
        }
        /*Shooter sequence:
        Set Spindexer Position for shooting
        feeder up
        wait
        feeder down
        */


        if ( gamepad2.y && !yPressed ) {
            launcher.shootThird();
            yPressed = true;
        } else if(!gamepad2.y){
            yPressed = false;
        }

        if ( gamepad2.b && !bPressed ) {
            launcher.shootFirst();
            bPressed = true;
        } else if(!gamepad2.b){
            bPressed = false;
        }

        if (gamepad2.right_bumper && !right_bumper) {
            isShooterOn = !isShooterOn;
            if( flyWheelStart == 0 ){
                elapsedTime = 0;
                flyWheelStart = System.currentTimeMillis();
            }
            right_bumper = true;
        } else if (!gamepad2.right_bumper) {
            right_bumper = false;
        }

        if(gamepad2.left_bumper && !left_bumper){
            intake.setPower( Math.abs( intake.getPower() - 1.0) );
            left_bumper = true;
        } else if (!gamepad2.left_bumper) {
            left_bumper = false;
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
            launcher.stopShooter();
        }
/*
        if( Math.abs(shooterVelocity - shooter.getVelocity() ) < SHOOTER_VELOCITY_FUDGE_FACTOR && elapsedTime == 0 && flyWheelStart != 0){
            long now = System.currentTimeMillis();
            elapsedTime = now - flyWheelStart;
            flyWheelStart = 0;
        }*/

        setShooterVelocity( shooterVelocity );

        /*
        if(gamepad2.dpad_right && !indexing){
            launcher.turnSpindexerCounterClockwise();
            indexing = true;
        } else if (gamepad2.dpad_left && !indexing) {
            launcher.turnSpindexerClockwise();
            indexing = true;
        }else if (!gamepad2.dpad_right && !gamepad2.dpad_left){
            indexing = false;
        }*/
        if(gamepad2.dpad_right){
            launcher.turnSpindexerCounterClockwise();
        } else if (gamepad2.dpad_left) {
            launcher.turnSpindexerClockwise();
        }

        telemetry.addData( "Shooter Velocity", shooterVelocity);
        telemetry.addData( "Motor Velocity", launcher.getShooterVelocity());
        telemetry.addData( "Distance To Goal", distanceToGoal);
        telemetry.addData("Driving Mode", robotCentric ? "Robot-Centric" : "Field-Centric");
        telemetry.addData("PP x", follower.getPose().getX());
        telemetry.addData("PP y", follower.getPose().getY());
        telemetry.addData("PP heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Spindexer position", launcher.getSpindexerPosition() );
        telemetry.addData("Shooter State", launcher.getState());
        if (elapsedTime > 0) {
            telemetry.addData( "Flywheel spin-up time (ms)", elapsedTime );
        }

    }

    private void setShooterVelocity(double p ){
        //rightShooter.setPower( -1 * p );
        //leftShooter.setPower(  p );
        launcher.startShooter( p );
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

