package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
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
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.VelocityCalculation;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "GBT Adjusting Shooter Power", group = "Examples")
public class GBTAdjustingShooterPower extends OpMode {

    public static boolean robotCentric = true;
    private Follower follower;

    public DcMotor intake = null;
    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter = null;
    public CRServo rightFeeder = null;
    public CRServo leftFeeder = null;
    public Servo diverter = null;
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

    @Override
    public void init() {
        Pose start = new Pose(17.0625/2d + 0.25,16.09375/2d, Math.toRadians(90) ); // Assumed heading is 0 since we didn't specify

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
        diverter = hardwareMap.get( Servo.class, "diverter");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(1); // Switch to pipeline

        diverter.setPosition( 0.02 );
    }

    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        telemetry.update();

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            LLResultTypes.FiducialResult fResult = result.getFiducialResults().get(0);
            int id = result.getFiducialResults().get(0).getFiducialId();
            telemetry.addData("April Tag ID", "" + id);

            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

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

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true
        );

        if (gamepad1.right_bumper) {
            rightFeeder.setPower(1);
        } else {
            rightFeeder.setPower(0);
        }

        if (gamepad1.left_bumper) {
            leftFeeder.setPower(-1);
        } else {
            leftFeeder.setPower(0);
        }

        if (gamepad1.x) {
            isShooterOn = true;
        } else if (gamepad1.y) {
            isShooterOn = false;
        }

        if(gamepad1.a){
            intake.setPower( -1.0 );
        } else if (gamepad1.b) {
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

        setShooterVelocity( shooterVelocity );

        if(gamepad1.dpad_right){
            diverter.setPosition( 0.34 );
        } else if (gamepad1.dpad_left ) {
            diverter.setPosition(0.02);
        }

        telemetry.addData( "Shooter Velocity", shooterVelocity);
        telemetry.addData( "Motor Velocity", rightShooter.getVelocity());
        telemetry.addData( "Distance To Goal", distanceToGoal);
        telemetry.addData("PP x", follower.getPose().getX());
        telemetry.addData("PP y", follower.getPose().getY());
        telemetry.addData("PP heading", follower.getPose().getHeading());

    }

    private void setShooterVelocity(double p ){
        //rightShooter.setPower( -1 * p );
        //leftShooter.setPower(  p );
        rightShooter.setVelocity(-1 * p);
        leftShooter.setVelocity(p);
    }
}

