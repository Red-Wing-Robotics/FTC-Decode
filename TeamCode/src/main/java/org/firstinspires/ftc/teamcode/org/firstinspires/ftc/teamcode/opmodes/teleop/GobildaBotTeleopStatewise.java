package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.log.ShootingState;


@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "Gobilda Bot Teleop Statewise", group = "Examples")
public class GobildaBotTeleopStatewise extends OpMode {

    public static boolean robotCentric = false;
    private Follower follower;

    public DcMotor intake = null;
    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter = null;
    public CRServo rightFeeder = null;
    public CRServo leftFeeder = null;
    public Servo diverter = null;
    private double shooterVelocity = 500;

    //public static double SHOOTER_DEFAULT_VELOCITY = 500;

    public static double MAX_SHOOTER_VELOCITY = (280 * 6);

    private boolean dpad_up = false;
    private boolean dpad_down = false;
    private int shootingSide = 0; // 0 is none, 1 is left, 2 is right

    private ShootingState sState = ShootingState.INACTIVE;

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

        diverter.setPosition( 0.02 );
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    public void loop() {

        follower.update();
        telemetry.update();

        //rightShooter.getV

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );

        switch (sState){
            case INACTIVE:
                setShooterVelocity( 0 );
                rightFeeder.setPower( 0 );
                leftFeeder.setPower( 0 );
            case PREPARING_TO_SHOOT:
                rightFeeder.setPower( 0 );
                leftFeeder.setPower( 0 );
                setShooterVelocity( shooterVelocity );
                if( Math.abs( rightShooter.getVelocity() - shooterVelocity ) < 50 && shootingSide != 0 ){
                    sState = ShootingState.SHOOTING;
                }
            case SHOOTING:
                if( shootingSide == 2 ){
                    rightFeeder.setPower(1);
                }else{
                    leftFeeder.setPower( -1 );
                }
                if( Math.abs( rightShooter.getVelocity() - shooterVelocity ) > 50 ){
                    shootingSide = 0;
                    sState = ShootingState.PREPARING_TO_SHOOT;
                }

        }

        if(gamepad1.a){
            intake.setPower( -1.0 );
        } else if (gamepad1.b) {
            intake.setPower( 0 );
        }

        if( sState != ShootingState.SHOOTING ) {
            if (gamepad1.dpad_up) {
                if (shooterVelocity < MAX_SHOOTER_VELOCITY && !dpad_up) {
                    shooterVelocity = shooterVelocity + 100;
                }
                dpad_up = true;
            } else if (gamepad1.dpad_down) {
                if (shooterVelocity > 0 && !dpad_down) {
                    shooterVelocity = shooterVelocity - 100;
                }
                dpad_down = true;
                dpad_up = false;
            } else {
                dpad_down = false;
                dpad_up = false;
            }
        }

        if(gamepad1.dpad_right){
            diverter.setPosition( 0.34 );
        } else if (gamepad1.dpad_left ) {
            diverter.setPosition(0.02);
        }

        if( gamepad1.right_bumper ){
            shootingSide = 2;
        }

        if ( gamepad1.left_bumper ){
            shootingSide = 1;
        }

        if( gamepad1.x ){
            sState = ShootingState.PREPARING_TO_SHOOT;
        }

        telemetry.addData( "Shooting State", sState );
        telemetry.addData( "Motor Velocity", rightShooter.getVelocity());
        telemetry.addData( "shooterVelocity", shooterVelocity);
    }

    private void setShooterVelocity( double p ){
        //rightShooter.setPower( -1 * p );
        //leftShooter.setPower(  p );
        rightShooter.setVelocity(-1 * p);
        leftShooter.setVelocity(p);
    }
}
