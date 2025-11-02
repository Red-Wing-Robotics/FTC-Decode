package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class GobildaBotTeleop extends OpMode {

    public static boolean robotCentric = false;
    private Follower follower;

    public DcMotor intake = null;
    public DcMotor leftShooter = null;
    public DcMotor rightShooter = null;
    public CRServo rightFeeder = null;
    public CRServo leftFeeder = null;
    private double shooterPower = 0;

    private boolean dpad_up = false;
    private boolean dpad_down = false;




    @Override
    public void init() {
        Pose start = new Pose(17.0625/2d + 0.25,16.09375/2d, Math.toRadians(90) ); // Assumed heading is 0 since we didn't specify

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(start); //startingPose == null ? new Pose() : startingPose);
        follower.update();

        intake = hardwareMap.get(DcMotor.class, "intake");
        leftShooter = hardwareMap.get(DcMotor.class, "leftShooter");
        rightShooter = hardwareMap.get(DcMotor.class, "rightShooter");

        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
    }

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

        if( gamepad1.right_bumper ){
            rightFeeder.setPower(1);
        } else {
            rightFeeder.setPower(0);
        }

        if( gamepad1.left_bumper ){
            leftFeeder.setPower(1);
        } else {
            leftFeeder.setPower(0);
        }

        if(gamepad1.x){
            setShooterPower( shooterPower );
        } else if (gamepad1.y) {
            setShooterPower( 0 );
        }

        if(gamepad1.a){
            intake.setPower( 1.0 );
        } else if (gamepad1.b) {
            intake.setPower( 0 );
        }

        if( gamepad1.dpad_up ){
            if (shooterPower < 1 && !dpad_up) {
                shooterPower = shooterPower + 0.1;
                setShooterPower(shooterPower);
            }
            dpad_up = true;
        } else if ( gamepad1.dpad_down ) {
            if( shooterPower > 0.1 && !dpad_down ) {
                shooterPower = shooterPower - 0.1;
                setShooterPower(shooterPower);

            }
            dpad_down = true;
            dpad_up = false;
        }else{
            dpad_down = false;
            dpad_up = false;
        }

        telemetry.addData( "Shooter Power", shooterPower);

    }

    private void setShooterPower( double p ){
        rightShooter.setPower( p );
        leftShooter.setPower( -1 * p );
    }
}
