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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.auto.NearSideAutoRed;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.state.OdometryTurret;
import org.firstinspires.ftc.teamcode.state.SingleLauncher;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.VelocityCalculation;

import java.util.function.Supplier;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "Red Teleop Near Start", group = "Examples")
public class RedTeleopNearStart extends OpMode {

    public static boolean robotCentric = true;
    private Follower follower;

    public DcMotor intake = null;

    public SingleLauncher launcher = null;


    Limelight3A limelight;
    private Alliance alliance = Alliance.RED;
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

    private boolean isDpadUP = false;
    private boolean isDpadDown = false;
    private boolean isXPressed = false;
    private boolean isYPressed = false;
    private boolean isBPressed = false;
    private boolean isRobotIndexing = false;
    private boolean isRightBumperPressed = false;
    private boolean isLeftBumperPressed = false;
    private boolean isAPressed = false;
    private boolean isGamepad1XPressed = false;
    private boolean isTurretEnabled = true;


    private double trigDistanceToGoal = 0;
    private double taDistanceToGoal = 0;//9

    private double distanceToGoal = 0;

    public static double SHOOTER_VELOCITY_FUDGE_FACTOR = 100;

    private long flyWheelStart = 0;
    private long elapsedTime = 0;

    private Supplier<PathChain> gotoShootPoseNear, gotoShootPoseFar;

    private boolean automatedDrive = false;

    private OdometryTurret turretStateMachine;
    private BallColor[] indexerLoad = {BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};// extra,intake,shoot

    NormalizedColorSensor colorSensorIntake;
    private Servo shooterLight;
    private int purpleHueMin = 165;
    private int purpleHueMax = 240;
    private int greenHueMin = 150;
    private int greenHueMax = 163;
    private float intakeHue;
    private int extraHue;
    private int shootHue;

    @Override
    public void init() {
        Pose start = new Pose(NearSideAutoRed.leaveX, NearSideAutoRed.leaveY, Math.toRadians(NearSideAutoRed.leaveHeading) ); // Assumed heading is 0 since we didn't specify
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
        turretStateMachine = new OdometryTurret(hardwareMap, telemetry, follower, alliance);

        intake = hardwareMap.get(DcMotor.class, "intake");
        colorSensorIntake = hardwareMap.get(NormalizedColorSensor.class, "color1");

        launcher = new SingleLauncher( hardwareMap, telemetry, null);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(2); // Switch to pipeline

        shooterLight = hardwareMap.get(Servo.class, "shooterlight");
    }

    public void start() {
        follower.startTeleopDrive();
        launcher.initializeSpindexer();
        launcher.deactivateFeeders();
        isShooterOn = true;
        launcher.startShooter();
        if (colorSensorIntake instanceof SwitchableLight) {
            ((SwitchableLight)colorSensorIntake).enableLight(true);
        }
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
            turretStateMachine.update(result);
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
            turretStateMachine.update( result );
            distanceToGoal = 0;
        }

        intakeHue = getColorSensorHue();
        if( purpleHueMin <= intakeHue && intakeHue <= purpleHueMax){
            indexerLoad[1] = BallColor.PURPLE;
        } else if (greenHueMin <= intakeHue && intakeHue <= greenHueMax) {
            indexerLoad[1] = BallColor.GREEN;
        }else {
            indexerLoad[1] = null;
        }

        if(automatedDrive && !follower.isBusy()){
            automatedDrive = false;
            follower.startTeleopDrive();
        }

        if( !automatedDrive ) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.right_stick_x,
                    -gamepad1.left_stick_x * 0.75,
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

        // Toggle turret enabled/disabled with gamepad1 X
        if (gamepad1.x && !isGamepad1XPressed) {
            isTurretEnabled = !isTurretEnabled;
            turretStateMachine.setEnabled(isTurretEnabled);
            isGamepad1XPressed = true;
        } else if (!gamepad1.x) {
            isGamepad1XPressed = false;
        }

        if ( gamepad2.x && !isXPressed) {
            launcher.shootExtra();
            isXPressed = true;
        } else if(!gamepad2.x){
            isXPressed = false;
        }
        /*Shooter sequence:
        Set Spindexer Position for shooting
        feeder up
        wait
        feeder down
        */

        if ( gamepad2.y && !isYPressed) {
            launcher.shootIntake();
            isYPressed = true;
        } else if(!gamepad2.y){
            isYPressed = false;
        }

        if ( gamepad2.b && !isBPressed) {
            launcher.shootShoot();
            isBPressed = true;
        } else if(!gamepad2.b){
            isBPressed = false;
        }

        if (gamepad2.right_bumper && !isRightBumperPressed) {
            isShooterOn = !isShooterOn;
            if( isShooterOn){
                launcher.startShooter();
            }
            shooterVelocity = VelocityCalculation.VELOCITY_DEFAULT;
            if( flyWheelStart == 0 ){
                elapsedTime = 0;
                flyWheelStart = System.currentTimeMillis();
            }
            isRightBumperPressed = true;
        } else if (!gamepad2.right_bumper) {
            isRightBumperPressed = false;
        }

        if(gamepad2.left_bumper && !isLeftBumperPressed){
            intake.setPower( Math.abs( intake.getPower() - 1.0) );
            isLeftBumperPressed = true;
        } else if (!gamepad2.left_bumper) {
            isLeftBumperPressed = false;
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
            if( distanceToGoal != 0 ) {
                shooterVelocity = VelocityCalculation.getTargetVelocity(distanceToGoal);
            }
            setShooterVelocity( shooterVelocity );
            shooterLight.setPosition(1);
        }else{
            setShooterVelocity( 0 );
            launcher.stopShooter();
            shooterLight.setPosition(0);
        }
/*
        if( Math.abs(shooterVelocity - shooter.getVelocity() ) < SHOOTER_VELOCITY_FUDGE_FACTOR && elapsedTime == 0 && flyWheelStart != 0){
            long now = System.currentTimeMillis();
            elapsedTime = now - flyWheelStart;
            flyWheelStart = 0;
        }*/

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
            launcher.turnSpindexerClockwise();
        } else if (gamepad2.dpad_left) {
            launcher.turnSpindexerCounterClockwise();
        }

        if(gamepad2.a && !isAPressed){
            launcher.switchSpindexerMode();
            isAPressed = true;
        }else if(!gamepad2.a){
            isAPressed = false;
        }

        telemetry.addData( "Shooter Velocity", shooterVelocity);
        telemetry.addData( "Motor Velocity", launcher.getShooterVelocity());
        telemetry.addData( "Distance To Goal", distanceToGoal);
        telemetry.addData("Driving Mode", robotCentric ? "Robot-Centric" : "Field-Centric");
        telemetry.addData("Turret Enabled (GP1 X)", isTurretEnabled);
        telemetry.addData("PP x", follower.getPose().getX());
        telemetry.addData("PP y", follower.getPose().getY());
        telemetry.addData("PP heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Shooter State", launcher.getState());
        if (elapsedTime > 0) {
            telemetry.addData( "Flywheel spin-up time (ms)", elapsedTime );
        }

    }

    private void setShooterVelocity(double p ){
        //rightShooter.setPower( -1 * p );
        //leftShooter.setPower(  p );
        launcher.setShooterVelocity( p );
    }
    private float getColorSensorHue(){
        NormalizedRGBA colors = colorSensorIntake.getNormalizedColors();
        float hue = JavaUtil.colorToHue(colors.toColor());
        logColors(colors);
        return hue;
        // Convert the RGB colors to HSV values
        //Color.RGBToHSV((int)(colors.red * SCALE_FACTOR), (int)(colors.green * SCALE_FACTOR), (int)(colors.blue * SCALE_FACTOR), hsvValues);

        //return hsvValues[0];
    }

    private void adjustColorSensingClockwise(){
        BallColor extra = indexerLoad[0];
        indexerLoad[0] = indexerLoad[2];
        indexerLoad[2] = indexerLoad[1];
        indexerLoad[1] = extra;
    }

    private void adjustColorSensingCounterClockwise(){
        BallColor extra = indexerLoad[0];
        indexerLoad[0] = indexerLoad[1];
        indexerLoad[1] = indexerLoad[2];
        indexerLoad[2] = extra;
    }

    private void removeShootColor(){
        indexerLoad[2] = null;
    }
/*
    public void setDrivePower(double frontLeft, double backLeft, double frontRight, double backRight) {
            leftFrontDrive.setPower(frontLeft);
            leftBackDrive.setPower(backLeft);
            rightFrontDrive.setPower(frontRight);
            rightBackDrive.setPower(backRight);
        }
    }*/

    private void logColors(NormalizedRGBA colors) {
        telemetry.addData("R", colors.red);
        telemetry.addData("G", colors.green);
        telemetry.addData("B", colors.blue);
        telemetry.addData("A", colors.alpha);
    }

    private void shootGreen (){
        if(indexerLoad[2] == BallColor.GREEN){
            launcher.shootShoot();
            removeShootColor();
        }else if(indexerLoad[1] == BallColor.GREEN){
            launcher.shootIntake();
            adjustColorSensingClockwise();
            removeShootColor();
        }else if(indexerLoad[0] == BallColor.GREEN){
            launcher.shootExtra();
            adjustColorSensingCounterClockwise();
            removeShootColor();
        }else{
            //otPurple();
        }
    }

    private void shootPurple(){
        if(indexerLoad[2] == BallColor.PURPLE){
            launcher.shootShoot();
            removeShootColor();
        }else if(indexerLoad[1] == BallColor.PURPLE){
            launcher.shootIntake();
            adjustColorSensingClockwise();
            removeShootColor();
        }else if(indexerLoad[0] == BallColor.PURPLE){
            launcher.shootExtra();
            adjustColorSensingCounterClockwise();
            removeShootColor();
        }else{
            //shootGreen();
        }
    }
}

