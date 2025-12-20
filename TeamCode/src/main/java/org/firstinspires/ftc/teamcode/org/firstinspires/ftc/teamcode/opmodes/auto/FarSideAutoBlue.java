package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.ObeliskState;

import java.util.function.Supplier;

public class FarSideAutoBlue extends OpMode {

    Limelight3A limelight;

    private int pipeline;

    private Follower follower;
    private int pathState;

    private ObeliskState oState = ObeliskState.UNKNOWN;

    public DcMotor intake = null;
    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter = null;
    public CRServo rightFeeder = null;
    public CRServo leftFeeder = null;
    public Servo diverter = null;

    public double shootVelocity = 1500;
    public static double SHOOTER_VELOCITY_FUDGE_FACTOR = 100;

    // POSES -------------------------------------------------------------

    private double startX = 0;
    private double startY = 16.09375/2d;
    private double startHeading = 90;
    private double firstShootX = 67.02;
    private double firstShootY = 19.57;
    private double firstShootHeading = 116.7;
    private double loadingZoneCollectY = 0 ;
    private double loadingZoneCollectX = 0;
    private double loadingZoneCollectHeading = 0;
    private double gppX = 38.86;
    private double gppY = 36.68;
    private double gppHeading = 180;
    private double leaveY = 24;
    private double leaveX = 0;
    private double leaveHeading = 180;

    private final Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));

    private final Pose firstShootPose = new Pose(firstShootX, firstShootY, Math.toRadians(firstShootHeading));

    private final Pose loadingZoneCollectPose = new Pose(loadingZoneCollectX, loadingZoneCollectY, Math.toRadians(loadingZoneCollectHeading));

    private final Pose leavePose = new Pose(leaveX, leaveY, Math.toRadians(leaveHeading));


    private final Pose gppPose = new Pose(gppX, gppY, Math.toRadians(gppHeading));

    private final Pose pgpPose = new Pose(38.86, 60.16, Math.toRadians(180));

    private final Pose ppgPose = new Pose(38.86, 82.93, Math.toRadians(180));

    private final Pose gppCollectPose = new Pose(21.9, 36.68, Math.toRadians(180));

    private final Pose pgpCollectPose = new Pose(21.9, 60.16, Math.toRadians(180));

    private final Pose ppgCollectPose = new Pose(21.9, 82.93, Math.toRadians(180));

    private Pose preCollectPose;

    private Pose postCollectPose;

    private PathChain gotoFirstShootPose, gotoLoadingZoneCollectPose, gotoSecondShootPose, gotoLeavePose, gotoGPPCollect, gotoPGPCollect, gotoPPGCollect;

    //private Supplier<PathChain> extra;
    public void buildPaths() {
        gotoFirstShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstShootPose.getHeading())
                .build();

        gotoLoadingZoneCollectPose = follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, loadingZoneCollectPose))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), loadingZoneCollectPose.getHeading())
                .build();

        gotoSecondShootPose =  follower.pathBuilder()
                .addPath(new BezierLine(loadingZoneCollectPose, firstShootPose))
                .setLinearHeadingInterpolation(follower.getHeading(), firstShootPose.getHeading())
                .build();

        gotoLeavePose = follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, leavePose))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), leavePose.getHeading())
                .build();

        gotoGPPCollect = follower.pathBuilder()
                .addPath(new BezierLine(gppPose, gppCollectPose))
                .setLinearHeadingInterpolation(gppPose.getHeading(), gppCollectPose.getHeading())
                .build();
        gotoPGPCollect = follower.pathBuilder()
                .addPath(new BezierLine(pgpPose, pgpCollectPose))
                .setLinearHeadingInterpolation(pgpPose.getHeading(), pgpCollectPose.getHeading())
                .build();
        gotoPPGCollect = follower.pathBuilder()
                .addPath(new BezierLine(ppgPose, ppgCollectPose))
                .setLinearHeadingInterpolation(ppgPose.getHeading(), ppgCollectPose.getHeading())
                .build();
    }

    public void shootPreloadMotif( ObeliskState oState ){
        switch (oState){
            case PURPLE_GREEN_PURPLE:
                shootRight();
                intake.setPower(1.0);
                shootLeft();
                shootRight();
            case PURPLE_PURPLE_GREEN:
                shootRight();
                intake.setPower(1.0);
                shootRight();
                shootLeft();
            case GREEN_PURPLE_PURPLE:
                shootLeft();
                shootRight();
                intake.setPower(1.0);
                shootRight();
            default:
                shootRight();
                intake.setPower(1.0);
                shootRight();
                shootLeft();
        }
    }

    public PathChain getCollectionPath( ObeliskState oState ){
        switch (oState){
            case PURPLE_GREEN_PURPLE:
                return gotoPGPCollect;
            case PURPLE_PURPLE_GREEN:
                return gotoPPGCollect;
            case GREEN_PURPLE_PURPLE:
                return gotoGPPCollect;
            default:
                return gotoGPPCollect;
        }
    }
    public void setPathState(int pState) {
        pathState = pState;
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            //repeatable 99%
            case 0:
                setShooterVelocity( shootVelocity );
                follower.followPath(gotoFirstShootPose);
                setPathState(1);
                break;
            case 1:
                shootPreloadMotif( oState );
                setShooterVelocity( 0 );
                setPathState( 2 );
            case 2:
                if (!follower.isBusy()){
                    follower.followPath(gotoLoadingZoneCollectPose);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    setShooterVelocity( shootVelocity );
                    follower.followPath(gotoSecondShootPose);
                    setPathState(4);
                }
                break;
            case 4:
                shootPreloadMotif( oState );
                setShooterVelocity( 0 );
                setPathState( 5 );
            case 5:
                if(!follower.isBusy()) {
                    follower.followPath(gotoLeavePose);
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
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
        pipeline = 0;
        limelight.pipelineSwitch(pipeline); // Switch to pipeline number 1
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        if(pipeline == 0 && result != null && result.isValid() && oState == ObeliskState.UNKNOWN){
            int id = result.getFiducialResults().get(0).getFiducialId();
            oState = ObeliskState.fromInt( id );
            pipeline = 1;
        }

        if (pipeline == 1 && result != null && result.isValid()) {
            LLResultTypes.FiducialResult fResult = result.getFiducialResults().get(0);
            int id = result.getFiducialResults().get(0).getFiducialId();
            telemetry.addData("April Tag ID", "" + id);

            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                telemetry.addData("MT2 Target:", "(" + tx + ", " + ty + ", " + ta + ")");
                double distance = DistanceCalculation.getTrigDistanceToTarget(ty);
                telemetry.addData("Distance to Goal", distance);
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        } else {
            telemetry.addData("Limelight", "No Targets");
        }


        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    private void setShooterVelocity(double p ){
        //rightShooter.setPower( -1 * p );
        //leftShooter.setPower(  p );
        rightShooter.setVelocity(-1 * p);
        leftShooter.setVelocity(p);
    }

    private void shootRight(){
        if( Math.abs(rightShooter.getVelocity() - shootVelocity) < SHOOTER_VELOCITY_FUDGE_FACTOR){
            rightFeeder.setPower(1);
            //insert wait
            rightFeeder.setPower(0);
        }
    }

    private void shootLeft(){
        if( Math.abs(leftShooter.getVelocity() - shootVelocity) < SHOOTER_VELOCITY_FUDGE_FACTOR){
            leftFeeder.setPower(1);
            //insert wait
            leftFeeder.setPower(0);
        }
    }
}
