package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.RWRBaseOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.ObeliskState;
//import thread.sleep;

import java.util.function.Supplier;

@Configurable
@Autonomous(name = "Far Side Auto Blue", group = "Examples")
public class FarSideAutoBlue extends RWRBaseOpMode {

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

    public static double shootVelocity = 1580;
    public static double SHOOTER_VELOCITY_FUDGE_FACTOR = 100;

    public static long TIMEOUT_DEFAULT = 5000;

    // POSES -------------------------------------------------------------

    public static double startX = 57.2;
    public static double startY = 16.09375/2d;
    public static double startHeading = 90;
    public static double firstShootX = 57.2;
    public static double firstShootY = 10;
    public static double firstShootHeading = 108;
    public static double loadingZoneCollectY = 11.3;
    public static double loadingZoneCollectX = 13.1;
    public static double loadingZoneCollectHeading = 180;
    public static double gppX = 38.86;
    public static double gppY = 36.68;
    public static double gppHeading = 180;
    public static double leaveY = 24;
    public static double leaveX = 0;
    public static double leaveHeading = 180;

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
                shootRight( 100000 );
                intake.setPower(-1.0);
                shootLeft();
                shootRight();
            case PURPLE_PURPLE_GREEN:
                shootRight( 100000 );
                intake.setPower(-1.0);
                shootRight();
                shootLeft();
            case GREEN_PURPLE_PURPLE:
                shootLeft(100000);
                shootRight();
                intake.setPower(-1.0);
                shootRight();
            default:
                shootRight(100000);
                intake.setPower(-1.0);
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
                setShooterVelocity(shootVelocity);
                follower.followPath(gotoFirstShootPose, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    //shootPreloadMotif(oState);
                    setShooterVelocity(0);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()){
                    follower.followPath(gotoLoadingZoneCollectPose, true);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    setShooterVelocity( shootVelocity );
                    follower.followPath(gotoSecondShootPose,true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    shootPreloadMotif(oState);
                    setShooterVelocity(0);
                    setPathState(5);
                }
                break;
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

    public void start() {
        pathState = 0;
        diverter.setPosition( 0.02 );
    }

    @Override
    public void loop() {

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
            log("April Tag ID", "" + id);

            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                log("MT2 Target:", "(" + tx + ", " + ty + ", " + ta + ")");
                double distance = DistanceCalculation.getTrigDistanceToTarget(ty);
                log("Distance to Goal", distance);
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                log("MT2 Location:", "(" + x + ", " + y + ")");
            }
        } else {
            log("Limelight", "No Targets");
        }

        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        log("path state", pathState);
        log("x", follower.getPose().getX());
        log("y", follower.getPose().getY());
        log("heading", follower.getPose().getHeading());
        log("right shooter velocity", rightShooter.getVelocity());
        log("left shooter velocity", leftShooter.getVelocity());
        updateTelemetry();
    }

    private void setShooterVelocity(double p ){
        //rightShooter.setPower( -1 * p );
        //leftShooter.setPower(  p );
        rightShooter.setVelocity(-1 * p);
        leftShooter.setVelocity(p);
    }

    private void shoot( long timeout, boolean isRight ){
        log("State", "Start Shoot");
        updateTelemetry();
        CRServo feeder = (isRight) ? rightFeeder : leftFeeder;

        long startTime = System.currentTimeMillis();
        while ( !(Math.abs(rightShooter.getVelocity() - shootVelocity) < SHOOTER_VELOCITY_FUDGE_FACTOR)){
            if( System.currentTimeMillis() - startTime >= timeout ) {
                return;
            }
        }

        log("State", "Speed Reached");
        updateTelemetry();
        feeder.setPower(1);
        while ( (Math.abs(rightShooter.getVelocity() - shootVelocity) < SHOOTER_VELOCITY_FUDGE_FACTOR)){
            if( System.currentTimeMillis() - startTime >= timeout ) {
                return;
            }
        }

        log("State", "Shooting Completed");
        updateTelemetry();

        feeder.setPower(0);
    }

    private void shootRight( long timeout ){
        shoot( timeout, true);
    }

    private void shootRight(){
        shootRight( TIMEOUT_DEFAULT );
    }

    private void shootLeft( long timeout ){
        shoot( timeout, false);
    }

    private void shootLeft(){
        shootRight( TIMEOUT_DEFAULT );
    }
}
