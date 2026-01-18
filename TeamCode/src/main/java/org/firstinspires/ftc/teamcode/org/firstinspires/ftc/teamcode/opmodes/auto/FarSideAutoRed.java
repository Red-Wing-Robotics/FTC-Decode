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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.RWRBaseOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.state.SingleLauncher;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.ObeliskState;

@SuppressWarnings("unused")
@Configurable
@Autonomous(name = "Far Side Auto Red", group = "Examples")
public class FarSideAutoRed extends RWRBaseOpMode {

    Limelight3A limelight;

    private int pipeline;

    private Follower follower;

    private SingleLauncher launcher;

    private int pathState;

    private ObeliskState oState = ObeliskState.UNKNOWN;

    public DcMotor intake = null;
    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter = null;
    public CRServo rightFeeder = null;
    public CRServo leftFeeder = null;
    public Servo diverter = null;

    public static double shootVelocity = 1380;
    public static double SHOOTER_VELOCITY_FUDGE_FACTOR = 100;

    public static long TIMEOUT_DEFAULT = 5000;

    // POSES -------------------------------------------------------------

    public static double startX = 144 - FarSideAutoBlue.startX;
    public static double startY = 16.09375/2d;
    public static double startHeading = 180 - FarSideAutoBlue.startHeading;
    public static double firstShootX = 144 - FarSideAutoBlue.firstShootX;
    public static double firstShootY = 10;
    public static double firstShootHeading = 180 - FarSideAutoBlue.firstShootHeading;
    public static double secondLoadingZoneCollectY = 15;
    public static double secondLoadingZoneCollectX = 144 - FarSideAutoBlue.secondLoadingZoneCollectX;
    public static double secondLoadingZoneCollectHeading = 180 - FarSideAutoBlue.secondLoadingZoneCollectHeading;
    public static double firstLoadingZoneCollectY = 14.3;
    public static double firstLoadingZoneCollectX = 144 - FarSideAutoBlue.firstLoadingZoneCollectX;
    public static double firstLoadingZoneCollectHeading = 180 - FarSideAutoBlue.firstLoadingZoneCollectHeading;
    public static double greenCollectX = 144 - FarSideAutoBlue.greenCollectX;
    public static double greenCollectY = 37;
    public static double greenCollectHeading = 180 - FarSideAutoBlue.greenCollectHeading;
    public static double purpleCollectX = 144 - FarSideAutoBlue.purpleCollectX;
    public static double purpleCollectY = 37;
    public static double purpleCollectHeading = 180 - FarSideAutoBlue.purpleCollectHeading;
    public static double leaveY = 24;
    public static double leaveX = 144 - FarSideAutoBlue.leaveX;
    public static double leaveHeading = 180 - FarSideAutoBlue.leaveHeading;

    private final Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));
    private final Pose firstShootPose = new Pose(firstShootX, firstShootY, Math.toRadians(firstShootHeading));
    private final Pose firstLoadingZoneCollectPose = new Pose(firstLoadingZoneCollectX, firstLoadingZoneCollectY, Math.toRadians(firstLoadingZoneCollectHeading));
    private final Pose secondLoadingZoneCollectPose = new Pose(secondLoadingZoneCollectX, secondLoadingZoneCollectY, Math.toRadians(secondLoadingZoneCollectHeading));
    private final Pose greenCollectPose = new Pose( greenCollectX, greenCollectY, Math.toRadians(greenCollectHeading));
    private final Pose purpleCollectPose = new Pose( purpleCollectX, purpleCollectY, Math.toRadians(purpleCollectHeading));
    private final Pose leavePose = new Pose(leaveX, leaveY, Math.toRadians(leaveHeading));


    private PathChain gotoFirstShootPose, gotoFirstLoadingZoneCollectPose, gotoSecondLoadingZoneCollectPose, gotoSecondShootPose, gotoPurpleCollectPose, gotoGreenCollectPose, gotoThirdShootPose, gotoLeavePose, gotoGPPCollect, gotoPGPCollect, gotoPPGCollect;

    //private Supplier<PathChain> extra;

    public void buildPaths() {
        gotoFirstShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, firstShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), firstShootPose.getHeading())
                .build();

        gotoFirstLoadingZoneCollectPose = follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, firstLoadingZoneCollectPose))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), firstLoadingZoneCollectPose.getHeading())
                .build();

        gotoSecondLoadingZoneCollectPose = follower.pathBuilder()
                .addPath(new BezierLine(firstLoadingZoneCollectPose, secondLoadingZoneCollectPose))
                .setLinearHeadingInterpolation(firstLoadingZoneCollectPose.getHeading(), secondLoadingZoneCollectPose.getHeading())
                .build();

        gotoSecondShootPose =  follower.pathBuilder()
                .addPath(new BezierLine(secondLoadingZoneCollectPose, firstShootPose))
                .setLinearHeadingInterpolation(secondLoadingZoneCollectPose.getHeading(), firstShootPose.getHeading())
                .build();

        gotoGreenCollectPose = follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, greenCollectPose))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), greenCollectPose.getHeading())
                .build();

        gotoPurpleCollectPose = follower.pathBuilder()
                .addPath(new BezierLine(greenCollectPose, purpleCollectPose))
                .setLinearHeadingInterpolation(greenCollectPose.getHeading(), purpleCollectPose.getHeading())
                .build();

        gotoThirdShootPose =  follower.pathBuilder()
                .addPath(new BezierLine(purpleCollectPose, firstShootPose))
                .setLinearHeadingInterpolation(purpleCollectPose.getHeading(), firstShootPose.getHeading())
                .build();

        gotoLeavePose = follower.pathBuilder()
                .addPath(new BezierLine(firstShootPose, leavePose))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), leavePose.getHeading())
                .build();
/*
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
                .build();*/
    }

    public void shootPreloadMotif(ObeliskState oState) {
        // TODO - Refactor with Single Launcher
        switch (oState) {
            case PURPLE_GREEN_PURPLE:
                break;
            case GREEN_PURPLE_PURPLE:
                break;
            case PURPLE_PURPLE_GREEN:
            default:
                break;
        }
    }

    public void shootGPP(ObeliskState oState) {
        // TODO - Refactor with Single Launcher
        switch (oState) {
            case PURPLE_GREEN_PURPLE:
                break;
            case GREEN_PURPLE_PURPLE:
                break;
            case PURPLE_PURPLE_GREEN:
            default:
                break;
        }
    }

    public void shootPGP(ObeliskState oState) {
        // TODO - Refactor with Single Launcher
        switch (oState) {
            case PURPLE_GREEN_PURPLE:
                break;
            case GREEN_PURPLE_PURPLE:
                break;
            case PURPLE_PURPLE_GREEN:
            default:
                break;
        }
    }

    public PathChain getCollectionPath(ObeliskState oState) {
        switch (oState) {
            case PURPLE_GREEN_PURPLE:
                return gotoPGPCollect;
            case PURPLE_PURPLE_GREEN:
                return gotoPPGCollect;
            case GREEN_PURPLE_PURPLE:
            default:
                return gotoGPPCollect;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                launcher.startShooter(shootVelocity);
                follower.followPath(gotoFirstShootPose, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    shootPreloadMotif(oState);
                    setPathState(2);
                }
                break;
            case 2:
                // Launcher should have its own caser for this
                if (!launcher.isBusy()) {
                    launcher.stopShooter();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    //launcher.activateIntake();
                    follower.followPath(gotoFirstLoadingZoneCollectPose, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    diverter.setPosition(0.34);
                    follower.followPath( gotoSecondLoadingZoneCollectPose, true);
                    setPathState(5);
                }
            case 5:
                if(!follower.isBusy()) {
                    launcher.startShooter(shootVelocity);
                    follower.followPath(gotoSecondShootPose,true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    //launcher.deactivateIntake();
                    //shootLoadingZoneBalls(oState);
                    setPathState(7);
                }
                break;
            case 7:
                if (!launcher.isBusy()) {
                    launcher.stopShooter();
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    //launcher.activateIntake();
                    follower.followPath(gotoLeavePose, true);
                    setPathState(-1);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    diverter.setPosition(0.02);
                    //insert sleep
                    follower.followPath(gotoPurpleCollectPose, true);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    //launcher.deactivateIntake();
                    launcher.startShooter(shootVelocity);
                    follower.followPath(gotoThirdShootPose,true);
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()) {
                    shootPreloadMotif(oState);
                    setPathState(12);
                }
                break;
            case 12:
                if (!launcher.isBusy()) {
                    launcher.stopShooter();
                    setPathState(13);
                }
                break;
            case 13:
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
        diverter = hardwareMap.get(Servo.class, "diverter");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        pipeline = 0;
        limelight.pipelineSwitch(pipeline); // Switch to pipeline number 1
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        launcher = new SingleLauncher(hardwareMap, telemetry);
    }

    public void start() {
        pathState = 0;
        diverter.setPosition(0.02);
        intake.setPower(0);
    }

    @Override
    public void loop() {

        limelight.updateRobotOrientation(Math.toDegrees(follower.getHeading()));
        LLResult result = limelight.getLatestResult();

        if (pipeline == 0 && result != null && result.isValid() && oState == ObeliskState.UNKNOWN) {
            int id = result.getFiducialResults().get(0).getFiducialId();
            oState = ObeliskState.fromInt(id);
            pipeline = 1;
        }

        if (pipeline == 1 && result != null && result.isValid()) {
            LLResultTypes.FiducialResult fResult = result.getFiducialResults().get(0);
            int id = result.getFiducialResults().get(0).getFiducialId();
            logger.logData("April Tag ID", "" + id);

            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                logger.logData("MT2 Target:", "(" + tx + ", " + ty + ", " + ta + ")");
                double distance = DistanceCalculation.getTrigDistanceToTarget(ty);
                logger.logData("Distance to Goal", distance);
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                logger.logData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        } else {
            logger.logData("Limelight", "No Targets");
        }

        follower.update();
        autonomousPathUpdate();
        launcher.update();

        // Feedback to Driver Hub for debugging
        logger.logData("path state", pathState);
        logger.logData("launcher state", launcher.state);
        logger.logData("x", follower.getPose().getX());
        logger.logData("y", follower.getPose().getY());
        logger.logData("heading", follower.getPose().getHeading());
        logger.logData("right shooter velocity", rightShooter.getVelocity());
        logger.logData("left shooter velocity", leftShooter.getVelocity());
        logger.update();
    }
}
