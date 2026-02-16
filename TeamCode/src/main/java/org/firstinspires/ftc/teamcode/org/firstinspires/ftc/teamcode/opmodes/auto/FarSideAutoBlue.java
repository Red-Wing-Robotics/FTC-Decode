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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.RWRBaseOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.state.SingleLauncher;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.ObeliskState;

@SuppressWarnings("unused")
@Configurable
@Autonomous(name = "Far Side Auto Blue", group = "Examples")
public class FarSideAutoBlue extends RWRBaseOpMode {

    Limelight3A limelight;

    private int pipeline;

    private Follower follower;
    private SingleLauncher launcher;


    private int pathState;

    private ObeliskState oState = ObeliskState.UNKNOWN;

    public DcMotor intake = null;
    public Servo turret = null;


    public static double shootVelocity = 1800;
    public static double SHOOTER_VELOCITY_FUDGE_FACTOR = 100;

    public static long TIMEOUT_DEFAULT = 5000;

    // POSES -------------------------------------------------------------

    public static double startX = 57.2;
    public static double startY = 16.09375/2d;
    public static double startHeading = 90;
    public static double shootX = 57.2;
    public static double shootY = 10;
    public static double shootHeading = 90;
    public static double secondLoadingZoneY = 15;
    public static double secondLoadingZoneX = 14;
    public static double secondLoadingZoneHeading = 180;
    public static double firstLoadingZoneY = 14.3;
    public static double firstLoadingZoneX = 14;
    public static double firstLoadingZoneHeading = 180;
    public static double ppgX = 50;
    public static double ppgY = 37;
    public static double ppgHeading = 180;
    public static double ppgGreenX = 32;
    public static double ppgGreenY = 37;
    public static double ppgGreenHeading = 180;
    public static double ppgPurpleX = 25;
    public static double ppgPurpleY = 37;
    public static double ppgPurpleHeading = 180;
    public static double ppgLastX = 15;
    public static double ppgLastY = 37;
    public static double ppgLastHeading = 180;
    public static double secondPurpleCollectX = 15;
    public static double leaveY = 24;
    public static double leaveX = 24;
    public static double leaveHeading = 180;

    private final Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));
    private final Pose shootPose = new Pose(shootX, shootY, Math.toRadians(shootHeading));
    private final Pose firstLoadingZonePose = new Pose(firstLoadingZoneX, firstLoadingZoneY, Math.toRadians(firstLoadingZoneHeading));
    private final Pose secondLoadingZonePose = new Pose(secondLoadingZoneX, secondLoadingZoneY, Math.toRadians(secondLoadingZoneHeading));
    private final Pose ppgPose = new Pose( ppgX, ppgY, Math.toRadians(ppgHeading));
    private final Pose ppgGreenPose = new Pose( ppgGreenX, ppgGreenY, Math.toRadians(ppgGreenHeading));
    private final Pose ppgPurplePose = new Pose(ppgPurpleX, ppgPurpleY, Math.toRadians(ppgPurpleHeading));
    private final Pose ppgLastPose = new Pose( secondPurpleCollectX, ppgLastY, Math.toRadians(ppgLastHeading));
    private final Pose leavePose = new Pose(leaveX, leaveY, Math.toRadians(leaveHeading));


    private PathChain gotoShootPose, gotoFirstLoadingZonePose, gotoSecondLoadingZonePose, gotoSecondShootPose, gotoPPGPose, gotoPPGGreenPose, gotoPPGPurplePose, gotoPPGLastPose, gotoThirdShootPose, gotoLeavePose;

    //private Supplier<PathChain> extra;

    public void buildPaths() {
        gotoShootPose = follower.pathBuilder()
                .addPath(new BezierLine(ppgLastPose, shootPose))
                .setLinearHeadingInterpolation(ppgLastPose.getHeading(), shootPose.getHeading())
                .build();

        gotoFirstLoadingZonePose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstLoadingZonePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstLoadingZonePose.getHeading())
                .build();

        gotoSecondLoadingZonePose = follower.pathBuilder()
                .addPath(new BezierLine(firstLoadingZonePose, secondLoadingZonePose))
                .setLinearHeadingInterpolation(firstLoadingZonePose.getHeading(), secondLoadingZonePose.getHeading())
                .build();

        gotoSecondShootPose =  follower.pathBuilder()
                .addPath(new BezierLine(secondLoadingZonePose, shootPose))
                .setLinearHeadingInterpolation(secondLoadingZonePose.getHeading(), shootPose.getHeading())
                .build();

        gotoPPGPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, ppgPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), ppgPose.getHeading())
                .build();

        gotoPPGGreenPose = follower.pathBuilder()
                .addPath(new BezierLine(ppgPose, ppgGreenPose))
                .setLinearHeadingInterpolation(ppgPose.getHeading(), ppgGreenPose.getHeading())
                .build();

        gotoPPGPurplePose = follower.pathBuilder()
                .addPath(new BezierLine(ppgGreenPose, ppgPurplePose))
                .setLinearHeadingInterpolation(ppgGreenPose.getHeading(), ppgPurplePose.getHeading())
                .build();

        gotoPPGLastPose = follower.pathBuilder()
                .addPath(new BezierLine(ppgPurplePose, ppgLastPose))
                .setLinearHeadingInterpolation(ppgPurplePose.getHeading(), ppgLastPose.getHeading())
                .build();

        gotoThirdShootPose =  follower.pathBuilder()
                .addPath(new BezierLine(ppgLastPose, shootPose))
                .setLinearHeadingInterpolation(ppgLastPose.getHeading(), shootPose.getHeading())
                .build();

        gotoLeavePose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
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
/*
    public void shootPreloadMotif(ObeliskState oState) {
        // TODO - Refactor with Single Launcher
        // Preload will be Shoot: Purple; Intake: Green; Extra: Purple
        switch (oState) {
            case PURPLE_GREEN_PURPLE:
                launcher.shootShoot();
                launcher.shootIntake();
                launcher.shootIntake();
                break;
            case GREEN_PURPLE_PURPLE:
                launcher.shootIntake();
                launcher.shootIntake();
                launcher.shootIntake();
                break;
            case PURPLE_PURPLE_GREEN:
                launcher.shootShoot();
                launcher.shootExtra();
                launcher.shootExtra();
            default:
                break;
        }
    }

    public void shootGPP(ObeliskState oState) {
        // TODO - Refactor with Single Launcher
        // load will be Shoot: Purple; Intake: Purple; Extra: Green
        switch (oState) {
            case PURPLE_GREEN_PURPLE:
                launcher.shootShoot();
                launcher.shootExtra();
                launcher.shootExtra();
                break;
            case GREEN_PURPLE_PURPLE:
                launcher.shootExtra();
                launcher.shootExtra();
                launcher.shootExtra();
                break;
            case PURPLE_PURPLE_GREEN:
                launcher.shootShoot();
                launcher.shootIntake();
                launcher.shootIntake();
            default:
                break;
        }
    }*/

    public void shoot(ObeliskState oState) {
        // TODO - Refactor with Single Launcher
        // load will be Shoot: Purple; Intake: Purple; Extra: Green
        switch (oState) {
            case PURPLE_GREEN_PURPLE:
                launcher.shootShoot();
                launcher.shootExtra();
                launcher.shootExtra();
                break;
            case GREEN_PURPLE_PURPLE:
                launcher.shootExtra();
                launcher.shootExtra();
                launcher.shootExtra();
                break;
            case PURPLE_PURPLE_GREEN:
                launcher.shootShoot();
                launcher.shootIntake();
                launcher.shootIntake();
            default:
                break;
        }
    }
/*
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
    }*/

    public void setPathState(int pState) {
        pathState = pState;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                launcher.startShooter();
                launcher.setShooterVelocity(shootVelocity);
                turret.setPosition(0.75);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    shoot(oState);
                    setPathState(2);
                }
                break;
            case 2:
                if (!launcher.isBusy()) {
                    launcher.stopShooter();
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()){
                    intake.setPower(1.0);
                    follower.followPath(gotoPPGPose, true);
                    launcher.turnSpindexerCounterClockwise();
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    follower.followPath( gotoPPGGreenPose, true);
                    setPathState(5);
                }
            case 5:
                if(!follower.isBusy()) {
                    launcher.turnSpindexerCounterClockwise();
                    follower.followPath(gotoPPGPurplePose,true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    launcher.turnSpindexerCounterClockwise();
                    follower.followPath(gotoPPGLastPose,true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    launcher.setShooterVelocity(shootVelocity);
                    follower.followPath(gotoShootPose);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    shoot(oState);
                    setPathState(9);
                }
                break;
            case 9:
                if (!launcher.isBusy()) {
                    follower.followPath(gotoLeavePose, true);
                    setPathState(-1);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
                    launcher.turnSpindexerCounterClockwise();
                    //insert sleep
                    follower.followPath(gotoPPGLastPose, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    //launcher.deactivateIntake();
                    launcher.setShooterVelocity(shootVelocity);
                    follower.followPath(gotoThirdShootPose,true);
                    setPathState(12);
                }
                break;
            case 12:
                if (!follower.isBusy()) {
                    shoot(oState);
                    setPathState(13);
                }
                break;
            case 13:
                if (!launcher.isBusy()) {
                    launcher.stopShooter();
                    setPathState(14);
                }
                break;
            case 14:
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
        turret = hardwareMap.get(Servo.class, "turret");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        pipeline = 0;
        limelight.pipelineSwitch(pipeline); // Switch to pipeline number 1
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        launcher = new SingleLauncher(hardwareMap, telemetry, null);
    }

    public void start() {
        pathState = 0;
        //diverter.setPosition(0.02);
        intake.setPower(0);
        launcher.initializeSpindexer();
        turret.setPosition(0.5);
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
        logger.logData("shooter velocity", launcher.getShooterVelocity());
        logger.update();
    }
}

