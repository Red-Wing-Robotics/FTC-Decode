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
import org.firstinspires.ftc.teamcode.util.timer.NonBlockingTimer;

@SuppressWarnings("unused")
@Configurable
@Autonomous(name = "Near Side Auto Red", group = "Examples")
public class NearSideAutoRed extends RWRBaseOpMode {

    Limelight3A limelight;

    private int pipeline;

    private Follower follower;

    private SingleLauncher launcher;
    private NonBlockingTimer waitTimer;
    public static long WAIT_TIME = 500;

    private int pathState;

    private ObeliskState oState = ObeliskState.UNKNOWN;

    public DcMotor intake = null;
    public DcMotorEx leftShooter = null;
    public DcMotorEx rightShooter = null;
    public CRServo rightFeeder = null;
    public CRServo leftFeeder = null;
    public Servo diverter = null;

    public static double shootVelocity = 1060;
    public static double SHOOTER_VELOCITY_FUDGE_FACTOR = 100;

    public static long TIMEOUT_DEFAULT = 5000;

    // POSES -------------------------------------------------------------

    public static double startX = 144 - NearSideAutoBlue.startX;
    public static double startY = 144 - (16.09375/2d);
    public static double startHeading = NearSideAutoBlue.startHeading;
    public static double shootX = 144 - NearSideAutoBlue.shootX;
    public static double shootY = NearSideAutoBlue.shootY;
    public static double shootHeading = 180 - NearSideAutoBlue.shootHeading;//90-(135-90) = 45
    public static double obeliskReadHeading = 180 - NearSideAutoBlue.obeliskReadHeading;//90-(75-90) = 105
    public static double gppX = 144 - NearSideAutoBlue.gppX;
    public static double gppY = NearSideAutoBlue.gppY;
    public static double gppHeading = 180 - NearSideAutoBlue.gppHeading;
    public static double gppPurpleY = NearSideAutoBlue.gppPurpleY;
    public static double gppPurpleX = 144 - NearSideAutoBlue.gppPurpleX;
    public static double gppPurpleHeading = 180 - NearSideAutoBlue.gppPurpleHeading;
    public static double gppGreenX = 144 - NearSideAutoBlue.gppGreenX;
    public static double gppGreenY = 83.85;
    public static double gppGreenHeading = 180 - NearSideAutoBlue.gppGreenHeading;
    public static double pgpX = 144 - NearSideAutoBlue.pgpX;
    public static double pgpY = 59.34;
    public static double pgpHeading = 180 - NearSideAutoBlue.pgpHeading;
    public static double pgpPurpleX = 144 - NearSideAutoBlue.pgpPurpleX;
    public static double pgpPurpleY = 59.34;
    public static double pgpPurpleHeading = 180 - NearSideAutoBlue.pgpPurpleHeading;
    public static double pgpGreenX = 144 - NearSideAutoBlue.pgpGreenX;
    public static double pgpGreenY = 59.34;
    public static double pgpGreenHeading = 180 - NearSideAutoBlue.pgpGreenHeading;
    public static double leaveY = NearSideAutoBlue.leaveY;
    public static double leaveX = 144 - NearSideAutoBlue.leaveX;
    public static double leaveHeading = 180 - NearSideAutoBlue.leaveHeading;

    private final Pose startPose = new Pose(startX, startY, Math.toRadians(startHeading));
    private final Pose obeliskReadPose = new Pose(shootX, shootY, Math.toRadians(obeliskReadHeading));
    private final Pose shootPose = new Pose(shootX, shootY, Math.toRadians(shootHeading));
    private final Pose gppPose = new Pose(gppX, gppY, Math.toRadians(gppHeading));
    private final Pose gppPurplePose = new Pose(gppPurpleX, gppPurpleY, Math.toRadians(gppPurpleHeading));
    private final Pose gppGreenPose = new Pose(gppGreenX, gppGreenY, Math.toRadians(gppGreenHeading));
    private final Pose pgpPose = new Pose(pgpX, pgpY, Math.toRadians(pgpHeading));
    private final Pose pgpPurplePose = new Pose(pgpPurpleX, pgpPurpleY, Math.toRadians(pgpPurpleHeading));
    private final Pose pgpGreenPose = new Pose(pgpGreenX, pgpGreenY, Math.toRadians(pgpGreenHeading));
    private final Pose leavePose = new Pose(leaveX, leaveY, Math.toRadians(leaveHeading));


    private PathChain gotoObeliskReadPose, gotoFirstShootPose, gotoGppPose, gotoGppPurplePose, gotoGppGreenPose, gotoSecondShootPose, gotoPgpPose, gotoPgpPurplePose, gotoPgpGreenPose, gotoThirdShootPose, gotoLeavePose;

    //private Supplier<PathChain> extra;

    public void buildPaths() {
        gotoObeliskReadPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, obeliskReadPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), obeliskReadPose.getHeading())
                .build();

        gotoFirstShootPose = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        gotoGppPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, gppPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), gppPose.getHeading())
                .build();

        gotoGppPurplePose = follower.pathBuilder()
                .addPath(new BezierLine(gppPose, gppPurplePose))
                .setLinearHeadingInterpolation(gppPose.getHeading(), gppPurplePose.getHeading())
                .build();

        gotoGppGreenPose = follower.pathBuilder()
                .addPath(new BezierLine(gppPurplePose, gppGreenPose))
                .setLinearHeadingInterpolation(gppPurplePose.getHeading(), gppGreenPose.getHeading())
                .build();

        gotoSecondShootPose =  follower.pathBuilder()
                .addPath(new BezierLine(gppGreenPose, shootPose))
                .setLinearHeadingInterpolation(gppGreenPose.getHeading(), shootPose.getHeading())
                .build();

        gotoPgpPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, pgpPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pgpPose.getHeading())
                .build();

        gotoPgpPurplePose = follower.pathBuilder()
                .addPath(new BezierLine(pgpPose, pgpPurplePose))
                .setLinearHeadingInterpolation(pgpPose.getHeading(), pgpPurplePose.getHeading())
                .build();

        gotoPgpGreenPose = follower.pathBuilder()
                .addPath(new BezierLine(pgpPurplePose, pgpGreenPose))
                .setLinearHeadingInterpolation(pgpPurplePose.getHeading(), pgpGreenPose.getHeading())
                .build();

        gotoThirdShootPose =  follower.pathBuilder()
                .addPath(new BezierLine(pgpGreenPose, shootPose))
                .setLinearHeadingInterpolation(pgpGreenPose.getHeading(), shootPose.getHeading())
                .build();

        gotoLeavePose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, leavePose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), leavePose.getHeading())
                .build();


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
                launcher.setShooterVelocity(shootVelocity);
                follower.followPath(gotoFirstShootPose, true);
                setPathState(2);
                break;
            /*case 1:
                if (!follower.isBusy()) {
                    follower.followPath(gotoObeliskReadPose, true);
                    setPathState(2);
                }
                break;*/
            case 2:
                if (!follower.isBusy()) {
                    shootPreloadMotif(oState);
                    setPathState(3);
                }
                break;
            case 3:
                if (!launcher.isBusy()) {
                    launcher.stopShooter();
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()){
                    //launcher.activateIntake();
                    follower.followPath(gotoGppPose, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()){
                    follower.followPath( gotoGppPurplePose, true);
                    setPathState(6);
                }
            case 6:
                if (!follower.isBusy()){
                    diverter.setPosition(0.34);
                    waitTimer.start();
                    setPathState(7);
                }
            case 7:
                if (waitTimer.isFinished()) {
                    waitTimer.reset();
                    follower.followPath( gotoGppGreenPose, true);
                    setPathState(8);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    //launcher.deactivateIntake();
                    launcher.setShooterVelocity(shootVelocity);
                    follower.followPath(gotoSecondShootPose,true);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    shootGPP(oState);
                    setPathState(10);
                }
                break;
            case 10:
                if (!launcher.isBusy()) {
                    launcher.stopShooter();
                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy()){
                    follower.followPath(gotoLeavePose, true);
                    setPathState(-1);
                }
                break;
            case 12:
                if (!follower.isBusy()){
                    follower.followPath( gotoPgpPurplePose, true);
                    setPathState(13);
                }
            case 13:
                if (!follower.isBusy()){
                    diverter.setPosition(0.34);
                    //insert sleep
                    follower.followPath( gotoPgpGreenPose, true);
                    setPathState(14);
                }
            case 14:
                if(!follower.isBusy()) {
                    //launcher.deactivateIntake();
                    launcher.setShooterVelocity(shootVelocity);
                    follower.followPath(gotoSecondShootPose,true);
                    setPathState(15);
                }
                break;
            case 15:
                if (!follower.isBusy()) {
                    shootPGP(oState);
                    setPathState(16);
                }
                break;
            case 16:
                if (!launcher.isBusy()) {
                    launcher.stopShooter();
                    setPathState(17);
                }
                break;
            case 17:
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

        launcher = new SingleLauncher(hardwareMap, telemetry, null);
        waitTimer = new NonBlockingTimer(WAIT_TIME);
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
