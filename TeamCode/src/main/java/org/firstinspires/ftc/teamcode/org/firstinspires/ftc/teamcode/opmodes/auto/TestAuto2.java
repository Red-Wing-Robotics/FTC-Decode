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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.ObeliskState;

import java.util.function.Supplier;

@Configurable
@Autonomous(name = "Test Auto 2", group = "Examples")
public class TestAuto2 extends OpMode {

    Limelight3A limelight;

    private int pipeline;

    private Follower follower;
    private int pathState;

    private ObeliskState oState = ObeliskState.UNKNOWN;

    // POSES -------------------------------------------------------------

    private final Pose startPose = new Pose(56.25, 8.25, Math.toRadians(90));

    private final Pose shootPose = new Pose(59.5, 83.69, Math.toRadians(135));

    private final Pose gppPose = new Pose(38.86, 36.68, Math.toRadians(180));

    private final Pose pgpPose = new Pose(38.86, 60.16, Math.toRadians(180));

    private final Pose ppgPose = new Pose(38.86, 82.93, Math.toRadians(180));

    private final Pose gppCollectPose = new Pose(21.9, 36.68, Math.toRadians(180));

    private final Pose pgpCollectPose = new Pose(21.9, 60.16, Math.toRadians(180));

    private final Pose ppgCollectPose = new Pose(21.9, 82.93, Math.toRadians(180));

    private Pose preCollectPose;

    private Pose postCollectPose;

    private PathChain gotoShootPose1, gotoGPPCollect, gotoPGPCollect, gotoPPGCollect;

    private Supplier<PathChain> gotoCollectPose, gotoShootPose2;
    public void buildPaths() {
        gotoShootPose1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        gotoCollectPose = () -> follower.pathBuilder()
                .addPath(new BezierLine(shootPose, getCollectPose( oState )))
                .setLinearHeadingInterpolation(shootPose.getHeading(), getCollectPose( oState ).getHeading())
                .build();

        gotoShootPose2 = () -> follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, shootPose))
                .setLinearHeadingInterpolation(follower.getHeading(), shootPose.getHeading())
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

    public Pose getCollectPose( ObeliskState oState ){
        switch (oState){
            case PURPLE_GREEN_PURPLE:
                return pgpPose;
            case PURPLE_PURPLE_GREEN:
                return ppgPose;
            case GREEN_PURPLE_PURPLE:
                return gppPose;
            default:
                return pgpPose;
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
                follower.followPath(gotoShootPose1);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()){
                    follower.followPath(gotoCollectPose.get());
                    setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(getCollectionPath(oState));
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    follower.followPath(gotoShootPose2.get());
                    setPathState(-1);
                }
                break;
        }
    }

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        pipeline = 0;
        limelight.pipelineSwitch(pipeline); // Switch to pipeline number 1
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

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
                double distance = getDistanceToTarget(ty);
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

    public void start() {
        setPathState(0);
    }
    private double getDistanceToTarget(double ty) {
        double targetHeight = 29.4375;
        double limelightHeight = 13.34375;
        double limelightAngle = 0;

        double angleToTarget = Math.toRadians(limelightAngle + ty);
        return (targetHeight - limelightHeight) / Math.tan(angleToTarget);
    }
}
