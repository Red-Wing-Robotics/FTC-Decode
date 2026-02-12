package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.auto.NearSideAutoBlue;
import org.firstinspires.ftc.teamcode.util.Alliance;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "GBT Blue Near Auto", group = "Examples")
public class GBTBlueNearAuto extends GBTAutoTeleOpBase {

    @Override
    protected Pose getStartingPose() {
        return new Pose(NearSideAutoBlue.leaveX, NearSideAutoBlue.leaveY, Math.toRadians(NearSideAutoBlue.leaveHeading));
    }

    @Override
    protected Pose getShootPoseNear() {
        return new Pose(72.1, 75.15, Math.toRadians(135));
    }

    @Override
    protected Pose getShootPoseFar() {
        return new Pose(67.02, 19.57, 2.037);
    }

    @Override
    protected int getLimelightPipeline() {
        return 1;
    }

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
