package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.auto.FarSideAutoRed;
import org.firstinspires.ftc.teamcode.util.Alliance;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "GBT Red Far Auto", group = "Examples")
public class GBTRedFarAuto extends GBTAutoTeleOpBase {

    @Override
    protected Pose getStartingPose() {
        return new Pose(FarSideAutoRed.leaveX, FarSideAutoRed.leaveY, Math.toRadians(FarSideAutoRed.leaveHeading));
    }

    @Override
    protected Pose getShootPoseNear() {
        return new Pose(81, 92, Math.toRadians(180 - 135));
    }

    @Override
    protected Pose getShootPoseFar() {
        return new Pose(144 - 67.02, 19.57, Math.PI - 2.037);
    }

    @Override
    protected int getLimelightPipeline() {
        return 2;
    }

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}
