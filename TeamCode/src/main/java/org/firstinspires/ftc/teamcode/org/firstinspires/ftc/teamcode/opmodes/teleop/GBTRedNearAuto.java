package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.auto.NearSideAutoRed;

@Configurable
@TeleOp(name = "GBT Red Near Auto", group = "Examples")
public class GBTRedNearAuto extends GBTAutoTeleOpBase {

    @Override
    protected Pose getStartingPose() {
        return new Pose(NearSideAutoRed.leaveX, NearSideAutoRed.leaveY, Math.toRadians(NearSideAutoRed.leaveHeading));
    }

    @Override
    protected Pose getShootPoseNear() {
        return new Pose(144 - 72.1, 75.15, Math.toRadians(180 - 135));
    }

    @Override
    protected Pose getShootPoseFar() {
        return new Pose(144 - 67.02, 19.57, Math.PI - 2.037);
    }

    @Override
    protected int getLimelightPipeline() {
        return 2;
    }
}
