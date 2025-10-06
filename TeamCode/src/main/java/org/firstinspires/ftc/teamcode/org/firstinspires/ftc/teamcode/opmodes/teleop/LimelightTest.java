package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "Limelight Obelisk", group = "Examples")
public class LimelightTest extends OpMode {

    Limelight3A limelight;

    String[] motif = { "green, purple, purple", "purple, green, purple", "purple, purple, green" };

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            int id = result.getFiducialResults().get(0).getFiducialId();
            telemetry.addData("April Tag ID", "" + id);
            telemetry.addData( "Motif", motif[ id % 10 - 1]);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
    }
}
