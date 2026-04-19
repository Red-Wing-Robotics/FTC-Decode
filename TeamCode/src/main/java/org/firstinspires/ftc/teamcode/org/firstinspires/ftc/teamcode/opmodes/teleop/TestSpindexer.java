package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.state.Spindexer;

@SuppressWarnings("unused")
@Configurable
@TeleOp(name = "Test Spindexer (RTPAxon)", group = "Testing")
public class TestSpindexer extends OpMode {

    // Live-tunable overrides. Defaults mirror Spindexer's baked-in values.
    public static double MAX_POWER = Spindexer.MAX_POWER;
    public static double KP = Spindexer.KP;
    public static double KI = Spindexer.KI;
    public static double KD = Spindexer.KD;

    private Spindexer spindexer;

    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;

    private double lastMaxPower = Double.NaN;
    private double lastKP = Double.NaN;
    private double lastKI = Double.NaN;
    private double lastKD = Double.NaN;

    @Override
    public void init() {
        spindexer = new Spindexer(hardwareMap, telemetry);
        applyTuning();
    }

    @Override
    public void start() {
        applyTuning();
        spindexer.initialize();
    }

    @Override
    public void loop() {
        applyTuning();

        if (gamepad1.dpad_right && !dpadRightPressed) {
            spindexer.moveClockwise();
            dpadRightPressed = true;
        } else if (!gamepad1.dpad_right) {
            dpadRightPressed = false;
        }

        if (gamepad1.dpad_left && !dpadLeftPressed) {
            spindexer.moveCounterClockwise();
            dpadLeftPressed = true;
        } else if (!gamepad1.dpad_left) {
            dpadLeftPressed = false;
        }

        spindexer.update();

        telemetry.addData("Position", spindexer.position);
        telemetry.addData("State", spindexer.state);
        telemetry.addData("Mode", spindexer.mode);
        telemetry.addData("Total Rotation (deg)", spindexer.getPosition());
        telemetry.addLine(spindexer.log());
        telemetry.update();
    }

    private void applyTuning() {
        if (MAX_POWER != lastMaxPower) {
            spindexer.spindexer.setMaxPower(MAX_POWER);
            lastMaxPower = MAX_POWER;
        }
        if (KP != lastKP || KI != lastKI || KD != lastKD) {
            spindexer.spindexer.setPidCoeffs(KP, KI, KD);
            lastKP = KP;
            lastKI = KI;
            lastKD = KD;
        }
    }
}
