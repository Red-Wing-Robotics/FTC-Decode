package org.firstinspires.ftc.teamcode.state;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

@Configurable
public class Turret {

    // Proportional gain: how much servo position changes per degree of tx.
    // Start small (0.005 to 0.02). Increase until it tracks well but doesn't oscillate.
    public static double kP = 0.015;

    // Exponential smoothing factor:
    // 0.10 = very smooth (more lag), 0.30 = more responsive (less smoothing).
    public static double alpha = 0.25;

    // Degrees inside which we treat tx as "close enough" to avoid buzzing.
    public static double deadbandDeg = 1.0;

    // Maximum change in servo position per update (limits speed / prevents jumps).
    // Useful if your loop timing varies or you want smoother motion.
    public static double maxStepPerUpdate = 0.03;

    public static double TURRET_MIDPOINT = 0.5;

    private boolean enabled = true;

    // Servo limits and state
    public static double minPos = 0.10;
    public static double maxPos = 0.90;
    private double turretPos;

    // Filter state
    private double filteredTxDeg = 0.0;
    private double lastTxDeg = 0.0;
    private boolean lastHasTarget = false;

    public final Servo turret;
    private final Logger logger;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.logger = new Logger(telemetry);
        turret = hardwareMap.get(Servo.class, "turret");
        setToDefault();
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (!enabled) {
            turret.setPosition(TURRET_MIDPOINT);
        }
    }

    public void update() {
        filteredTxDeg = 0.0;
        lastHasTarget = false;
        setToDefault();
        logTurret();
    }

    public void update(LLResult result) {
        // bail out if not enabled
        if (!enabled) {
            return;
        }

        if (result != null && result.isValid()) {
            double txDeg = result.getTx(); // degrees

            lastHasTarget = true;
            lastTxDeg = txDeg;

            // Exponential smoothing (EMA)
            filteredTxDeg = alpha * txDeg + (1.0 - alpha) * filteredTxDeg;

            // Deadband to prevent chatter
            if (Math.abs(filteredTxDeg) < deadbandDeg) {
                filteredTxDeg = 0.0;
                return; // we're basically aligned; hold position
            }

            // Proportional correction
            // NOTE: Sign convention matters.
            // Commonly, if tx is positive (target right), turret must rotate left -> negative servo delta.
            double delta = -kP * filteredTxDeg;

            // Limit step per loop (optional but helpful)
            delta = clip(delta, -maxStepPerUpdate, maxStepPerUpdate);

            // Apply and clamp to turret safe range
            turretPos = clip(turretPos + delta, minPos, maxPos);
            turret.setPosition(turretPos);
            logTurret();
        } else {
            update();
        }
    }

    private void setToDefault() {
        turret.setPosition(TURRET_MIDPOINT);
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void logTurret() {
        logger.logData("Turret TX", lastTxDeg);
        logger.logData("Turret Pos", turretPos);
        logger.logData("Turret Enabled", enabled);
    }

}
