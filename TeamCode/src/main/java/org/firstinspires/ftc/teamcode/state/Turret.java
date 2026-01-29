package org.firstinspires.ftc.teamcode.state;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

enum TurretState {
    DISABLED,
    READY,
    TRACKING,
    SEARCHING,
}

@Configurable
public class Turret {

    // Proportional gain: how much servo position changes per degree of tx.
    // Start small (0.005 to 0.02). Increase until it tracks well but doesn't oscillate.
    public static double kP = 0.001;

    // Exponential smoothing factor:
    // 0.10 = very smooth (more lag), 0.30 = more responsive (less smoothing).
    public static double alpha = 1.0;

    // Degrees inside which we treat tx as "close enough" to avoid buzzing.
    public static double deadbandDeg = 1.5;

    // Maximum change in servo position per update (limits speed / prevents jumps).
    // Useful if your loop timing varies or you want smoother motion.
    public static double maxStepPerUpdate = 0.03;

    public static double TURRET_MIDPOINT = 0.5;

    // Servo limits and state
    public static double minPos = 0;
    public static double maxPos = 1;
    private double turretPos;

    // Filter state
    private double filteredTxDeg = 0.0;
    private double lastTxDeg = 0.0;

    public final Servo turret;
    private final Logger logger;

    private TurretState state;

    private long searchStartTime = 0;

    public static long SEARCH_TIMEOUT_MS = 2000;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.logger = new Logger(telemetry);
        turret = hardwareMap.get(Servo.class, "turret");
        setToDefault();
        this.state = TurretState.READY;
    }

    private void setState(TurretState newState) {
        if (this.state == newState) return; // No change

        this.state = newState;
        logger.logLine("Turret new state: " + newState);

        switch (newState) {
            case DISABLED:
            case READY:
                turretPos = TURRET_MIDPOINT;
                turret.setPosition(turretPos);
                break;
            case SEARCHING:
                // Start the timer and initiate a sweep to the last known direction
                searchStartTime = System.currentTimeMillis();
                turretPos = lastTxDeg > 0 ? minPos : maxPos;
                turret.setPosition(turretPos);
                break;
            case TRACKING:
                // Reset timer when we start tracking again
                searchStartTime = 0;
                break;
        }
    }

    public void setEnabled(boolean enabled) {
        setState(enabled ? TurretState.READY : TurretState.DISABLED);
    }

    public void update(LLResult result) {
        if (state == TurretState.DISABLED) {
            return; // Do nothing if disabled
        }

        if (result != null && result.isValid()) {
            // --- TARGET FOUND ---
            setState(TurretState.TRACKING);

            double txDeg = result.getTx();
            lastTxDeg = txDeg;

            // Exponential smoothing (EMA)
            filteredTxDeg = alpha * txDeg + (1.0 - alpha) * filteredTxDeg;

            // Deadband to prevent chatter
            if (Math.abs(filteredTxDeg) > deadbandDeg) {
                // Proportional correction
                double delta = -kP * filteredTxDeg;
                delta = clip(delta, -maxStepPerUpdate, maxStepPerUpdate);
                turretPos = clip(turretPos + delta, minPos, maxPos);
                turret.setPosition(turretPos);
            }
        } else {
            // --- NO TARGET ---
            switch (state) {
                case TRACKING: // Target was just lost
                    setState(TurretState.SEARCHING);
                    break;
                case SEARCHING:
                    // Check if search has timed out
                    if (System.currentTimeMillis() - searchStartTime > SEARCH_TIMEOUT_MS) {
                        logger.logLine("Search timed out, returning to READY");
                        setState(TurretState.READY);
                    }
                    break;
                // If READY or DISABLED, do nothing
            }
        }
        logTurret();
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
        logger.logData("Turret State", state);
    }

}
