package org.firstinspires.ftc.teamcode.state;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.DistanceCalculation;
import org.firstinspires.ftc.teamcode.util.log.Logger;

enum OdometryTurretState {
    DISABLED,
    TRACKING,
}

@Configurable
public class OdometryTurret {

    // --- Tunable control constants ---

    // Proportional gain: how aggressively the servo corrects toward the target.
    // Start small (e.g. 0.3) and increase until it tracks well but doesn't oscillate.
    public static double kP = 0.9;

    // Derivative gain: dampens oscillation by resisting rapid changes in error.
    // Start at 0 and increase if the turret oscillates around the target.
    public static double kD = 0.05;

    // Exponential smoothing factor for the target position:
    // 1.0 = no smoothing (instant response), 0.1 = very smooth (more lag).
    public static double alpha = 0.6;

    // Radians of error inside which we treat position as "close enough" to avoid buzzing.
    public static double deadbandRad = Math.toRadians(1.0 );

    // Maximum change in servo position per update cycle (limits speed / prevents jumps).
    public static double maxStepPerUpdate = 0.12;

    // --- Vision fine-tuning constants ---

    // How much of the LimeLight tx error to apply as a correction (servo units per degree).
    // Keep this small -- it's a fine-tuning nudge, not the primary control.
    public static double visionKP = 0.0005;

    // EMA smoothing for the vision correction to avoid jitter.
    // Lower = smoother but laggier. 0.2-0.4 is a good starting range.
    public static double visionAlpha = 0.3;

    // Maximum vision correction in servo units. Caps the offset so a noisy/bad
    // reading can't throw off the odometry-based position.
    public static double maxVisionCorrection = 0.05;

    // --- Turret geometry constants ---

    // Servo position when the turret faces the robot's forward direction.
    public static double TURRET_MIDPOINT = 0.5;

    // Servo travel limits.
    public static double minPos = 0.0;
    public static double maxPos = 1.0;

    // Turret offset from robot center in inches (robot-frame coordinates).
    // +X = forward, +Y = left.  Tune these to match your physical turret mounting.
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = -4.0;

    // Angular offset (radians) of the turret's "forward" direction relative to the
    // robot's forward direction.  0 means turret forward == robot forward at midpoint.
    public static double TURRET_FORWARD_OFFSET_RAD = 0.0;

    // Conversion factor: servo units per radian of turret rotation.
    // For a 180-degree servo whose full range (0..1) maps to pi radians:
    //   RAD_TO_SERVO = 1.0 / Math.PI  ≈ 0.318
    // Adjust if your servo or gear ratio differs.
    public static double RAD_TO_SERVO = 1.0 / Math.toRadians(109.6);

    // --- Instance fields ---

    private double turretPos;
    private double filteredTargetPos;
    private double previousError = 0.0;

    // Vision correction state
    private double visionCorrection = 0.0;
    private double filteredTxDeg = 0.0;
    private boolean visionEnabled = false;

    public final Servo turret;
    private final Logger logger;
    private final Follower follower;
    private final double goalX;
    private final double goalY;

    private OdometryTurretState state;

    /**
     * Creates an OdometryTurret that uses Pedro Pathing odometry to aim at the
     * alliance-specific goal.
     *
     * @param hardwareMap FTC hardware map
     * @param telemetry   FTC telemetry for logging
     * @param follower    Pedro Pathing Follower (provides robot pose via odometry)
     * @param alliance    Which alliance goal to track (RED or BLUE)
     */
    public OdometryTurret(HardwareMap hardwareMap, Telemetry telemetry, Follower follower, Alliance alliance) {
        this.logger = new Logger(telemetry);
        this.follower = follower;
        this.turret = hardwareMap.get(Servo.class, "turret");

        // Select goal coordinates based on alliance
        if (alliance == Alliance.RED) {
            this.goalX = DistanceCalculation.redGoalx;
            this.goalY = DistanceCalculation.redGoaly;
        } else {
            this.goalX = DistanceCalculation.blueGoalx;
            this.goalY = DistanceCalculation.blueGoaly;
        }

        this.turretPos = TURRET_MIDPOINT;
        this.filteredTargetPos = TURRET_MIDPOINT;
        setToDefault();
        this.state = OdometryTurretState.TRACKING;
    }

    // --- State management ---

    private void setState(OdometryTurretState newState) {
        if (this.state == newState) return;

        this.state = newState;
        logger.logLine("OdometryTurret new state: " + newState);

        switch (newState) {
            case DISABLED:
                turretPos = TURRET_MIDPOINT;
                filteredTargetPos = TURRET_MIDPOINT;
                previousError = 0.0;
                visionCorrection = 0.0;
                filteredTxDeg = 0.0;
                visionEnabled = false;
                turret.setPosition(turretPos);
                break;
            case TRACKING:
                break;
        }
    }

    /**
     * Enable or disable the turret.  When disabled, the turret returns to its
     * midpoint and ignores updates.  When enabled, it immediately begins tracking.
     */
    public void setEnabled(boolean enabled) {
        setState(enabled ? OdometryTurretState.TRACKING : OdometryTurretState.DISABLED);
    }

    /**
     * Enable or disable LimeLight vision correction.
     * When disabled, the turret uses odometry only.
     * When disabled, any accumulated vision correction is cleared immediately.
     */
    public void setVisionEnabled(boolean enabled) {
        this.visionEnabled = enabled;
        if (!enabled) {
            visionCorrection = 0.0;
            filteredTxDeg = 0.0;
        }
    }

    // --- Core update loop ---

    /**
     * Call every loop iteration.  Reads the robot pose from the Follower,
     * computes the angle to the goal, and smoothly drives the turret servo
     * toward the target position.  If vision is enabled and a valid LLResult
     * is provided, a small correction is layered on top of the odometry target
     * to compensate for odometry drift.
     *
     * @param result LimeLight result for vision fine-tuning, or null if unavailable
     */
    public void update(LLResult result) {
        if (state == OdometryTurretState.DISABLED) {
            logTurret(0, 0);
            return;
        }

        // --- TRACKING ---

        // 1. Get robot pose from odometry
        Pose robotPose = follower.getPose();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();

        // 2. Compute turret world position accounting for mounting offset
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);
        double turretWorldX = robotX + TURRET_OFFSET_X * cosH - TURRET_OFFSET_Y * sinH;
        double turretWorldY = robotY + TURRET_OFFSET_X * sinH + TURRET_OFFSET_Y * cosH;

        // 3. Angle from turret to goal in world frame
        double angleToGoal = Math.atan2(goalY - turretWorldY, goalX - turretWorldX);

        // 4. Convert to turret-relative angle (how far the turret must rotate from
        //    the robot's forward direction to point at the goal)
        double turretRelativeAngle = normalizeAngle(angleToGoal - robotHeading - TURRET_FORWARD_OFFSET_RAD);

        // 5. Convert to a target servo position
        double rawTargetPos = TURRET_MIDPOINT + turretRelativeAngle * RAD_TO_SERVO;

        // 5b. Vision fine-tuning (only when enabled)
        if (visionEnabled && result != null && result.isValid()) {
            double txDeg = result.getTx();
            // EMA smooth the tx reading
            filteredTxDeg = visionAlpha * txDeg + (1.0 - visionAlpha) * filteredTxDeg;
            // Convert to a small servo correction and clamp it
            visionCorrection = clip(-visionKP * filteredTxDeg, -maxVisionCorrection, maxVisionCorrection);
        } else {
            // When vision is unavailable or disabled, let visionCorrection decay toward 0
            // so stale corrections don't persist forever.
            visionCorrection *= 0.95;
        }

        double correctedTargetPos = rawTargetPos + visionCorrection;

        // 6. Exponential moving average (EMA) smoothing on the target position
        filteredTargetPos = alpha * correctedTargetPos + (1.0 - alpha) * filteredTargetPos;

        // 7. Compute error and derivative
        double error = filteredTargetPos - turretPos;

        // Deadband: if we're close enough, don't move (prevents servo buzzing)
        double errorInRad = error / RAD_TO_SERVO; // convert back to radians for deadband check
        if (Math.abs(errorInRad) < deadbandRad) {
            previousError = error;
            logTurret(turretRelativeAngle, error);
            return;
        }

        // PD control
        double derivative = error - previousError;
        double delta = kP * error + kD * derivative;
        previousError = error;

        // 8. Clamp the step size to prevent jumps
        delta = clip(delta, -maxStepPerUpdate, maxStepPerUpdate);

        // 9. Apply and clamp to servo limits
        turretPos = clip(turretPos + delta, minPos, maxPos);
        turret.setPosition(turretPos);

        logTurret(turretRelativeAngle, error);
    }

    // --- Helpers ---

    private void setToDefault() {
        turret.setPosition(TURRET_MIDPOINT);
    }

    /**
     * Normalize an angle to the range [-pi, pi].
     */
    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private void logTurret(double relativeAngleRad, double error) {
        logger.logData("OdoTurret Angle", Math.toDegrees(relativeAngleRad));
        logger.logData("OdoTurret Error", error);
        logger.logData("OdoTurret Pos", turretPos);
        logger.logData("OdoTurret State", state);
        logger.logData("OdoTurret Vision Correction", visionCorrection);
        logger.logData("OdoTurret Vision TX", filteredTxDeg);
        logger.logData("OdoTurret Vision Enabled", visionEnabled);
    }

}
