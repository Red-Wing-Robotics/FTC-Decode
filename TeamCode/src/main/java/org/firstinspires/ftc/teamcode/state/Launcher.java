package org.firstinspires.ftc.teamcode.state;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

import java.util.ArrayDeque;
import java.util.Queue;

public class Launcher {

    public static double FEED_TIME_MS = 200; // in milliseconds
    public static double LAUNCHER_VELOCITY_PERCENT_FUDGE_FACTOR = 0.10; // Decimal percentage
    public static double TIME_BETWEEN_SHOTS_MS = 2000; // in milliseconds
    public static double INTAKE_TIME_MS = 1000; // in milliseconds

    private double targetVelocity;

    private static class ShotRequest {
        final ShooterSide side;
        final boolean shouldActivateIntake;

        ShotRequest(ShooterSide side, boolean shouldActivateIntake) {
            this.side = side;
            this.shouldActivateIntake = shouldActivateIntake;
        }
    }

    public enum LauncherState {
        IDLE,
        RAMPING,
        READY,
        INTAKING,
        SHOOTING,
        WAITING
    }

    private enum ShooterSide {
        LEFT,
        RIGHT
    }

    public final DcMotorEx leftShooter;
    public final DcMotorEx rightShooter;
    public final CRServo rightFeeder;
    public final CRServo leftFeeder;
    public final DcMotor intake;

    public LauncherState state = LauncherState.IDLE;
    private final Queue<ShotRequest> shotQueue;
    private long stateStartTime = 0;

    private final Logger logger;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
        this.logger = new Logger(telemetry);
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        intake = hardwareMap.get(DcMotor.class, "intake");

        this.shotQueue = new ArrayDeque<>();
    }

    public void update() {
        long currentTime = System.currentTimeMillis();
        logger.logData("Launcher State", this.state.toString());

        switch(this.state) {
            case IDLE:
                // Waiting for startShooter() to be called
                break;
            case RAMPING:
                if(isShooterWithinFudgeFactor()) {
                    this.state = LauncherState.READY;
                    logger.logLine("Flywheel at speed, ready to shoot.");
                }
                break;
            case READY:
                if (!shotQueue.isEmpty()) {
                    ShotRequest currentShot = shotQueue.peek();
                    if (currentShot != null && currentShot.shouldActivateIntake) {
                        this.state = LauncherState.INTAKING;
                        stateStartTime = currentTime;
                        activateIntake();
                        logger.logLine("Starting intake for next shot.");
                    } else if (currentShot != null) {
                        // Skip intake and go straight to shooting
                        this.state = LauncherState.SHOOTING;
                        stateStartTime = currentTime;
                        activateFeeder(currentShot.side);
                        logger.logLine("Shooting " + currentShot.side);
                    }
                }
                break;
            case INTAKING:
                if (currentTime - stateStartTime >= INTAKE_TIME_MS) {
                    deactivateIntake();
                    ShotRequest currentShot = shotQueue.peek();
                    if (currentShot != null) {
                        this.state = LauncherState.SHOOTING;
                        stateStartTime = currentTime;
                        activateFeeder(currentShot.side);
                        logger.logLine("Finished intake, now shooting " + currentShot.side);
                    } else {
                        this.state = LauncherState.IDLE;
                        logger.logLine("Error: Intake finished but shot queue is empty.");
                    }
                }
                break;
            case SHOOTING:
                if (currentTime - stateStartTime >= FEED_TIME_MS) {
                    deactivateFeeders();
                    shotQueue.poll(); // remove shot from queue
                    logger.logLine("Finished shot.");
                    if (shotQueue.isEmpty()) {
                        this.state = LauncherState.READY;
                        logger.logLine("Shot queue empty, returning to ready.");
                    } else {
                        this.state = LauncherState.WAITING;
                        stateStartTime = currentTime;
                        logger.logLine("Waiting between shots.");
                    }
                }
                break;
            case WAITING:
                if (currentTime - stateStartTime >= TIME_BETWEEN_SHOTS_MS) {
                    this.state = LauncherState.READY;
                    logger.logLine("Finished waiting, ready for next shot.");
                }
                break;
        }
    }

    public boolean isShooterWithinFudgeFactor() {
        double currentVelocity = rightShooter.getVelocity();
        // Handle case where target velocity is 0
        if (this.targetVelocity == 0) {
            return Math.abs(currentVelocity) < (LAUNCHER_VELOCITY_PERCENT_FUDGE_FACTOR * 100); // just a small number
        }
        return Math.abs(currentVelocity - this.targetVelocity) < (LAUNCHER_VELOCITY_PERCENT_FUDGE_FACTOR * this.targetVelocity);
    }

    public boolean isBusy() {
        return this.state == LauncherState.SHOOTING || this.state == LauncherState.WAITING || this.state == LauncherState.INTAKING;
    }

    public void startShooter(double velocity) {
        if (this.state == LauncherState.IDLE || this.state == LauncherState.RAMPING || this.state == LauncherState.READY) {
            if (this.targetVelocity != velocity) {
                setShooterVelocity(velocity);
                this.state = LauncherState.RAMPING;
                stateStartTime = System.currentTimeMillis();
                logger.logLine("Starting/adjusting flywheel speed to " + velocity);
            }
        } else {
            logger.logLine("CANNOT start/adjust shooter, currently shooting or intaking.");
        }
    }

    public void stopShooter() {
        setShooterVelocity(0);
        deactivateFeeders();
        deactivateIntake();
        shotQueue.clear();
        this.state = LauncherState.IDLE;
        logger.logLine("Shooter stopped and returned to idle.");
    }

    public void shootRight(boolean shouldActivateIntake) {
        if (state != LauncherState.IDLE) {
            this.shotQueue.add(new ShotRequest(ShooterSide.RIGHT, shouldActivateIntake));
            logger.logLine("Queued RIGHT shot. Intake: " + shouldActivateIntake);
        } else {
            logger.logLine("Cannot queue shot, launcher is idle.");
        }
    }

    public void shootRight() {
        this.shootRight(false);
    }

    public void shootLeft(boolean shouldActivateIntake) {
        if (state != LauncherState.IDLE) {
            this.shotQueue.add(new ShotRequest(ShooterSide.LEFT, shouldActivateIntake));
            logger.logLine("Queued LEFT shot. Intake: " + shouldActivateIntake);
        } else {
            logger.logLine("Cannot queue shot, launcher is idle.");
        }
    }

    public void shootLeft() {
        this.shootLeft(false);
    }

    private void activateFeeder(ShooterSide side) {
        switch (side) {
            case LEFT:
                leftFeeder.setPower(1);
                break;
            case RIGHT:
                rightFeeder.setPower(1);
                break;
        }
    }

    private void deactivateFeeders() {
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    private void activateIntake() {
        intake.setPower(-1.0);
    }

    private void deactivateIntake() {
        intake.setPower(0);
    }

    private void setShooterVelocity(double p){
        this.targetVelocity = p;
        rightShooter.setVelocity(-1 * p);
        leftShooter.setVelocity(p);
    }

}
