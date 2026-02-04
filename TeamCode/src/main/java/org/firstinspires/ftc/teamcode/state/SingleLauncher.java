package org.firstinspires.ftc.teamcode.state;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

import java.util.ArrayDeque;
import java.util.Queue;

@Configurable
public class SingleLauncher {

    public static double FEED_TIME_MS = 700; // in milliseconds
    public static double LAUNCHER_VELOCITY_PERCENT_FUDGE_FACTOR = 0.20; // Decimal percentage
    public static double LAUNCHER_RAMPING_TIME = 300;
    public static double TIME_BETWEEN_SHOTS_MS = 750; // in milliseconds
    public static double SPINDEXER_TURN_TIME_MS = 700; // in milliseconds
    public static double SPINDERXER_TURN_MAGNITUDE = 0.38;



    public static double FEEDER_SHOOT_POSITION = 0;
    public static double FEEDER_INDEX_POSITION = 0;

    public static double SPINDEXER_START = 0.04;

    private double targetVelocity;

    private static class ShotRequest {
        final SpindexerSlot slot;

        ShotRequest(SpindexerSlot slot) {
            this.slot = slot;
        }
    }

    public enum LauncherState {
        IDLE,
        RAMPING,
        READY,
        SHOOTING,
        WAITING,
        TURN_SPINDEXER,
        DONE
    }

    private enum SpindexerSlot {
        SHOOT,
        INTAKE,
        EXTRA
    }

    public final DcMotorEx shooter;
    public final DcMotor feeder;
    public final Spindexer spindexer;

    public LauncherState state = LauncherState.IDLE;
    private final Queue<ShotRequest> shotQueue;
    private long stateStartTime = 0;

    private final Logger logger;

    private final Turret turret;

    public SingleLauncher(HardwareMap hardwareMap, Telemetry telemetry, Turret turret) {
        this.logger = new Logger(telemetry);
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        feeder = hardwareMap.get(DcMotor.class, "feederMotor");
        spindexer = new Spindexer(hardwareMap,telemetry);
        this.turret = turret;
        this.shotQueue = new ArrayDeque<>();
    }

    public void update() {
        long currentTime = System.currentTimeMillis();
        logger.logData("Launcher State", this.state.toString());
        spindexer.update();
        if( spindexer.state == Spindexer.SpindexerState.NOT_MOVING && feeder.getPower() == 1.0){
            deactivateFeeders();
        }

        switch(this.state) {
            case IDLE:
                // Waiting for startShooter() to be called
                break;
            case RAMPING:
                if(System.currentTimeMillis() - stateStartTime >= LAUNCHER_RAMPING_TIME) {//isShooterWithinFudgeFactor()
                    this.state = LauncherState.READY;
                    logger.logLine("Flywheel at speed, ready to shoot.");
                }
                break;
            case READY:
                if (!shotQueue.isEmpty()) {
                    ShotRequest currentShot = shotQueue.peek();
                    if (currentShot != null && currentShot.slot != SpindexerSlot.SHOOT) {
                        this.state = LauncherState.TURN_SPINDEXER;
                        stateStartTime = currentTime;
                        turnSpindexer( currentShot.slot );
                        logger.logLine("Starting intake for next shot.");
                    } else if (currentShot != null) {
                        // Skip intake and go straight to shooting
                        this.state = LauncherState.SHOOTING;
                        stateStartTime = currentTime;
                        activateFeeder();
                        logger.logLine("Shooting");
                    }
                }
                break;
            case TURN_SPINDEXER:
                if (currentTime - stateStartTime >= SPINDEXER_TURN_TIME_MS) {
                    ShotRequest currentShot = shotQueue.peek();
                    if (currentShot != null) {
                        this.state = LauncherState.SHOOTING;
                        stateStartTime = currentTime;
                        activateFeeder();
                        logger.logLine("Turned spindexer, now shooting ");
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
            case DONE:
                if (!shotQueue.isEmpty()) {
                    this.state = LauncherState.READY;
                }
                break;
        }
    }

    public boolean isShooterWithinFudgeFactor() {
        double currentVelocity = shooter.getVelocity();
        // Handle case where target velocity is 0
        if (this.targetVelocity == 0) {
            return Math.abs(currentVelocity) < (LAUNCHER_VELOCITY_PERCENT_FUDGE_FACTOR * 100); // just a small number
        }
        return Math.abs(currentVelocity - this.targetVelocity) < (LAUNCHER_VELOCITY_PERCENT_FUDGE_FACTOR * this.targetVelocity);
    }

    public boolean isBusy() {
        return this.state == LauncherState.SHOOTING || this.state == LauncherState.WAITING || this.state == LauncherState.TURN_SPINDEXER || this.state == LauncherState.RAMPING || this.state == LauncherState.READY;
    }

    public void startShooter(double velocity) {
        if (this.state == LauncherState.IDLE || this.state == LauncherState.RAMPING || this.state == LauncherState.READY) {
            if (this.targetVelocity != velocity) {
                setShooterVelocity(velocity);
                if( this.state == LauncherState.IDLE) {
                    this.state = LauncherState.RAMPING;
                }else{
                    this.state = LauncherState.READY;
                }
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
        shotQueue.clear();
        this.state = LauncherState.IDLE;
        logger.logLine("Shooter stopped and returned to idle.");
    }

    public void shootShoot() {
        if (state != LauncherState.IDLE) {
            shootTurret();
            this.shotQueue.add(new ShotRequest(SpindexerSlot.SHOOT));
            logger.logLine("Queued RIGHT shot. Intake: " + false);
        } else {
            logger.logLine("Cannot queue shot, launcher is idle.");
        }
    }

    public void shootExtra() {
        if (state != LauncherState.IDLE) {
            shootTurret();
            this.shotQueue.add(new ShotRequest(SpindexerSlot.EXTRA));
            logger.logLine("Queued RIGHT shot. Intake: " + false);
        } else {
            logger.logLine("Cannot queue shot, launcher is idle.");
        }
    }

    public void shootIntake() {
        if (state != LauncherState.IDLE) {
            shootTurret();
            this.shotQueue.add(new ShotRequest(SpindexerSlot.INTAKE));
            logger.logLine("Queued RIGHT shot. Intake: " + false);
        } else {
            logger.logLine("Cannot queue shot, launcher is idle.");
        }
    }
/*
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
    }*/

    public void activateFeeder() {
        feeder.setPower(-1.0);
    }

    public void deactivateFeeders() {
        feeder.setPower(0.0);
    }

    public void reverseFeeder(){
        feeder.setPower(1.0);
    }

    private void setShooterVelocity(double p){
        this.targetVelocity = p;
        shooter.setVelocity(-1 * p);
    }

    public void turnSpindexerCounterClockwise(){
        spindexer.moveCounterClockwise();
        reverseFeeder();
    }

    public void turnSpindexerClockwise(){
        spindexer.moveClockwise();
        reverseFeeder();
    }

    public void initializeSpindexer(){
        spindexer.initialize();
    }

    private void turnSpindexer( SpindexerSlot slot ){
        switch (slot) {
            case EXTRA:
                spindexer.moveCounterClockwise();
                break;
            case INTAKE:
                spindexer.moveClockwise();
                break;
        }
    }

    public double getShooterVelocity(){
        return shooter.getVelocity();
    }

//    public double getSpindexerPosition(){
//        return spindexer.getPosition();
//    }

    public LauncherState getState(){
        return this.state;
    }

    public void setToDone(){
        this.state = LauncherState.DONE;
    }

    public void switchSpindexerMode(){
        spindexer.switchMode();
    }

    private void shootTurret() {
        if(turret != null) {
            turret.shoot();
        }
    }

}
