package org.firstinspires.ftc.teamcode.state;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

@Configurable
public class Diverter {

    public static double LEFT_POSITION = 0.02;
    public static double RIGHT_POSITION = 0.34;
    public static long DIVERTER_MOVE_TIME_MS = 500;

    public enum DiverterState {
        MOVING,
        NOT_MOVING
    }

    public enum DiverterPosition {
        LEFT,
        RIGHT,
        UNKNOWN
    }

    private final Servo diverter;
    private final Logger logger;

    public DiverterState state = DiverterState.NOT_MOVING;
    private DiverterPosition targetPosition = DiverterPosition.UNKNOWN;
    private DiverterPosition currentPosition = DiverterPosition.UNKNOWN;
    private long moveStartTime = 0;

    public Diverter(HardwareMap hardwareMap, Telemetry telemetry) {
        this.diverter = hardwareMap.get(Servo.class, "diverter");
        this.logger = new Logger(telemetry);
        // Set initial position without triggering a MOVING state, assuming it's done during init
        this.diverter.setPosition(LEFT_POSITION);
        this.currentPosition = DiverterPosition.LEFT;
        this.targetPosition = DiverterPosition.LEFT;
        this.state = DiverterState.NOT_MOVING;
    }

    public void update() {
        logger.logData("Diverter State", state.toString());
        logger.logData("Diverter Position", currentPosition.toString());

        if (state == DiverterState.MOVING) {
            if (System.currentTimeMillis() - moveStartTime >= DIVERTER_MOVE_TIME_MS) {
                state = DiverterState.NOT_MOVING;
                currentPosition = targetPosition;
                logger.logLine("Diverter reached position: " + currentPosition.toString());
            }
        }
    }

    public boolean isBusy() {
        return state == DiverterState.MOVING;
    }

    public void goToLeft() {
        if (targetPosition != DiverterPosition.LEFT) {
            targetPosition = DiverterPosition.LEFT;
            state = DiverterState.MOVING;
            diverter.setPosition(LEFT_POSITION);
            moveStartTime = System.currentTimeMillis();
            logger.logLine("Diverter moving to LEFT");
        }
    }

    public void goToRight() {
        if (targetPosition != DiverterPosition.RIGHT) {
            targetPosition = DiverterPosition.RIGHT;
            state = DiverterState.MOVING;
            diverter.setPosition(RIGHT_POSITION);
            moveStartTime = System.currentTimeMillis();
            logger.logLine("Diverter moving to RIGHT");
        }
    }

    public void toggleDiverter() {
        if (isBusy()) {
            logger.logLine("CANNOT toggle diverter, currently moving.");
            return;
        }

        if (currentPosition == DiverterPosition.LEFT) {
            goToRight();
        } else { // Handles RIGHT and UNKNOWN
            goToLeft();
        }
    }

    public DiverterPosition getCurrentPosition() {
        return currentPosition;
    }
}
