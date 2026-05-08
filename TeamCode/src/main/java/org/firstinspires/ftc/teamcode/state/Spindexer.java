package org.firstinspires.ftc.teamcode.state;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.RTPAxon;
import org.firstinspires.ftc.teamcode.util.log.Logger;

@Configurable
public class Spindexer {

    // Degrees between each of the 3 slots (120 degrees apart on a full rotation)
    public static double SLOT_SPACING_DEGREES = 120.0;

    // Offset in degrees from slot center for shooting vs intaking positions.
    // These let you fine-tune the exact stopping angle within each slot.
    public static double SHOOT_OFFSET_DEGREES = 0.0;
    public static double INTAKE_OFFSET_DEGREES = 10.0;

    // Tuned in TestSpindexer. Update here to propagate to all opmodes.
    public static double MAX_POWER = 1.0;
    public static double KP = 0.012;
    public static double KI = 0.0;
    public static double KD = 0.0003;

    // If a move takes longer than this (in seconds), the spindexer is considered
    // jammed and will revert to the previous slot it was at.
    public static double moveTimeoutSeconds = 0.9;

    public static double AUTO_MOVE_TIMEOUT = 10.0;
    public static double TELEOP_MOVE_TIMEOUT = 0.9;


    private int fullRotationTurns = 0;

    public enum SpindexerPosition {
        SLOT_1,
        SLOT_2,
        SLOT_3
    }

    public enum SpindexerState {
        MOVING,
        NOT_MOVING,
        FULL_ROTATION
    }

    public enum SpindexerMode {
        SHOOTING,
        INTAKING
    }

    public final RTPAxon spindexer;

    public SpindexerState state = SpindexerState.NOT_MOVING;
    public SpindexerPosition position = SpindexerPosition.SLOT_1;
    public SpindexerMode mode = SpindexerMode.INTAKING;
    private int slotIndex = 0;
    // Slot index we were at before the current move started. Used to revert
    // on jam timeout.
    private int previousSlotIndex = 0;
    // Tracks whether the current move is itself a recovery from a jam, so we
    // don't ping-pong forever if the revert also fails.
    private boolean isRecovering = false;
    private final ElapsedTime moveTimer = new ElapsedTime();
    private final Logger logger;

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry, double moveTimeout) {
        this.logger = new Logger(telemetry);
        CRServo servo = hardwareMap.get(CRServo.class, "spindexer");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "axonEncoder");
        spindexer = new RTPAxon(servo, encoder);
        spindexer.setMaxPower(MAX_POWER);
        spindexer.setPidCoeffs(KP, KI, KD);
        moveTimeoutSeconds = moveTimeout;
    }

    public void update() {
        spindexer.update();

        if (state == SpindexerState.MOVING) {
            if (spindexer.isAtTarget()) {
                state = SpindexerState.NOT_MOVING;
                isRecovering = false;
            } else if (moveTimer.seconds() > moveTimeoutSeconds) {
                handleJamTimeout();
            }
        } else if (state == SpindexerState.FULL_ROTATION) {
            if (spindexer.isAtTarget()) {
                if (fullRotationTurns == 3) {
                    state = SpindexerState.NOT_MOVING;
                    fullRotationTurns = 0;
                    isRecovering = false;
                } else {
                    previousSlotIndex = slotIndex;
                    slotIndex++;
                    updatePositionEnum();
                    goToCurrentSlot();
                    fullRotationTurns++;
                }
            } else if (moveTimer.seconds() > moveTimeoutSeconds) {
                // Jammed mid full-rotation; abort and revert to the slot we
                // were at before the current sub-move started.
                fullRotationTurns = 0;
                handleJamTimeout();
            }
        }
    }

    /**
     * Called when the spindexer fails to reach its target within
     * {@link #moveTimeoutSeconds}. Reverts to the slot we were at before
     * the current move. If we're already trying to recover (i.e. the revert
     * also jammed), give up and stop so we don't oscillate forever.
     */
    private void handleJamTimeout() {
        if (isRecovering) {
            // The revert move itself timed out — stop trying.
            state = SpindexerState.NOT_MOVING;
            isRecovering = false;
            spindexer.setTargetRotation(spindexer.getTotalRotation());
            return;
        }

        slotIndex = previousSlotIndex;
        updatePositionEnum();
        isRecovering = true;
        // Force a MOVING state for the revert even if we were in FULL_ROTATION.
        state = SpindexerState.NOT_MOVING;
        goToCurrentSlot();
    }

    /**
     * Advance one slot in the clockwise direction (positive rotation).
     * Always rotates forward — never backtracks.
     */
    public void moveClockwise() {
        if (state == SpindexerState.NOT_MOVING) {
            previousSlotIndex = slotIndex;
            isRecovering = false;
            slotIndex++;
            updatePositionEnum();
            goToCurrentSlot();
        }
    }

    /**
     * Advance one slot in the counter-clockwise direction (negative rotation).
     * Always rotates backward — never backtracks.
     */
    public void moveCounterClockwise() {
        if (state == SpindexerState.NOT_MOVING) {
            previousSlotIndex = slotIndex;
            isRecovering = false;
            slotIndex--;
            updatePositionEnum();
            goToCurrentSlot();
        }
    }

    public void moveFulRotation(){
        if(state == SpindexerState.NOT_MOVING){
            previousSlotIndex = slotIndex;
            isRecovering = false;
            state = SpindexerState.FULL_ROTATION;
            moveTimer.reset();
        }
    }

    public void initialize() {
        slotIndex = 0;
        previousSlotIndex = 0;
        isRecovering = false;
        position = SpindexerPosition.SLOT_1;
        spindexer.forceResetTotalRotation();
        goToCurrentSlot();
    }

    public double getPosition() {
        return spindexer.getTotalRotation();
    }

    public void switchMode() {
        if (mode == SpindexerMode.INTAKING) {
            mode = SpindexerMode.SHOOTING;
        } else {
            mode = SpindexerMode.INTAKING;
        }
        // switchMode only changes the offset within the same slot, so a
        // jam-revert here should snap back to the same slot index, not the
        // pre-switch one. Refresh previousSlotIndex accordingly.
        previousSlotIndex = slotIndex;
        isRecovering = false;
        goToCurrentSlot();
    }

    public String log() {
        return spindexer.log();
    }

    private void goToCurrentSlot() {
        double offset = (mode == SpindexerMode.SHOOTING) ? SHOOT_OFFSET_DEGREES : INTAKE_OFFSET_DEGREES;
        double targetDegrees = slotIndex * SLOT_SPACING_DEGREES + offset;
        spindexer.setTargetRotation(targetDegrees);
        if(state != SpindexerState.FULL_ROTATION) {
            state = SpindexerState.MOVING;
        }
        moveTimer.reset();
    }

    private void updatePositionEnum() {
        int mod = ((slotIndex % 3) + 3) % 3;
        switch (mod) {
            case 0: position = SpindexerPosition.SLOT_1; break;
            case 1: position = SpindexerPosition.SLOT_2; break;
            case 2: position = SpindexerPosition.SLOT_3; break;
        }
    }
}
