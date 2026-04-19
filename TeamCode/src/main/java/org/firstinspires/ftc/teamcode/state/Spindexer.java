package org.firstinspires.ftc.teamcode.state;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    public enum SpindexerPosition {
        SLOT_1,
        SLOT_2,
        SLOT_3
    }

    public enum SpindexerState {
        MOVING,
        NOT_MOVING
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
    private final Logger logger;

    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry) {
        this.logger = new Logger(telemetry);
        CRServo servo = hardwareMap.get(CRServo.class, "spindexer");
        servo.setDirection(DcMotorSimple.Direction.REVERSE);
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "axonEncoder");
        spindexer = new RTPAxon(servo, encoder);
        spindexer.setMaxPower(MAX_POWER);
        spindexer.setPidCoeffs(KP, KI, KD);
    }

    public void update() {
        spindexer.update();

        if (state == SpindexerState.MOVING && spindexer.isAtTarget()) {
            state = SpindexerState.NOT_MOVING;
        }
    }

    /**
     * Advance one slot in the clockwise direction (positive rotation).
     * Always rotates forward — never backtracks.
     */
    public void moveClockwise() {
        if (state == SpindexerState.NOT_MOVING) {
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
            slotIndex--;
            updatePositionEnum();
            goToCurrentSlot();
        }
    }

    public void initialize() {
        slotIndex = 0;
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
        goToCurrentSlot();
    }

    public String log() {
        return spindexer.log();
    }

    private void goToCurrentSlot() {
        double offset = (mode == SpindexerMode.SHOOTING) ? SHOOT_OFFSET_DEGREES : INTAKE_OFFSET_DEGREES;
        double targetDegrees = slotIndex * SLOT_SPACING_DEGREES + offset;
        spindexer.setTargetRotation(targetDegrees);
        state = SpindexerState.MOVING;
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
