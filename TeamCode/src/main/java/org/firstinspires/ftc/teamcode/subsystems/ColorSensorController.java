package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorController {

    NormalizedColorSensor color1;
    NormalizedColorSensor color2;
    Telemetry telemetry;

    // Threshold values based on observed sensor readings (gain=4)
    // PURPLE: R=0.0693, G=0.1175, B=0.1141
    // GREEN:  R=0.0676, G=0.1170, B=0.1113
    // NONE:   R=0.0650, G=0.1115, B=0.1066

    // PURPLE has highest red (0.0693) - threshold between PURPLE and GREEN
    private static final float PURPLE_RED_THRESHOLD = 0.0685f;

    // PURPLE has highest blue (0.1141) - threshold between PURPLE and GREEN
    private static final float PURPLE_BLUE_THRESHOLD = 0.1127f;

    // GREEN/PURPLE have higher green than NONE - threshold to separate from NONE
    private static final float GREEN_MIN_THRESHOLD = 0.1140f;

    public ColorSensorController(HardwareMap hardwareMap, Telemetry telemetry) {
        this.color1 = hardwareMap.get(NormalizedColorSensor.class, "color1");
        this.color1.setGain(4);
        this.color2 = hardwareMap.get(NormalizedColorSensor.class, "color2");
        this.color2.setGain(4);
        if (color1 instanceof SwitchableLight) {
            ((SwitchableLight)this.color1).enableLight(true);
        }
        if (color2 instanceof SwitchableLight) {
            ((SwitchableLight)this.color2).enableLight(true);
        }
        this.telemetry = telemetry;
    }

    public ColorSensorState getColor() {
        ColorSensorState detColor1 = getDetectedColor(color1);
        //ColorSensorState detColor2 = getDetectedColor(color2);
        return detColor1;
    }

    private ColorSensorState getDetectedColor (NormalizedColorSensor sensor) {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;
        telemetry.addData("Norm Red", normRed);
        telemetry.addData("Norm Green", normGreen);
        telemetry.addData("Norm Blue", normBlue);

        ColorSensorState detected = classifyColor(normRed, normGreen, normBlue);
        telemetry.addData("Detected Color", detected.toString());

        return detected;
    }

    /**
     * Classifies the color based on normalized RGB values.
     *
     * Classification logic:
     * - PURPLE: Has highest red (>= 0.0685) AND highest blue (>= 0.1127)
     * - GREEN: Has green >= 0.1140 (but not PURPLE's high red/blue)
     * - NONE: Everything else (lowest values across the board)
     */
    private ColorSensorState classifyColor(float normRed, float normGreen, float normBlue) {
        // Check for PURPLE first - it has the highest red AND blue values
        if (normRed >= PURPLE_RED_THRESHOLD && normBlue >= PURPLE_BLUE_THRESHOLD) {
            return ColorSensorState.PURPLE;
        }

        // Check for GREEN - has higher green than NONE
        if (normGreen >= GREEN_MIN_THRESHOLD) {
            return ColorSensorState.GREEN;
        }

        // Default to NONE
        return ColorSensorState.NONE;
    }

    public boolean isInstaceOfSwitchableLight(){
        return color1 instanceof SwitchableLight;
    }

    public void enableLight(boolean b){//not sure what b does
        ((SwitchableLight)color1).enableLight(b);
        ((SwitchableLight)color2).enableLight(b);
    }

}
