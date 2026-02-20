package org.firstinspires.ftc.teamcode.state;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorState;

public class DualColorSensorStateMachine {

    Telemetry telemetry;
    RevColorSensorV3 color1;
    RevColorSensorV3 color2;

    public static float DISTANCE_THRESHOLD = 0.74f;
    public static float SENSOR_GAIN = 10.0f;
    public static float EMA_ALPHA = 0.9f;

    private double smoothedDistance1 = 0;
    private double smoothedDistance2 = 0;

    private double smoothedHue1 = -1.0;
    private double smoothedHue2 = -1.0;

    public DualColorSensorStateMachine(HardwareMap hardwareMap, Telemetry telemetry) {
        this.color1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        configureSensor(color1);
        this.color2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        configureSensor(color2);
        this.telemetry = telemetry;
    }

    private void configureSensor(RevColorSensorV3 sensor) {
        sensor.setGain(SENSOR_GAIN);
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight)sensor).enableLight(true);
        }
    }

    public boolean isIntakeFilled() {
        return smoothedDistance2 < DISTANCE_THRESHOLD;
    }

    public ColorSensorState getIntakeColor() {
        if(!isIntakeFilled()) {
            return ColorSensorState.NONE;
        }
        if(smoothedHue2 > 182.0) {
            return ColorSensorState.PURPLE;
        }
        else {
            return ColorSensorState.GREEN;
        }
    }

    public void update() {
        double rawDistance1 = color1.getDistance(DistanceUnit.INCH);
        double rawDistance2 = color2.getDistance(DistanceUnit.INCH);

        smoothedDistance1 = smoothedDistance1 < 0
                ? rawDistance1
                : EMA_ALPHA * rawDistance1 + (1.0 - EMA_ALPHA) * smoothedDistance1;

        smoothedDistance2 = smoothedDistance2 < 0
                ? rawDistance2
                : EMA_ALPHA * rawDistance2 + (1.0 - EMA_ALPHA) * smoothedDistance2;

        NormalizedRGBA colors1 = color1.getNormalizedColors();
        float rawHue1 = JavaUtil.colorToHue(colors1.toColor());
        NormalizedRGBA colors2 = color2.getNormalizedColors();
        float rawHue2 = JavaUtil.colorToHue(colors2.toColor());

        smoothedHue1 = smoothedHue1 < 0
                ? rawHue1
                : EMA_ALPHA * rawHue1 + (1.0 - EMA_ALPHA) * smoothedHue1;

        smoothedHue2 = smoothedHue2 < 0
                ? rawHue2
                : EMA_ALPHA * rawHue2 + (1.0 - EMA_ALPHA) * smoothedHue2;

        telemetry.addData("Smoothed Hue 1", smoothedHue1);
        telemetry.addData("Smoothed Hue 2", smoothedHue2);
        telemetry.addData("Smoothed Distance 1", smoothedDistance1);
        telemetry.addData("Smoothed Distance 2", smoothedDistance2);
        telemetry.addData("Intake Filled", isIntakeFilled());
    }
}
