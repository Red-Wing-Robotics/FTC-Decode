package org.firstinspires.ftc.teamcode.state;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DualColorSensorStateMachine {

    Telemetry telemetry;
    RevColorSensorV3 color1;
    RevColorSensorV3 color2;

    public static float DISTANCE_THRESHOLD = 4.0f;
    public static float SENSOR_GAIN = 10.0f;
    public static float EMA_ALPHA = 0.2f;

    private double smoothedDistance1 = 10.0f;
    private double smoothedDistance2 = 10.0f;

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
        return (smoothedDistance1 >= 0 && smoothedDistance1 < DISTANCE_THRESHOLD)
                || (smoothedDistance2 >= 0 && smoothedDistance2 < DISTANCE_THRESHOLD);
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
        float hue1 = JavaUtil.colorToHue(colors1.toColor());
        NormalizedRGBA colors2 = color2.getNormalizedColors();
        float hue2 = JavaUtil.colorToHue(colors2.toColor());

        telemetry.addData("Hue 1", hue1);
        telemetry.addData("Hue 2", hue2);
        telemetry.addData("Smoothed Distance 1", smoothedDistance1);
        telemetry.addData("Smoothed Distance 2", smoothedDistance2);
        telemetry.addData("Intake Filled", isIntakeFilled());
    }
}
