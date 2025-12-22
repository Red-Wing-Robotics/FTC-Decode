package org.firstinspires.ftc.teamcode.util.log;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Logger {

    private final Telemetry telemetry;
    private final TelemetryManager telemetryM;

    public Logger(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void logLine(String line) {
        telemetry.addLine(line);
        telemetryM.addLine(line);
    }

    public void logData(String caption, Object value) {
        telemetry.addData(caption, value);
        telemetryM.addData(caption, value);
    }

    public void update() {
        telemetry.update();
        telemetryM.update();
    }

}
