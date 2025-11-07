package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.log.LoggingTarget;

abstract public class RWRBaseOpMode extends OpMode {
    private TelemetryManager telemetryM;

    public TelemetryManager getPanelsTelemetry() {
        if(telemetryM == null) {
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        }
        return telemetryM;
    }

    public void log(String caption, Object value) {
        telemetry.addData(caption, value );
        getPanelsTelemetry().addData(caption, value);
    }

    public void log(LoggingTarget target, String caption, Object value){
        if (target == LoggingTarget.PANELS || target == LoggingTarget.ALL) {
            getPanelsTelemetry().addData(caption, value);
        }
        if (target == LoggingTarget.DRIVER_HUB || target == LoggingTarget.ALL){
            telemetry.addData(caption, value );
        }
    }
}
