package org.firstinspires.ftc.teamcode.state;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

public class Turret {

    public static double TURRET_MIDPOINT = 0.5;
    public static double ANGLE_MAX = 29.0;
    public final Servo turret;
    private final Logger logger;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.logger = new Logger(telemetry);
        turret = hardwareMap.get(Servo.class, "turret");
        setToDefault();
    }

    public void update() {
        setToDefault();
    }

    public void update(LLResult result) {
        if (result != null && result.isValid()) {
            double tx = -(result.getTx());
            logger.logData("TX", tx);
            logger.logData("Turret Pos", turret.getPosition());
            turret.setPosition(getTargetPosition(tx));
        } else {
            setToDefault();
        }
    }

    private void setToDefault() {
        logger.logData("TX", "Out of View");
        logger.logData("Turret Pos", turret.getPosition());
        turret.setPosition(TURRET_MIDPOINT);
    }

    private double getTargetPosition(double tx) {
        if(Math.abs(tx) > ANGLE_MAX) {
            return TURRET_MIDPOINT;
        }
        return tx + ANGLE_MAX / (ANGLE_MAX * 2);
    }

}
