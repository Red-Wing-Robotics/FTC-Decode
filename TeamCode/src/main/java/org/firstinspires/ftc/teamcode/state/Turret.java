package org.firstinspires.ftc.teamcode.state;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

public class Turret {

    public static double TURRET_MIDPOINT = 0.5;
    public static double FUDGE_FACTOR = 1.0;

    public static double FACTOR_ADJUSTMENT = 250;

    public final Servo turret;
    private final Logger logger;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry){
        this.logger = new Logger(telemetry);
        turret = hardwareMap.get(Servo.class, "turret");
    }

    public void update() {
        setToDefault();
    }

    public void update(LLResult result) {
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            logger.logData("TX", tx);
            logger.logData("Turret Pos", turret.getPosition());
            if(Math.abs(tx) < FUDGE_FACTOR) {
                return;
            }
            double currentPosition = turret.getPosition();
            turret.setPosition(currentPosition + (tx / FACTOR_ADJUSTMENT));
        } else {
            setToDefault();
        }
    }

    private void setToDefault() {
        turret.setPosition(TURRET_MIDPOINT);
    }

}
