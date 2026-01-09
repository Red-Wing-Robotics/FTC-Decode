package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.log.Logger;

abstract public class RWRBaseOpMode extends OpMode {

    public enum DiverterPosition {
        LEFT,
        RIGHT
    }

    public final Logger logger;

    public boolean isDiverterMoving = false;
    public double diverterTarget;

    public Servo diverter = null;

    public static double DIVERTER_LEFT = 0.02;

    public static double DIVERTER_RIGHT = 0.34;

    public DiverterPosition diverterPosition = DiverterPosition.LEFT;

    public void setupHardware () {
        diverter = hardwareMap.get( Servo.class, "diverter");
    }

    public RWRBaseOpMode () {
        super();
        this.logger = new Logger(this.telemetry);
    }

    public void toggleDiverter() {
//        if(!isDiverterMoving) {
//            diverterTarget = (diverter.getPosition() == DIVERTER_2) ? DIVERTER_1 : DIVERTER_2;
//            isDiverterMoving = true;
//            diverter.setPosition(diverterTarget);
//        }
    }

    public void updateDiverter () {
//        if (isDiverterMoving) {
//            if (diverter.getPosition() == diverterTarget) {
//                isDiverterMoving = false;
//            }
//        }
    }



}
