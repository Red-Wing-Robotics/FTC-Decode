package org.firstinspires.ftc.teamcode.state;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

public class Spindexer {

    public static double SPINDERXER_POSITION_1 = 0.04;
    public static double SPINDERXER_POSITION_2 = 0.42;
    public static double SPINDERXER_POSITION_3 = 0.8;
    public static long SPINDEXER_MOVE_TIME_MS = 500;

    public enum SpindexerPosition{
        SHOOT_1,
        SHOOT_2,
        SHOOT_3
    }

    public enum SpindexerState{
        MOVING,
        NOT_MOVING
    }


    public final Servo spindexer;
    public SpindexerState state = SpindexerState.NOT_MOVING;
    public SpindexerPosition position = SpindexerPosition.SHOOT_1;
    private long moveStartTime = 0;
    private final Logger logger;
    public Spindexer(HardwareMap hardwareMap, Telemetry telemetry){
        this.logger = new Logger(telemetry);
        spindexer = hardwareMap.get(Servo.class, "spindexer");
    }

    public void update(){
        if (state == SpindexerState.MOVING){
            if (System.currentTimeMillis() - moveStartTime >= SPINDEXER_MOVE_TIME_MS) {
                state = SpindexerState.NOT_MOVING;
                //logger.logLine("Diverter reached position: " + currentPosition.toString());
            }
        }
    }

    public void moveClockwise(){
        if( state == SpindexerState.NOT_MOVING) {
            state = SpindexerState.MOVING;
            moveStartTime = System.currentTimeMillis();
            switch (position) {
                case SHOOT_1:
                    spindexer.setPosition(SPINDERXER_POSITION_3);
                    position = SpindexerPosition.SHOOT_3;
                case SHOOT_2:
                    spindexer.setPosition(SPINDERXER_POSITION_1);
                    position = SpindexerPosition.SHOOT_1;
                case SHOOT_3:
                    spindexer.setPosition(SPINDERXER_POSITION_2);
                    position = SpindexerPosition.SHOOT_2;
            }
        }
    }

    public void moveCounterClockwise(){
        if( state == SpindexerState.NOT_MOVING) {
            state = SpindexerState.MOVING;
            moveStartTime = System.currentTimeMillis();
            switch (position) {
                case SHOOT_1:
                    spindexer.setPosition(SPINDERXER_POSITION_2);
                    position = SpindexerPosition.SHOOT_2;
                case SHOOT_2:
                    spindexer.setPosition(SPINDERXER_POSITION_3);
                    position = SpindexerPosition.SHOOT_3;
                case SHOOT_3:
                    spindexer.setPosition(SPINDERXER_POSITION_1);
                    position = SpindexerPosition.SHOOT_1;
            }
        }
    }

    public void initialize(){
        state = SpindexerState.MOVING;
        moveStartTime = System.currentTimeMillis();
        spindexer.setPosition( SPINDERXER_POSITION_1);
        position = SpindexerPosition.SHOOT_1;
    }

    public double getPosition(){
        return spindexer.getPosition();
    }

}
