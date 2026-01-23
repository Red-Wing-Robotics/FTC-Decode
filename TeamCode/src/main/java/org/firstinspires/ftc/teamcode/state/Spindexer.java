package org.firstinspires.ftc.teamcode.state;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.log.Logger;

@Configurable
public class Spindexer {

    public static double SPINDERXER_POSITION_SHOOT_1 = 0.04;
    public static double SPINDERXER_POSITION_SHOOT_2 = 0.42;
    public static double SPINDERXER_POSITION_SHOOT_3 = 0.8;
    public static double SPINDERXER_POSITION_INTAKE_1 = 0.93;
    public static double SPINDERXER_POSITION_INTAKE_2 = 0.17;
    public static double SPINDERXER_POSITION_INTAKE_3 = 0.55;
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

    public enum SpindexerMode{
        SHOOTING,
        INTAKING
    }


    public final Servo spindexer;

    public SpindexerState state = SpindexerState.NOT_MOVING;
    public SpindexerPosition position = SpindexerPosition.SHOOT_1;
    public SpindexerMode mode = SpindexerMode.INTAKING;
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
            if(mode == SpindexerMode.SHOOTING) {
                switch (position) {
                    case SHOOT_1:
                        spindexer.setPosition(SPINDERXER_POSITION_SHOOT_3);
                        position = SpindexerPosition.SHOOT_3;
                        break;
                    case SHOOT_2:
                        spindexer.setPosition(SPINDERXER_POSITION_SHOOT_1);
                        position = SpindexerPosition.SHOOT_1;
                        break;
                    case SHOOT_3:
                        spindexer.setPosition(SPINDERXER_POSITION_SHOOT_2);
                        position = SpindexerPosition.SHOOT_2;
                        break;
                }
            }else {
                switch (position) {
                    case SHOOT_1:
                        spindexer.setPosition(SPINDERXER_POSITION_INTAKE_3);
                        position = SpindexerPosition.SHOOT_3;
                        break;
                    case SHOOT_2:
                        spindexer.setPosition(SPINDERXER_POSITION_INTAKE_1);
                        position = SpindexerPosition.SHOOT_1;
                        break;
                    case SHOOT_3:
                        spindexer.setPosition(SPINDERXER_POSITION_INTAKE_2);
                        position = SpindexerPosition.SHOOT_2;
                        break;
                }
            }
        }
    }

    public void moveCounterClockwise(){
        if( state == SpindexerState.NOT_MOVING) {
            state = SpindexerState.MOVING;
            moveStartTime = System.currentTimeMillis();
            if (mode == SpindexerMode.SHOOTING) {
                switch (position) {
                    case SHOOT_1:
                        spindexer.setPosition(SPINDERXER_POSITION_SHOOT_2);
                        position = SpindexerPosition.SHOOT_2;
                        break;
                    case SHOOT_2:
                        spindexer.setPosition(SPINDERXER_POSITION_SHOOT_3);
                        position = SpindexerPosition.SHOOT_3;
                        break;
                    case SHOOT_3:
                        spindexer.setPosition(SPINDERXER_POSITION_SHOOT_1);
                        position = SpindexerPosition.SHOOT_1;
                        break;
                }
            }else {
                switch (position) {
                    case SHOOT_1:
                        spindexer.setPosition(SPINDERXER_POSITION_INTAKE_2);
                        position = SpindexerPosition.SHOOT_2;
                        break;
                    case SHOOT_2:
                        spindexer.setPosition(SPINDERXER_POSITION_INTAKE_3);
                        position = SpindexerPosition.SHOOT_3;
                        break;
                    case SHOOT_3:
                        spindexer.setPosition(SPINDERXER_POSITION_INTAKE_1);
                        position = SpindexerPosition.SHOOT_1;
                        break;
                }
            }
        }
    }

    public void initialize(){
        state = SpindexerState.MOVING;
        moveStartTime = System.currentTimeMillis();
        spindexer.setPosition( SPINDERXER_POSITION_INTAKE_1);
        position = SpindexerPosition.SHOOT_1;
    }

    public double getPosition(){
        return spindexer.getPosition();
    }

    public void switchMode(){
        if( mode == SpindexerMode.INTAKING){
            mode = SpindexerMode.SHOOTING;
            switch (position) {
                case SHOOT_2:
                    spindexer.setPosition(SPINDERXER_POSITION_SHOOT_2);
                    position = SpindexerPosition.SHOOT_2;
                    break;
                case SHOOT_3:
                    spindexer.setPosition(SPINDERXER_POSITION_SHOOT_3);
                    position = SpindexerPosition.SHOOT_3;
                    break;
                case SHOOT_1:
                    spindexer.setPosition(SPINDERXER_POSITION_SHOOT_1);
                    position = SpindexerPosition.SHOOT_1;
                    break;
            }
        }else{
            mode = SpindexerMode.INTAKING;
            switch (position) {
                case SHOOT_2:
                    spindexer.setPosition(SPINDERXER_POSITION_INTAKE_2);
                    position = SpindexerPosition.SHOOT_2;
                    break;
                case SHOOT_3:
                    spindexer.setPosition(SPINDERXER_POSITION_INTAKE_3);
                    position = SpindexerPosition.SHOOT_3;
                    break;
                case SHOOT_1:
                    spindexer.setPosition(SPINDERXER_POSITION_INTAKE_1);
                    position = SpindexerPosition.SHOOT_1;
                    break;
            }
        }
    }

}
