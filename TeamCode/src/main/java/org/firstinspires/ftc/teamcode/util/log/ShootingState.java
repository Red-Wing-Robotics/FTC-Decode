package org.firstinspires.ftc.teamcode.util.log;

public enum ShootingState {

    INACTIVE ( 0 ),
    PREPARING_TO_SHOOT( 1 ),
    SHOOTING( 2 ),
    UNKNOWN( -1 );

    private final int value;

    ShootingState(int value) { this.value = value; }

    public int getValue() {
        return value;
    }
}
