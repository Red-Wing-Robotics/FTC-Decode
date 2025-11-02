package org.firstinspires.ftc.teamcode.util;

public enum Alliance {

    BLUE( 0 ),
    RED( 1 );

    private final int value;

    Alliance(int value) { this.value = value; }

    public int getValue() {
        return value;
    }

}
