package org.firstinspires.ftc.teamcode.util;

public enum BallColor {
    GREEN( 0 ),
    PURPLE( 1 );

    private final int value;

    BallColor(int value) { this.value = value; }

    public int getValue() {
        return value;
    }

}
