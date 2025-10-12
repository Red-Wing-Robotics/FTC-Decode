package org.firstinspires.ftc.teamcode.util;

public enum ObeliskState {

    GREEN_PURPLE_PURPLE(21),
    PURPLE_GREEN_PURPLE(22),
    PURPLE_PURPLE_GREEN(23),
    UNKNOWN(-1);

    private final int value;

    ObeliskState(int value) {
        this.value = value;
    }

    public int getValue() {
        return value;
    }

    public static ObeliskState fromInt(int input) {
        for (ObeliskState status : ObeliskState.values()) {
            if (status.value == input) {
                return status;
            }
        }
        return UNKNOWN;
    }

}
