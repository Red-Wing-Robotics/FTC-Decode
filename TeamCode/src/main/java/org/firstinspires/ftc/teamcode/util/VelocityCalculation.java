package org.firstinspires.ftc.teamcode.util;

public class VelocityCalculation {

    public static double getTargetVelocity( double distanceToGoal ){
        if(distanceToGoal < 75 ){
            return 1827  - ( 18.1 * distanceToGoal ) + ( 0.177 * Math.pow( distanceToGoal, 2 ) );
        } else if (distanceToGoal > 105 ) {
            return 3215 - ( 30.5 * distanceToGoal ) + ( 0.141 * Math.pow( distanceToGoal, 2 ) );
        } else {
            return 1480;
        }
    }
}
