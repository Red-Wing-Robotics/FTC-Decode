package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class VelocityCalculation {

    public static double NEAR_VELOCITY_COEFFICIENT_1 = 18.2;

    public static double NEAR_VELOCITY_COEFFICIENT_2 = 0.177;

    public static double getTargetVelocity( double distanceToGoal ){
        if(distanceToGoal < 75 ){
            return 1827  - ( NEAR_VELOCITY_COEFFICIENT_1 * distanceToGoal ) + ( NEAR_VELOCITY_COEFFICIENT_2 * Math.pow( distanceToGoal, 2 ) );
        } else if (distanceToGoal > 105 ) {
            return 3215 - ( 30.5 * distanceToGoal ) + ( 0.141 * Math.pow( distanceToGoal, 2 ) );
        } else {
            return 1480;
        }
    }
}
