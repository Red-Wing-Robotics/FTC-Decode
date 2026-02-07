package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class VelocityCalculation {

    public static double NEAR_VELOCITY_COEFFICIENT = 1140;

    public static double VELOCITY_DEFAULT = 1350;

    public static double getTargetVelocity( double distanceToGoal ){
        if(distanceToGoal == 0){
            return 0;
        } else if(distanceToGoal < 90 ){
            return 6.27 * distanceToGoal + NEAR_VELOCITY_COEFFICIENT;
        } else if (distanceToGoal > 110 ) {
            return 1850;
        } else {
            return 1480;
        }
    }
}
