package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class VelocityCalculation {

    public static double NEAR_VELOCITY_COEFFICIENT = 1150;

    public static double VELOCITY_DEFAULT = 1350;

    public static double getTargetVelocity( double distanceToGoal ){
        if(distanceToGoal == 0){
            return 0;
        } else if(distanceToGoal < 90 ){
            return 1172 - ( 2.91 * distanceToGoal ) + (0.0682 * Math.pow( distanceToGoal, 2 ));
        } else if (distanceToGoal > 110 ) {
            return 1850;
        } else {
            return 1480;
        }
    }


}
