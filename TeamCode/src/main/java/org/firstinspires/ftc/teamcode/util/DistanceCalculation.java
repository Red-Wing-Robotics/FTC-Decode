package org.firstinspires.ftc.teamcode.util;

public class DistanceCalculation {

    /**
     *
     * @param ty the angle from the ground to the line formed by the limelight and the april tag
     * @return the distance from the limelight to the goal
     */
    private static double getTrigDistanceToTarget(double ty) {
        double targetHeight = 29.4375;
        double limelightHeight = 13.34375;
        double limelightAngle = 0;

        double angleToTarget = Math.toRadians(limelightAngle + ty);
        return (targetHeight - limelightHeight) / Math.tan(angleToTarget);
    }

    /**
     *
     * @param ta the area of the limelight's display that contains the april tag
     * @return the distance from the limelight to the goal
     */
    private static double getAreaDistanceToTarget( double ta ){
        return 69.6*(Math.pow(ta, -0.502));
    }
}
