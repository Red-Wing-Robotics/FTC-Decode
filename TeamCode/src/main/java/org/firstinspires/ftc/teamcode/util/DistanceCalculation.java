package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class DistanceCalculation {

    public static double redGoalx = 128d;
    public static double redGoaly = 132d;
    public static double blueGoalx = 16d;
    public static double blueGoaly = 132d;


    /**
     *
     * @param ty the angle from the ground to the line formed by the limelight and the april tag
     * @return the distance from the limelight to the goal
     */
    public static double getTrigDistanceToTarget(double ty) {
        double targetHeight = 29.4375;
        double limelightHeight = 13.34375;
        double limelightAngle = 26.3;

        double angleToTarget = Math.toRadians(limelightAngle + ty);
        return (targetHeight - limelightHeight) / Math.tan(angleToTarget);
    }

    public static double getTrigDistanceToTargetBot2(double ty) {
        double targetHeight = 29.4375;
        double limelightHeight = 13;
        double limelightAngle = 25;

        double angleToTarget = Math.toRadians(limelightAngle + ty);
        return (targetHeight - limelightHeight) / Math.tan(angleToTarget);
    }

    /**
     *
     * @param ta the area of the limelight's display that contains the april tag
     * @return the distance from the limelight to the goal
     */
    public static double getAreaDistanceToTarget( double ta ){
        return 69.6*(Math.pow(ta, -0.502));
    }

    public static double getOdometryDistanceToBlueGoal( Pose pose ){
        return Math.sqrt( Math.pow( pose.getX() - blueGoalx, 2 ) + Math.pow( pose.getY() - blueGoaly, 2 ) );
    }

    public static double getOdometryDistanceToRedGoal( Pose pose ){
        return Math.sqrt( Math.pow( pose.getX() - redGoalx, 2 ) + Math.pow( pose.getY() - redGoaly, 2 ) );
    }

    public static double getOdometryDistanceToGoal( Pose pose, Alliance alliance ){
        switch ( alliance ){
            case BLUE:
                return Math.sqrt( Math.pow( pose.getX() - blueGoalx, 2 ) + Math.pow( pose.getY() - blueGoaly, 2 ) );
            case RED:
                return Math.sqrt( Math.pow( pose.getX() - redGoalx, 2 ) + Math.pow( pose.getY() - redGoaly, 2 ) );
        }

        return -1;
    }
}
