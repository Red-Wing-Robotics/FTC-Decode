package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.83)
            .forwardZeroPowerAcceleration(-34.30537347818821)
            .lateralZeroPowerAcceleration(-241.50027821714204)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    1.2,
                    0,
                    0.001,
                    1.75
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.08,
                    0,
                    0.0008,
                    0.003
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.6,
                    0,
                    0.0001,
                    0.03
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    1.6,
                    0,
                    0.0001,
                    0.005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.5,
                    0,
                    0.0001,
                    0.6,
                    0.1
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.4,
                    0,
                    0.005,
                    0.6,
                    0.04
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("frontLeftMotor")
            .leftRearMotorName("backLeftMotor")
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(52.2883615568867)
            .yVelocity(45.78680937684427);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY( -0.5 )//0.375 for other chasee
            .strafePodX( -6.125 )//-6.045 for other chasee
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart

     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            2,
            10,
            0.8
    );

    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
