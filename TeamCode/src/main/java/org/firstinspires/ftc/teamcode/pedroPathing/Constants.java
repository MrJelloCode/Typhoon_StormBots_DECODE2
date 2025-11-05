package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-84.29227179548407)
            .lateralZeroPowerAcceleration(-91.60769382579304)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.02, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.2,0.0))
            .centripetalScaling(0.006);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.75)
            .xVelocity(57.964169713632)
            .yVelocity(50.5166327977101072)

            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.000592952184)
            .strafeTicksToInches(0.000620927)
            .turnTicksToInches(-0.0005392134)
            .leftPodY(7)
            .rightPodY(-7)
            .strafePodX(-5)
            .leftEncoder_HardwareMapName("backRightMotor")
            .rightEncoder_HardwareMapName("frontLeftMotor")
            .strafeEncoder_HardwareMapName("frontRightMotor")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)

                .build();
    }
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


}
