package org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
public class AutoConstants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.2)
            .forwardZeroPowerAcceleration(-55.6272)
            .lateralZeroPowerAcceleration(-110.6846093)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.25, 0, 0.02, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.2, 0.0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.2,0.0))
            .centripetalScaling(0.006);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(50.26715)
            .yVelocity(37.85232848)

            .rightFrontMotorName("FR")
            .rightRearMotorName("FL")
            .leftRearMotorName("BL")
            .leftFrontMotorName("BR")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(-0.00058376)
            .strafeTicksToInches(0.00057631)
            .turnTicksToInches(0.000532430)
            .leftPodY(7)
            .rightPodY(-7)
            .strafePodX(-6)
            .leftEncoder_HardwareMapName("FL")
            .rightEncoder_HardwareMapName("BR")
            .strafeEncoder_HardwareMapName("FR")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)

                .build();
    }
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);


}
