package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-96.64896166337861) // jan 14, 2026
            .lateralZeroPowerAcceleration(-75.62120444667116) // jan 14, 2026
            .translationalPIDFCoefficients(new PIDFCoefficients(0.03, 0, 0.001, 0.03))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(0.5,0, 0.03, 0.05))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.05, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.15, 0, 0.01, 0.01, 0.6))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.15, 0.01, 0.6))
            .centripetalScaling(0.00005)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .mass(8.70); // new weight (Jan 4, 2026)

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(144.88213100418454)
            .yVelocity(95.49539819566854)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD);


    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("rightFront")
            .strafeEncoder_HardwareMapName("backLeft")
            .strafePodX(-5.5)
            .forwardPodY(6)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .forwardTicksToInches(0.006249952029222989) // NEW MULTIPLIER: 0.006249952029222989 (Jan 13, 2026)
            .strafeTicksToInches(0.00459384451211043) //NEW multiplier: 0.00459384451211043 (Jan 13, 2026)
            /*higher number= less distance, lower number= more distance*/
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                    )
            );
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }

}
