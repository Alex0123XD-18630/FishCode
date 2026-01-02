package org.firstinspires.ftc.teamcode.pedroPathing;

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
            //.forwardZeroPowerAcceleration(//deceleration//)
            //.lateralZeroPowerAcceleration(//deceleration//)
            .mass(8.92); // new weight (nov 6)

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(137.86573659667957)
            //.yVelocity(//insert velocity//)
            .rightFrontMotorName("0")
            .rightRearMotorName("bR")
            .leftRearMotorName("bL")
            .leftFrontMotorName("1")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD);


    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("0")
            .strafeEncoder_HardwareMapName("1")
            .strafePodX(6.25)
            .forwardPodY(-5.25)
            .strafeEncoderDirection(Encoder.REVERSE) // I swapped the encoders...
            .forwardEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .forwardTicksToInches(0.005594963117452) //THIS IS THE X-axis(strafe) MULTIPLIER...
            // NEW MULTIPLIER: 0.001986246460070257
            //lowk you just gotta explode the robot atp

            .strafeTicksToInches(100.4594963117452) //THIS IS THE Y-axis(forward) MULTIPLIER...
            /*higher number= less distance, lower number= more distance*/
            //CURRENT: 0.04594963117452, theoretically should work correctly
            //NEW multiplier: 0.36194963117452 (OCT/31) **needs some fine tuning
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
    //party like a human by something electrics
}
