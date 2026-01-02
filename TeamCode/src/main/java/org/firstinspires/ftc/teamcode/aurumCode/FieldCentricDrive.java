package org.firstinspires.ftc.teamcode.aurumCode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class FieldCentricDrive {
    private final IMU imu;
    private final DcMotorEx fL, fR, bL, bR;

    public FieldCentricDrive(HardwareMap hardwareMap) {
        //initialize motors
        bL = hardwareMap.get(DcMotorEx.class, "bL");
        fL = hardwareMap.get(DcMotorEx.class, "1");
        bR = hardwareMap.get(DcMotorEx.class, "bR");
        fR = hardwareMap.get(DcMotorEx.class, "0");

        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to brake mode
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }



    public void resetHeading() {
        imu.resetYaw();
    }

    public void drive(Gamepad gamepad) {
        // Get the robot's current heading from the IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);

        // Get the drive inputs from the gamepad
        double y = -gamepad.left_stick_y; // Invert for "forward" to be up on the stick
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;

        double deadband = 0.05;
        if(Math.abs(y) < deadband) y =0;
        if(Math.abs(x) < deadband) x =0;
        if(Math.abs(rx) < deadband) rx =0;

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double leftFrontPower = (rotY + rotX + rx) / denominator;
        double leftRearPower = (rotY - rotX + rx) / denominator;
        double rightFrontPower = (rotY - rotX - rx) / denominator;
        double rightRearPower = (rotY + rotX - rx) / denominator;

        fL.setPower(leftFrontPower);
        bL.setPower(leftRearPower);
        fR.setPower(rightFrontPower);
        bR.setPower(rightRearPower);
    }

    public void alignmentMotorPower(double x, double y, double yaw){
        //wheel power
        double frontLeftPower = x - y - yaw;
        double frontRightPower = x + y + yaw;
        double backLeftPower = x + y - yaw;
        double backRightPower = x - y + yaw;

        //normalize wheel power to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if(max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        fL.setPower(frontLeftPower);
        bL.setPower(backLeftPower);
        fR.setPower(frontRightPower);
        bR.setPower(backRightPower);
    }

}