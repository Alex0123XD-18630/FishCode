package org.firstinspires.ftc.teamcode.aurumCode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class Fishpop extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection DesiredTag = null;
    final double DESIRED_DISTANCE = 30.0; // In INCHES
    final double SPEED_GAIN = 0.02;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN = 0.01;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE = 0.5;
    final double MAX_AUTO_TURN = 0.5;

    private DcMotorEx flyWHeel, Intake;
    private Servo flicker;

    //PIDF Coefficients
    /**
     * F is the most important
     * P helps recover speed after a shot
     * I and D usually 0 for flywheels to prevent oscillation
     */
    public static double NEW_F = 21.1084;
    public static double NEW_P = 12.00;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;

    //known data points (knots)
    private final double[] rangeData = {36.07, 49.5, 58.97, 70.40, 84.60};
    private final double[] speedData = {1000, 1150, 1200, 1250, 1300};

    private CubicSpline shooterSpline;

    @Override
    public void runOpMode(){

        double drive = 0;
        double strafe = 0;
        double turn = 0;

        FieldCentricDrive Drive = new FieldCentricDrive(hardwareMap);
        DesiredTag desiredTag = new DesiredTag();
        initVision();

        flyWHeel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        Intake = hardwareMap.get(DcMotorEx.class, "intake");
        flicker = hardwareMap.get(Servo.class, "flicker");

        flicker.setPosition(1);
        //PIDF SETUP
        flyWHeel.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWHeel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfOrig = flyWHeel.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        //new one with updated values
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        flyWHeel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

            shooterSpline = new CubicSpline(rangeData, speedData);

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int detectedTagId = desiredTag.findPatternTag(currentDetections);

            double targetDistanceInches = desiredTag.getTagRange();

            //flicker
            if(gamepad2.dpad_up){
                flicker.setPosition(0);
            }
            else{
                flicker.setPosition(0.5);
            }


            //intake
            if(gamepad2.left_bumper){
                ((DcMotorEx) Intake).setVelocity(4500);

            }
            else if (gamepad2.right_bumper) {
                ((DcMotorEx) Intake).setVelocity(-4500);
            }
            else{
                Intake.setVelocity(0);
            }

            //shooter
            if(gamepad2.a){
                //safety clamp
                double clampedRange = Math.max(36.07, Math.min(targetDistanceInches, 84.60));

                //calculate velocity
                double targetVelocity = shooterSpline.interpolate(clampedRange);
                flyWHeel.setVelocity(targetVelocity);

                telemetry.addData("target Dist (in)", "%.2f", targetDistanceInches);
                telemetry.addData("Calculated vel", "%.2f", targetVelocity);
                telemetry.addData("Target Vel", targetVelocity);
                telemetry.addData("Actual Velocity", flyWHeel.getVelocity());
                telemetry.addData("Power", flyWHeel.getPower());
            }
            else if(gamepad2.b){
                flyWHeel.setVelocity(-1250);
            }
            else{
                flyWHeel.setVelocity(0);
            }

            if(gamepad1.y){
                Drive.resetHeading();
            }

            //alignment
            if(gamepad1.right_bumper) {
                if(detectedTagId != -1){
                    double rangeError = (desiredTag.getTagRange() - DESIRED_DISTANCE);
                    double headingError = desiredTag.getTagBearing();
                    double yawError = desiredTag.getTagYaw();


                    drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    Drive.alignmentMotorPower(drive, strafe, turn);
                }
            }
            Drive.drive(gamepad1);
            telemetry.update();
        }
    }

    private void initVision(){
        //create apriltag processor by using builder
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(3);

        //create vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .addProcessor(aprilTag)
                .build();


    }




}