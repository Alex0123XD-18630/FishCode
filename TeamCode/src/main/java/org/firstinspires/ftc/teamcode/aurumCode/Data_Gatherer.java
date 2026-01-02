package org.firstinspires.ftc.teamcode.aurumCode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp
public class Data_Gatherer extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection DesiredTag = null;

    //PIDF Coefficients
    /**
     * F is the most important
     * P helps recover speed after a shot
     * I and D usually 0 for flywheels to prevent oscillation
     */
    public static double NEW_F = 20.0084;
    public static double NEW_P = 12.00;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;

    private DcMotorEx flyWheel, Intake;
    private Servo flicker;

    @Override
    public void runOpMode(){

        DesiredTag desiredTag = new DesiredTag();
        initVision();

        flyWheel = hardwareMap.get(DcMotorEx.class, "spinny");
        Intake = hardwareMap.get(DcMotorEx.class, "intake");
        flicker = hardwareMap.get(Servo.class, "SingleBallPusherServo");
        flicker.setPosition(0.2);

        flyWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //new one with updated values
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int detectedTagId = desiredTag.findPatternTag(currentDetections);
            double targetDistanceInches = desiredTag.getTagRange();

            //intake
            if(gamepad1.left_bumper){
                ((DcMotorEx) Intake).setVelocity(4500);
            }
            else if (gamepad1.right_bumper) {
                ((DcMotorEx) Intake).setVelocity(-4500);
            }
            else{
                Intake.setVelocity(0);
            }

            //flicker
            if(gamepad1.dpad_up){
                flicker.setPosition(1);
            }
            else{
                flicker.setPosition(0);
            }


            //shooter
            if(detectedTagId != -1){
                if(gamepad1.a){
                    /*
                     * Distance: 36.07,Velocity: 1000
                     * Distance: 49.5, Velocity: 1150
                     * Distance: 58.97, Velocity: 1200
                     * Distance: 70.40, Velocity: 1250
                     * Distance: 84.60, Velocity: 1300
                     * */

                    double targetVelocity = 1300;
                    flyWheel.setVelocity(targetVelocity);

                    telemetry.addData("target Dist (in)", "%.2f", targetDistanceInches);
                    telemetry.addData("Target Velocity", targetVelocity);
                    telemetry.addData("Actual Velocity", flyWheel.getVelocity());
                    telemetry.addData("Power", flyWheel.getPower());
                }
                else if(gamepad1.b){

                    flyWheel.setVelocity(-1000);
                }
                else{
                    flyWheel.setVelocity(0);
                }
            }

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
