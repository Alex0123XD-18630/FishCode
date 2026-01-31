package org.firstinspires.ftc.teamcode.aurumCode;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@SuppressWarnings("FieldCanBeLocal")
@Configurable
@TeleOp
public class NewFishPop extends LinearOpMode {
    // Pedro Pathing
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0); // Adjust if needed

    private final Pose blueLaunchPose = new Pose(70.03, 21.21, Math.toRadians(108));
    private final Pose redLaunchPose  = new Pose(76.96, 17.85, Math.toRadians(45));


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private DesiredTag desiredTagWrapper = new DesiredTag();


    private DcMotorEx flyWHeel, Intake;
    private Servo flicker;

    //PIDF
    public static double NEW_F = 21.1084;
    public static double NEW_P = 15.00;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;

    // --- Spline Data ---
    private final double[] rangeData = {36.07, 49.5, 58.97, 70.40, 84.60}; // adjust to match the shooter
    private final double[] speedData = {1000, 1150, 1200, 1250, 1300}; // the velocity of the flyWheel at each range

    private CubicSpline shooterSpline;

    // State Variables
    private Timer actionTimer = new Timer();
    private int shootState = 0; // 0 = Idle
    private int shotCount = 0;
    private boolean isShootingMacro = false;

    // Constants for State Variables
    private static final double SERVO_UP = 0.7;
    private static final double SERVO_DOWN = 0.0;
    private static final double INTAKE_SPEED = 4500;
    private static final double SHOOTER_MACRO_SPEED = 4500;
    @Override
    public void runOpMode(){
        //Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleopDrive();


        initVision();
        flyWHeel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        Intake = hardwareMap.get(DcMotorEx.class, "intake");
        flicker = hardwareMap.get(Servo.class, "flicker");

        flicker.setPosition(SERVO_DOWN);

        // 3. Setup PIDF
        flyWHeel.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWHeel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        flyWHeel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        // Start the timer
        actionTimer.resetTimer();

        while(opModeIsActive()){
            follower.update();

            // Vision Updates
            shooterSpline = new CubicSpline(rangeData, speedData);
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            int detectedTagId = desiredTagWrapper.findPatternTag(currentDetections);
            double targetDistanceInches = desiredTagWrapper.getTagRange();
            if (targetDistanceInches == 0) targetDistanceInches = 50.0;

            //Drive Control
            //manual drive
            if (!follower.isBusy()) {
                // Pedro Pathing handles field centric, can make a FieldCentricDrive instance if needed to use field centric
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        false // false = Field Centric
                );
            }

            // Auto-Drive to Launch Zone
            if (gamepad1.dpad_up) {
                // Go to Blue Side Launch
                Path path = new Path(new BezierLine(follower::getPose, blueLaunchPose));
                path.setLinearHeadingInterpolation(follower.getPose().getHeading(), blueLaunchPose.getHeading());
                follower.followPath(path);
            }
            else if (gamepad1.dpad_down) {
                // Go to Red Side Launch
                Path path = new Path(new BezierLine(follower::getPose, redLaunchPose));
                path.setLinearHeadingInterpolation(follower.getPose().getHeading(), redLaunchPose.getHeading());
                follower.followPath(path);
            }

             //Burst Shot (Gamepad 1 X)
            if (gamepad1.x && shootState == 0) {
                shootState = 1;
                shotCount = 0;
                isShootingMacro = true;
                actionTimer.resetTimer();
            }



            if (isShootingMacro) {
                runShootingSequence();
            } else {
                // Manual Intake
                if (gamepad1.left_bumper) {
                    Intake.setVelocity(INTAKE_SPEED);
                } else if (gamepad1.right_bumper) {
                    Intake.setVelocity(-INTAKE_SPEED);
                } else {
                    Intake.setVelocity(0);
                }

                //Manual Flywheel
                if (gamepad1.a) {
                    // Calculated Shot using Spline
                    double clampedRange = Math.max(36.07, Math.min(targetDistanceInches, 84.60));
                    double targetVelocity = shooterSpline.interpolate(clampedRange);
                    flyWHeel.setVelocity(targetVelocity);

                    telemetry.addData("Target Vel", targetVelocity);
                } else if (gamepad1.b) {
                    flyWHeel.setVelocity(-1250);
                } else {
                    flyWHeel.setVelocity(0);
                }

                //flicker
                if(gamepad1.dpad_up){
                    flicker.setPosition(1);
                } else if (gamepad1.dpad_down) {
                    flicker.setPosition(0);
                }


            }

            // Reset Heading
            if (gamepad1.y) {
                follower.setPose(new Pose(follower.getPose().getX(), follower.getPose().getY(), 0));
            }

            // Telemetry
            telemetry.addData("State", shootState);
            telemetry.addData("Shot Count", shotCount);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading", follower.getPose().getHeading());
            telemetry.addData("flyWheel speed:", flyWHeel.getVelocity());
            telemetry.update();
        }

    }

    private void runShootingSequence() {
        // keep flywheel spinning
        flyWHeel.setVelocity(SHOOTER_MACRO_SPEED);

        switch (shootState) {
            case 1: // Flick UP
                flicker.setPosition(SERVO_UP);
                if (actionTimer.getElapsedTimeSeconds() > 0.3) { // Wait for servo
                    shootState = 2;
                    actionTimer.resetTimer();
                }
                break;

            case 2: // Flick DOWN
                flicker.setPosition(SERVO_DOWN);
                if (actionTimer.getElapsedTimeSeconds() > 0.3) { // Wait for servo
                    shootState = 3;
                    actionTimer.resetTimer();
                }
                break;

            case 3: // Intake ON
                Intake.setVelocity(INTAKE_SPEED);
                if (actionTimer.getElapsedTimeSeconds() > 2.0) { // Wait 2 seconds
                    Intake.setVelocity(0);
                    shotCount++;

                    if (shotCount < 3) {
                        shootState = 1; // Repeat
                    } else {
                        shootState = 0; // End
                        isShootingMacro = false;
                        flyWHeel.setVelocity(0); // Turn off flywheel
                    }
                    actionTimer.resetTimer();
                }
                break;
        }
    }

    private void initVision(){
        //apriltag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(3);

        //vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,480))
                .addProcessor(aprilTag)
                .build();


    }

}
