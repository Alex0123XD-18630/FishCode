package org.firstinspires.ftc.teamcode.aurumCode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@SuppressWarnings("FieldCanBeLocal")
@Autonomous
public class FeedFishAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    private int pathState;
    private DcMotorEx flyWHeel, Intake;
    private Servo flicker;

    // PIDF Coefficients

    public static double NEW_F = 21.1084;
    public static double NEW_P = 15.00;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;

    // --- Auto Constants ---
    private static final double START_WAIT_SECONDS = 3.0; // Wait time at start
    private static final double INTAKE_VELOCITY = 4500;   // Velocity for Intake
    private static final double FLYWHEEL_VELOCITY = 2000; // Target velocity for shooting
    private int shotCount = 0; // Counter for the 3 shots

    private final Pose startingPose = new Pose(56.8, 7.1, Math.toRadians(90));
    private final Pose scorePose = new Pose(75.01, 22.83, Math.toRadians(108));
    private final Pose setUpPose = new Pose(45, 39.76, Math.toRadians(180));
    private final Pose pickUpPose = new Pose(1.08, 34.66, Math.toRadians(180));

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        buildPaths();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        flyWHeel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        Intake = hardwareMap.get(DcMotorEx.class, "intake");
        flicker = hardwareMap.get(Servo.class, "flicker");

        flicker.setPosition(0);

        // PIDF SETUP
        flyWHeel.setDirection(DcMotorSimple.Direction.REVERSE);
        flyWHeel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Apply new tuned coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        flyWHeel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfNew);

        telemetry.update();
    }
    private Path setupPickUp;
    private PathChain grabPickUp, scorePickUp;
    public void buildPaths(){

        setupPickUp = new Path(new BezierLine(startingPose, setUpPose));
        setupPickUp.setLinearHeadingInterpolation(startingPose.getHeading(), setUpPose.getHeading());


        grabPickUp = follower.pathBuilder()
                .addPath(new BezierLine(setUpPose, pickUpPose))
                .setLinearHeadingInterpolation(setUpPose.getHeading(), pickUpPose.getHeading())
                .build();
        scorePickUp = follower.pathBuilder()
                .addPath(new BezierLine(pickUpPose, scorePose))
                .setLinearHeadingInterpolation(pickUpPose.getHeading(), scorePose.getHeading())
                .build();
    }


    public void autoUpdate(){
        switch (pathState) {
            case 0: // Wait state at Start
                if (pathTimer.getElapsedTimeSeconds() > START_WAIT_SECONDS) {
                    if(!follower.isBusy()) {
                        follower.followPath(setupPickUp);
                        setPathState(1);
                    }
                }
                break;

            case 1: // Moving to Pickup
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    Intake.setVelocity(INTAKE_VELOCITY);
                }

                if(pathTimer.getElapsedTimeSeconds() > START_WAIT_SECONDS){
                    if(!follower.isBusy()) {
                        // Turn Intake OFF after path finishes
                        Intake.setVelocity(0);
                        follower.followPath(grabPickUp, true);
                        setPathState(2);
                    }
                }
                break;

            case 2: // Moving to Score
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    Intake.setVelocity(INTAKE_VELOCITY);
                }

                if(!follower.isBusy()){
                    follower.followPath(scorePickUp, true);
                    setPathState(3);
                }
                break;

            case 3: // Arrived at Score Pose
                if(!follower.isBusy()){
                    //Flywheel is spinning
                    flyWHeel.setVelocity(FLYWHEEL_VELOCITY);
                    shotCount = 0;
                    setPathState(4);
                }
                break;

            // Shooting Loop (Cases 4, 5, 6)
            case 4: // Flick UP
                flicker.setPosition(1);
                // Wait small amount
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(5);
                }
                break;

            case 5: // Flick DOWN
                flicker.setPosition(0);
                // Wait small amount
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    setPathState(6);
                }
                break;

            case 6: // Intake for 2 Seconds (Reload)
                Intake.setVelocity(INTAKE_VELOCITY);

                // Wait 2 seconds
                if (pathTimer.getElapsedTimeSeconds() > 2.0) {
                    Intake.setVelocity(0); // Stop Intake
                    shotCount++; //a cycle is done

                    // Check if need to repeat
                    if (shotCount < 3) {
                        setPathState(4); // Loop back
                    } else {
                        setPathState(-1); // End Auto
                    }
                }
                break;
        }
    }
    public void setPathState(int pState){
        pathState = pState;
        pathTimer.resetTimer();
    }
    @Override
    public void loop() {
        follower.update();
        autoUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("shot count", shotCount);
        telemetry.addData("flywheel velocity", flyWHeel.getVelocity()); // Monitor velocity
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init_loop(){}

    @Override
    public void start(){
        opModeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop(){
        flyWHeel.setVelocity(0);
        Intake.setVelocity(0);
    }


}
