package org.firstinspires.ftc.teamcode.aurumCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlyWheel_Tuner extends LinearOpMode {

    private DcMotorEx flyWheel;

    //default tuning values
    // F = 0, P = 0 to see raw behaivors or estimated values
    // F will be calculated by the "Calibration Mode"
    public static double tuning_F = 0; // 24.0084
    public static double tuning_P = 10; // 13.00
    public static double tuning_I = 0;
    public static double tuning_D = 0;

    //test target: gonna estimate because it is now weighted
    public static double TARGET_VELOCITY = 1000;

    private double maxVelocitySeen = 0;

    @Override
    public void runOpMode(){
        flyWheel = hardwareMap.get(DcMotorEx.class, "spinny");

        //reset
        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheel.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Ready to Tune");
        telemetry.addData("Instructions", "Press A for Max Speed (Calculate F)");
        telemetry.addData("Instructions", "Press B for Max Speed (Test P)");
        telemetry.update();

        waitForStart();
        boolean isTuningPID = false;  // False = Manual/Max Speed, True = PID Loop

        while(opModeIsActive()){
            // INPUTS

            //MODE 1: MAX SPEED CALIBRATION (Hold A)
            if(gamepad1.a){
                isTuningPID = false;
                flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flyWheel.setPower(1.0); // full power

                double currentVel = flyWheel.getVelocity();

                // Track highest speed seen
                if(currentVel > maxVelocitySeen){
                    maxVelocitySeen = currentVel;
                }

                //Formula: F = 32767 / MaxVelocity
                if(maxVelocitySeen > 0){
                    tuning_F = 32767.0 / maxVelocitySeen;
                }
            }
            //MODE 2: PID TESTING (Press B to Toggle ON, X to toggle OFF)
            else if (gamepad1.b) {
                isTuningPID = true;
                flyWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else if (gamepad1.x) {
                isTuningPID = false;
                flyWheel.setPower(0);
            }

            //LIVE TUNING (ONLY WORKS WHEN NOT HOLDING A)
            if(!gamepad1.a){
                //Adjust P (Proportional)
                if(gamepad1.dpad_up){
                    tuning_P += 0.1;
                    sleep(200);
                }
                else if (gamepad1.dpad_down) {
                    tuning_P -= 0.1;
                    sleep(200);
                }

                //Adjust F (Feedforward) manually if needed
                if(gamepad1.dpad_right){
                    tuning_F += 0.1;
                    sleep(200);
                } else if (gamepad1.dpad_left) {
                    tuning_F -= 0.1;
                    sleep(200);
                }
            }

            //EXECUTION
            if(isTuningPID){
                //update coefficients in real-time
                PIDFCoefficients newPIDF = new PIDFCoefficients(tuning_P, tuning_I, tuning_D, tuning_F);
                flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);

                //run at target
                flyWheel.setVelocity(TARGET_VELOCITY);
            } else if (!gamepad1.a) {
                //if not holding A and not in PID mode, stop.
                flyWheel.setPower(0);
            }

            telemetry.addData("Mode", isTuningPID ? "PID CONTROL": "MANUAL POWER");
            telemetry.addData("Current Velocity", "%.2f", flyWheel.getVelocity());
            telemetry.addData("Max Velocity Seen", "%.2f", maxVelocitySeen);

            telemetry.addLine("\n---PIDF VALUES---");
            telemetry.addData("F (FeedForward)", "%.4f", tuning_F);
            telemetry.addData("P (Proportional)", "%.4f", tuning_P);

            if(isTuningPID){
                telemetry.addLine("\n---PERFORMANCE---");
                telemetry.addData("Target", TARGET_VELOCITY);
                telemetry.addData("Error", TARGET_VELOCITY - flyWheel.getVelocity());
            }
            telemetry.update();

        }
    }
}
