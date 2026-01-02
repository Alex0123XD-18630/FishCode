package org.firstinspires.ftc.teamcode.aurumCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class fishConstants {
    private final DcMotorEx fL, fR, bL, bR, ultraSpinny;
    private final DcMotor spinny;

    public fishConstants (HardwareMap hardwareMap) {
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

        //initialize mechanism
        spinny = hardwareMap.get(DcMotor.class, "spinny");
        ultraSpinny = hardwareMap.get(DcMotorEx.class, "ultraSpinny");
    }
}
