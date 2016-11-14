package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Bo on 11/11/2016.
 */

@Autonomous(name = "BasicAuto", group = "Auto")
public class BasicAuto extends LinearOpMode {

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorLaunchR;
    DcMotor motorLaunchL;

    double launchSpeed = 0.0;


    @Override
    public void runOpMode() throws InterruptedException{

        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBL");
        motorFL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorBL");
        motorLaunchR = hardwareMap.dcMotor.get("motorBL");
        motorLaunchL = hardwareMap.dcMotor.get("motorBL");

        waitForStart();
        moveTime(100, -0.5, 0.5);

        while (launchSpeed < 1) {

            motorLaunchR.setPower(launchSpeed);
            motorLaunchL.setPower(launchSpeed);
        }

        motorLaunchL.setPower(0);
        motorLaunchR.setPower(0);


    }

    public void moveTime(int msTime, double leftSpeed, double rightSpeed) throws InterruptedException{
        motorBL.setPower(-leftSpeed);
        motorBR.setPower(-rightSpeed);
        motorFL.setPower(leftSpeed);
        motorFR.setPower(rightSpeed);

        sleep(msTime);

        motorBR.setPower(0);
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

    }
}
