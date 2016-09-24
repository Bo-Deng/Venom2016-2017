package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Navya on 9/13/2016.
 */
@Autonomous(name = "AutoTrollBot", group = "Auto")
public class TrollBotAutonomous extends LinearOpMode{

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    public void runOpMode() {

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorFR

        moveTime(-1, 1, 5000);
        moveTime(-1, -1, 500);
        moveTime(-1, 1, 3000);

    }

    public void moveTime(double leftSideSpeed, double rightSideSpeed, int msSleep) {

        motorFL.setPower(leftSideSpeed);
        motorFR.setPower(rightSideSpeed);
        motorBL.setPower(leftSideSpeed);
        motorBR.setPower(rightSideSpeed);

        try{
            sleep(msSleep);
        }
        catch(Exception e){
        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }
}
