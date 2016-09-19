package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Navya on 9/19/2016.
 */

@Autonomous(name = "AutoMethods", group = "Auto")

public class AutoMethods extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    public void runOpMode()  {

    }

    public void turnTime (boolean left, int msTurn) throws InterruptedException {

        if (left) {

            setMotor(-0.5);

        } else {

            setMotor(0.5);
        }

        sleepTime(msTurn);

        setMotor(0);

    }

    public void setMotor(double motorValue){
        motorBL.setPower(motorValue);
        motorFL.setPower(motorValue);
        motorBR.setPower(motorValue);
        motorFR.setPower(motorValue);

    }

    public void sleepTime(int sleepTime) throws InterruptedException{
        sleep(sleepTime);

      /*try {
          sleep(sleepTime);
        } catch (Exception e) {
        }*/
    }

    public void shoot ()


}
