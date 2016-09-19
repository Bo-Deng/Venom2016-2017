package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Bo on 9/14/2016.
 */
@Autonomous(name = "AutoTrollBot", group = "Auto")
public class AutoTest extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;
    ColorSensor colorF;
    ColorSensor colorB;
    ColorSensor colorBeacon;

    public void runOpMode() {
        while (colorB.alpha() < 20) {
            move(1, 1);
        }
        //Assuming the back color sensor is at robot pivot point
        while (colorF.alpha() < 20) {
            move(-.5, .5);
        }
        while (colorBeacon.blue() < 100 && colorBeacon.red() < 100) {
            lineFollow();
        }
    }

    public void lineFollow() {
        while (colorF.alpha() > 10 && colorB.alpha() > 10) {
            move(1, 1);
        }
        

    }

    public void move(double leftSpeed, double rightSpeed) {
        motorBL.setPower(-leftSpeed);
        motorBR.setPower(rightSpeed);
        motorFL.setPower(-leftSpeed);
        motorFR.setPower(rightSpeed);
    }
}
