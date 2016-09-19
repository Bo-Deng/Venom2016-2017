package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        colorF = hardwareMap.colorSensor.get("colorF");
        colorB = hardwareMap.colorSensor.get("colorB");
        colorBeacon = hardwareMap.colorSensor.get("colorBeacon");
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        while (colorB.alpha() < 20) {
            move(1, 1);
        }
        //Assuming the back color sensor is at robot pivot point
        while (colorF.alpha() < 20) {
            move(.5, -.5);
        }
        while (colorBeacon.blue() < 100 && colorBeacon.red() < 100) {
            lineFollow();
        }
    }

    public void lineFollow() {
        boolean isLeft = true;

        while (colorF.alpha() > 10 && colorB.alpha() > 10) {
            move(1, 1);
        }

        if (colorF.alpha() < 10) {
            if (isLeft) {
                while (colorF.alpha() < 10) {
                    move(.5, -.5);
                }
            }
            else {
                while (colorF.alpha() < 10) {
                    move(-.5, .5);
                }
            }
        }

        if (colorB.alpha() < 10) {
            if (isLeft) {
                //while ()
            }
            else {

            }
        }


    }

    public void move(double leftSpeed, double rightSpeed) {
        motorBL.setPower(leftSpeed);
        motorBR.setPower(rightSpeed);
        motorFL.setPower(leftSpeed);
        motorFR.setPower(rightSpeed);
    }
}
