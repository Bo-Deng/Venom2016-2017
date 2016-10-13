package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Bo on 9/14/2016.
 */
@Autonomous(name = "AutoTest", group = "Auto")
public class AutoTest extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;
    ColorSensor colorF;
    ColorSensor colorB;
    ColorSensor colorBeacon;

    ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        colorF = hardwareMap.colorSensor.get("colorF");
        colorB = hardwareMap.colorSensor.get("colorB");
        colorBeacon = hardwareMap.colorSensor.get("colorBeacon");
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();

        colorB.setI2cAddress(I2cAddr.create8bit(0x42));
        colorBeacon.setI2cAddress(I2cAddr.create8bit(0x24));

        waitForStart();

        time = new ElapsedTime();
        time.reset();

        /*while (time.time() < 10.0) {
            telemetry.addData("LightF: ", colorF.alpha());
            telemetry.addData("LightB: ", colorB.alpha());
            telemetry.update();
        }*/

        while (colorB.alpha() < 5 && opModeIsActive()) {
            move(.105, .105);
        }
        move(0, 0);
        telemetry.addData("waiting", "...");
        sleep(3000);
        //Assuming the back color sensor is at robot pivot point
        while (colorF.alpha() < 10 && opModeIsActive()) {
            move(.115, -.115);
        }

        move(0, 0);

        sleep(1000);
        lineFollow();
        pressBeacon();

        /* moveTime(500, -1, -1);
        moveTime(250, 1, -1);

        while (colorB.alpha() < 5 && opModeIsActive()) {
            move(.115, .115);
        }

        while (colorF.alpha() < 5 && opModeIsActive()) {
            move(-.115, .115);
        }

        lineFollow(colorBeacon.blue() > 4 || colorBeacon.red() > 4);
        pressBeacon();
        moveTime(250,-.08, .08);

        while (colorF.alpha() < 5 && opModeIsActive()){
            move(-.115, .115);
        }

        moveTime(2000, 1, 1); */

    }

    public void lineFollow() throws InterruptedException {
        boolean isLeft = true;

        while (opModeIsActive() && colorBeacon.red() <= 3 && colorBeacon.blue() <= 3) {

            while (colorF.alpha() > 10 && colorB.alpha() > 10 && colorBeacon.red() <= 3 && colorBeacon.blue() <= 3 && opModeIsActive()) {
                move(.135, .135);
                telemetry.addData("moving", " forward");
                telemetry.addData("colorF: ", colorF.alpha());
                telemetry.addData("colorB: ", colorB.alpha());
                DbgLog.error("going straight");
            }

            if (colorF.alpha() < 10) {
                DbgLog.error("front not on line");
                if (isLeft) {
                    while (colorF.alpha() < 10) {
                        move(.115, -.115);
                        DbgLog.error("turning right");
                    }
                    move(0, 0);
                    isLeft = false;

                } else {
                    while (colorF.alpha() < 10) {
                        move(-.115, .115);
                        DbgLog.error("turning left");
                    }
                    move(0, 0);
                    DbgLog.error("it's fixed");
                    isLeft = true;
                    //This ain't right
                }
            }

            if (colorB.alpha() < 10) {
                DbgLog.error("back not on line");
                if (isLeft) {
                    while (colorF.alpha() < 10) {
                        move(.115, -.115);
                        DbgLog.error("turning right");
                    }
                    move(0, 0);
                    while (colorB.alpha() < 10) {
                        move(.115, .115);
                    }
                    move(0, 0);
                    while (colorF.alpha() < 10) {
                        move(-.115, .115);
                        DbgLog.error("turning left");
                    }
                    move(0, 0);
                    isLeft = false;
                } else {
                    while (colorF.alpha() < 10) {
                        move(-.115, .115);
                        DbgLog.error("turning left");
                    }
                    move(0, 0);
                    while (colorB.alpha() < 10) {
                        move(.115, .115);
                    }
                    move(0, 0);
                    while (colorF.alpha() < 10) {
                        move(.115, -.115);
                        DbgLog.error("turning right");
                    }
                    move(0, 0);
                    isLeft = true;
                }
            }
            telemetry.addData("RED: " + colorBeacon.red(), "      BLUE: " + colorBeacon.blue());
            DbgLog.error("RED: " + colorBeacon.red() + "     BLUE: " + colorBeacon.blue());
        }
        move(0, 0);
    }

    public void pressBeacon() throws InterruptedException {
        if (!opModeIsActive())
            return;

        if (colorBeacon.red() > 3) {
            time.reset();
            while (time.time() < 3) {
                telemetry.addData("Left side:", " is red");
                DbgLog.error("RED RED RED RED RED");
            }
        }

        else if (colorBeacon.blue() > 3) {
            time.reset();
            while (time.time() < 3) {
                telemetry.addData("Left side:", " is blue");
                DbgLog.error("BLUE BLUE BLUE BLUE BLUE");
            }
        }

        else {
            time.reset();
            while (time.time() < 3) {
                telemetry.addData("LOW BLUE " + colorBeacon.blue(),    "LOW RED " + colorBeacon.red());
                DbgLog.error("LOW BLUE " + colorBeacon.blue(),"       LOW RED " + colorBeacon.red());
            }
        }
    }

    public void move(double leftSpeed, double rightSpeed) throws InterruptedException{
        if (!opModeIsActive())
            return;
        motorBL.setPower(leftSpeed);
        motorBR.setPower(-rightSpeed);
        motorFL.setPower(leftSpeed);
        motorFR.setPower(-rightSpeed);

        //telemetry.addData("back: ", colorB.alpha());
        //telemetry.addData("front: ", colorF.alpha());
    }

    public void moveTime(int msTime, double leftSpeed, double rightSpeed) throws InterruptedException{
        if (!opModeIsActive())
            return;
        motorBL.setPower(leftSpeed);
        motorBR.setPower(-rightSpeed);
        motorFL.setPower(leftSpeed);
        motorFR.setPower(-rightSpeed);

        sleep(msTime);

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        //telemetry.addData("back: ", colorB.alpha());
        //telemetry.addData("front: ", colorF.alpha());
    }

}
