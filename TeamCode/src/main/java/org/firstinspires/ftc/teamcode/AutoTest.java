package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
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
    IMU imu;

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

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "IMU"));
        imu.IMUinit(hardwareMap);

        time = new ElapsedTime();
        telemetry.addData("Init", " completed");
        DbgLog.error("init complete");
        waitForStart();

        move(.125, .125);
        sleep(50);
        move(.25, .25);
        sleep(50);
        move(.5, .5);
        sleep(50);
        move(1, 1);
        sleep(750);
        while (colorB.alpha() < 3 && opModeIsActive()) {
            move(.100, .100);
        }
        //sleep(50); //ensures that it overshoots, so that it will start facing right
        move(0, 0);
        sleep(250);
        telemetry.addData("WAITING WAITING WAITING WAITING", "...");
        //Assuming the back color sensor is at robot pivot point
        while (colorF.alpha() < 3 && opModeIsActive()) {
            move(.115, -.115);
        }

        move(0, 0);
        lineFollow(false);
        pressBeacon();

        moveTime(50, -1, -1);

        turn(-90);

        moveTime(508, .25, .25);

        while (colorB.alpha() < 3 && opModeIsActive()) {
            move(.115, .115);
        }

        while (colorF.alpha() < 3 && opModeIsActive()) {
            move(-.115, .115);
        }

        lineFollow(true);
        pressBeacon();

       /* moveTime(250, -.08, .08);

        while (colorF.alpha() < 5 && opModeIsActive()){
            move(-.115, .115);
        }

        moveTime(2000, 1, 1); */

    }

    public void lineFollow(boolean left) throws InterruptedException {
        boolean isLeft = left;

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
                    while (colorF.alpha() < 10 && opModeIsActive()) {
                        move(.115, -.115);
                        DbgLog.error("turning right");
                    }
                    move(0, 0);
                    isLeft = false;

                } else {
                    while (colorF.alpha() < 10 && opModeIsActive()) {
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
                    while (colorB.alpha() < 10 && opModeIsActive()) {
                        move(.115, .115);
                    }
                    move(0, 0);
                    while (colorF.alpha() < 10 && opModeIsActive()) {
                        move(-.115, .115);
                        DbgLog.error("turning left");
                    }
                    move(0, 0);
                    isLeft = true;
                } else {
                    move(0, 0);
                    while (colorB.alpha() < 10 && opModeIsActive()) {
                        move(.115, .115);
                    }
                    move(0, 0);
                    while (colorF.alpha() < 10 && opModeIsActive()) {
                        move(.115, -.115);
                        DbgLog.error("turning right");
                    }
                    move(0, 0);
                    isLeft = false;
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
            while (time.time() < 3 && opModeIsActive()) {
                telemetry.addData("Left side:", " is red");
                DbgLog.error("RED RED RED RED RED");
            }
        }

        else if (colorBeacon.blue() > 3) {
            time.reset();
            while (time.time() < 3 && opModeIsActive()) {
                telemetry.addData("Left side:", " is blue");
                DbgLog.error("BLUE BLUE BLUE BLUE BLUE");
            }
        }

        else {
            time.reset();
            while (time.time() < 3 && opModeIsActive()) {
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

    public void turn(double turnAngle) throws InterruptedException { // -179.9999 to 180 deg
        imu.IMUinit(hardwareMap);                                    // negative is clockwise positive is counter
        DbgLog.error("" + imu.getYaw());
        if (turnAngle > 0) { //turn left
            DbgLog.error("turnAngle > 0");
            move(-.25, .25);
            while (imu.getYaw() < turnAngle - 30) {
            }
            move(0, 0);
        }
        else if (turnAngle < 0) {  //turn right
            DbgLog.error("turnAngle < 0");
            move(.25, -.25);
            while (imu.getYaw() > turnAngle + 30) {
            }
            move(0, 0);
        }
        sleep(500);
        DbgLog.error("Done turning: ");
        DbgLog.error("" + imu.getYaw());
    }
}