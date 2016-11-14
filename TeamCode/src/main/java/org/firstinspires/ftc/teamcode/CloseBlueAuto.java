package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "CloseBlueAuto", group = "Auto")
public class CloseBlueAuto extends LinearOpMode {

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorM;
    DcMotor motorLaunchL;
    DcMotor motorLaunchR;
    DcMotor motorLift;

    Servo servoButtonAuto;
    CRServo servoButtonL;
    CRServo servoButtonR;
    CRServo servoCapL;
    CRServo servoCapR;


    ColorSensor colorF;
    ColorSensor colorB;
    ColorSensor colorBeacon;
    IMU imu;

    ElapsedTime time;

    @Override
    public void runOpMode() throws InterruptedException {

        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorM = hardwareMap.dcMotor.get("motorM");
        motorLaunchL = hardwareMap.dcMotor.get("motorLaunchL");
        motorLaunchR = hardwareMap.dcMotor.get("motorLaunchR");
        motorLift = hardwareMap.dcMotor.get("motorLift");

        servoButtonAuto = hardwareMap.servo.get("servoButtonAuto");
        servoButtonL = hardwareMap.crservo.get("servoButtonL");
        servoButtonR = hardwareMap.crservo.get("servoButtonR");
        servoCapL = hardwareMap.crservo.get("servoCapL");
        servoCapR = hardwareMap.crservo.get("servoCapR");

        colorF = hardwareMap.colorSensor.get("colorF");
        colorB = hardwareMap.colorSensor.get("colorB");
        colorB.setI2cAddress(I2cAddr.create8bit(0x42));
        colorBeacon = hardwareMap.colorSensor.get("colorBeacon");
        colorBeacon.setI2cAddress(I2cAddr.create8bit(0x24));
        colorBeacon.enableLed(false);

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "IMU"));
        imu.IMUinit(hardwareMap);

        //motorLaunchL.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        servoButtonAuto.setPosition(.25);
        telemetry.addData("init ", "complete");
        waitForStart();

        move(.25, .225);
        sleep(75);            //warms up motors on the way to beacon
        move(.5, .45);
        sleep(75);
        move(1, .9);
        sleep(750);
        while (colorB.alpha() < 3 && opModeIsActive()) {  //
            move(.275, .26);
        }
        move(0, 0);
        sleep(150);
        telemetry.addData("WAITING WAITING WAITING WAITING", "...");
        //Assuming the back color sensor is at robot pivot point
        while (colorF.alpha() < 3 && opModeIsActive()) {
            move(.475, -.475);
        }
        //DbgLog.error("IMUyaw: " + imu.getYaw());
        move(0, 0);
        DbgLog.error("LINE FOLLOWING STARTS HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        sleep(400);
        //lineFollow(false);
        move(.265, .28);
        while (colorBeacon.blue() <= 3 && colorBeacon.red() <= 3) {
            //DbgLog.error(colorBeacon.red() + "        " + colorBeacon.blue());
        }
        move(0, 0);
        sleep(3000);
        pressBeacon();
/*
        moveTime(158, -1, .85);

        turn(90);

        moveTime(50, .125, .125);
        moveTime(50, .25, .25);
        moveTime(500, .5, .5);

        move(.415, .415);
        while (colorB.alpha() < 3 && opModeIsActive()) {
        }
        move(0, 0);

        sleep(100);

        while (colorF.alpha() < 3 && opModeIsActive()) {
            move(-.550, .550);
        }
        move(0, 0);
        sleep(50);
        lineFollow(true);
        pressBeacon();
*/
       /* moveTime(250, -.08, .08);

        while (colorF.alpha() < 5 && opModeIsActive()){
            move(-.115, .115);
        }

        moveTime(2000, 1, 1); */

    }

    public void lineFollow(boolean left) throws InterruptedException {
        boolean isLeft = left;

        while (opModeIsActive() && colorBeacon.red() <= 3 && colorBeacon.blue() <= 3) {

            while (colorF.alpha() >= 10 && colorB.alpha() >= 10 && colorBeacon.red() <= 3 && colorBeacon.blue() <= 3 && opModeIsActive()) {
                move(.40, .45);
                telemetry.addData("moving", " forward");
                telemetry.addData("colorF: ", colorF.alpha());
                telemetry.addData("colorB: ", colorB.alpha());
                //DbgLog.error("going straight");
            }

            if (colorF.alpha() < 10) {
                DbgLog.error("front not on line");
                if (isLeft) {
                    while (colorF.alpha() < 10 && opModeIsActive()) {
                        move(.520, -.575);
                        //DbgLog.error("turning right");
                    }
                    move(0, 0);
                    isLeft = false;
                    DbgLog.error("turning right");

                } else {
                    while (colorF.alpha() < 10 && opModeIsActive()) {
                        move(-.520, .575);
                        //DbgLog.error("turning left");
                    }
                    move(0, 0);
                    DbgLog.error("turning left");
                    //DbgLog.error("it's fixed");
                    isLeft = true;
                    //This ain't right
                }// rice was here
            }

            if (colorB.alpha() < 10) {

                DbgLog.error("back not on line");
                if (isLeft) {
                    while (colorF.alpha() < 10 && opModeIsActive()) {
                        move(.420, -.575);
                    }
                    move(0, 0);
                    while (colorB.alpha() < 10 && opModeIsActive()) {
                        move(.420, .575);
                    }
                    move(0, 0);
                    while (colorF.alpha() < 10 && opModeIsActive()) {
                        move(.420, -.575);
                    }
                    move(0, 0);
                    isLeft = false;
                } else {
                    while (colorF.alpha() < 10 && opModeIsActive()) {
                        move(-.420, .575);
                    }
                    move(0, 0);
                    DbgLog.error("turning left");
                    sleep(400);

                    while (colorB.alpha() < 10 && opModeIsActive()) {
                        move(.420, .575);
                    }
                    move(0, 0);
                    DbgLog.error("moving straight");
                    sleep(400);

                    while (colorF.alpha() < 10 && opModeIsActive()) {
                        move(-.420, .575);
                    }
                    move(0, 0);
                    DbgLog.error("turning left");
                    sleep(400);

                    isLeft = true;
                }
            }
        }
        move(0, 0);
    }

    public void pressBeacon() throws InterruptedException {
        if (!opModeIsActive())
            return;

        if (colorBeacon.red() > 3) {
            telemetry.addData("Right:", " is red");
            servoButtonAuto.setPosition(.5);
            DbgLog.error("RED RED RED RED RED");
        }

        else if (colorBeacon.blue() > 3) {
            telemetry.addData("Right:", " is blue");
            servoButtonAuto.setPosition(.75);
            DbgLog.error("BLUE BLUE BLUE BLUE BLUE");
        }
        sleep(100);
        moveTime(1200, .6, .6);
        sleep(200);
        moveTime(1200, -.6, -.6);
    }

    public void move(double leftSpeed, double rightSpeed) throws InterruptedException{
        if (!opModeIsActive())
            return;
        motorBL.setPower(-leftSpeed);
        motorBR.setPower(-rightSpeed);
        motorFL.setPower(leftSpeed);
        motorFR.setPower(rightSpeed);

        //telemetry.addData("back: ", colorB.alpha());
        //telemetry.addData("front: ", colorF.alpha());
    }

    public void moveTime(int msTime, double leftSpeed, double rightSpeed) throws InterruptedException{
        if (!opModeIsActive())
            return;

        move(leftSpeed, rightSpeed);

        sleep(msTime);

        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);

        //telemetry.addData("back: ", colorB.alpha());
        //telemetry.addData("front: ", colorF.alpha());
    }

    public void turn(double turnAngle) throws InterruptedException { // -179.9999 to 180 deg
        if (!opModeIsActive())
            return;

        imu.IMUinit(hardwareMap);                                    // negative is clockwise positive is counter
        DbgLog.error("" + imu.getYaw());
        if (turnAngle > 0) { //turn left
            DbgLog.error("turnAngle > 0");
            move(-.500, .500);
            while (imu.getYaw() < turnAngle - 10) {
            }
            move(0, 0);
        }
        else if (turnAngle < 0) {  //turn right
            DbgLog.error("turnAngle < 0");
            move(.500, -.500);
            while (imu.getYaw() > turnAngle + 10) {
            }
            move(0, 0);
        }
        //sleep(400);
        //DbgLog.error("Done turning: ");
        //DbgLog.error("" + imu.getYaw());
    }
}