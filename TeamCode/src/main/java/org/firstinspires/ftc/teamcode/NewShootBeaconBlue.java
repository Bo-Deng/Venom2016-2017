package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Navya on 11/28/2016.
 */

@Autonomous(name = "NewShootBeaconBlue", group = "Autonomous")
public class NewShootBeaconBlue extends LinearOpMode {

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorM;
    DcMotor motorLaunchL;
    DcMotor motorLaunchR;
    DcMotor motorLift;

    Servo servoButtonAuto;
    Servo servoCapTop;
    CRServo servoButtonL;
    CRServo servoButtonR;
    //CRServo servoCapL;
    //CRServo servoCapR;
    Servo servoCapL;
    Servo servoCapR;
    Servo servoLaunch;

    ColorSensor colorF;
    ColorSensor colorB;
    ColorSensor colorBeacon;
    IMU imu;

    double squaresToEncoder = 1084;

    double targetPower = 0.0;
    double shootPower = 0.0;
    double rRatio = 1;//0.905;
    double motorMultiplier = 1.0;

    ElapsedTime time = new ElapsedTime();


    public void initStuff() throws InterruptedException {

        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
        motorM = hardwareMap.dcMotor.get("motorM");
        motorLaunchL = hardwareMap.dcMotor.get("motorLaunchL");
        motorLaunchR = hardwareMap.dcMotor.get("motorLaunchR");
        motorLaunchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLaunchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLift = hardwareMap.dcMotor.get("motorLift");

        servoButtonAuto = hardwareMap.servo.get("servoButtonAuto");
        servoCapTop = hardwareMap.servo.get("servoCapTop");
        servoButtonL = hardwareMap.crservo.get("servoButtonL");
        servoButtonR = hardwareMap.crservo.get("servoButtonR");
        //servoCapL = hardwareMap.crservo.get("servoCapL");
        //servoCapR = hardwareMap.crservo.get("servoCapR");
        servoCapL = hardwareMap.servo.get("servoCapL");
        servoCapR = hardwareMap.servo.get("servoCapR");
        servoLaunch = hardwareMap.servo.get("servoLaunch");

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

        servoButtonAuto.setPosition(.3);
        servoCapTop.setPosition(0);
        servoCapL.setPosition(1);
        servoLaunch.setPosition(.8);

        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        //motorMultiplier = voltage
        telemetry.addData("init", " complete");
        telemetry.update();
    }

    public void runOpMode() throws InterruptedException {
        initStuff();
        waitForStart();
        servoCapTop.setPosition(.5);
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        targetPower = -0.144 * voltage + 2.65;

        moveSquares(1, .5);
        while (shootPower < targetPower) {
            shootPower = Range.clip(shootPower + .1, 0, targetPower);
            motorLaunchL.setPower(shootPower);
            motorLaunchR.setPower(-shootPower);
            sleep(50);
            //warm up launcher
        }
        motorM.setPower(1);
        sleep(2500);
        motorM.setPower(0);
        motorLaunchL.setPower(0);
        motorLaunchR.setPower(0);
        moveSquares(-.725, .5);
        PDturn(48, 2550);
        moveSquares(1.40, 1);
        stopMotors();
        sleep(50);
        move(.156, .156);
        while (colorB.alpha() < 1 && opModeIsActive()) {
        }
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(200);
        move(.402, -.402);
        while (colorF.alpha() < 3 && opModeIsActive()) {
        }
        stopMotors();
        sleep(100);
        move(.20, .20);
        while (colorBeacon.blue() <= 3 && colorBeacon.red() <= 3 && opModeIsActive()) {
        }
        move(0, 0);
        pressBeacon();
        PDturn(0, 2400);
        moveSquares(1.25, 1);
        stopMotors();
        sleep(150);
        move(.160, .160);
        while (colorB.alpha() < 1 && opModeIsActive()) {
        }
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(200);
        move(.41, -.413); //hackedd
        while (colorF.alpha() < 6 && opModeIsActive()) {
        }
        stopMotors();
        sleep(100);
        move(.225, .225);
        while (colorBeacon.blue() <= 3 && colorBeacon.red() <= 3 && opModeIsActive()) {
        }
        stopMotors();
        pressBeacon();
        //moveSquares(-1.5, 1);
        //moveTime(2000, 1, -1);
    }

    public void moveSquares(double squares, double pow) throws InterruptedException {
        resetEncoders();
        double encoderVal = squares * squaresToEncoder;
        if (squares >= 0) {
            while (motorBL.getCurrentPosition() < encoderVal && opModeIsActive()) {
                motorBL.setPower(-pow);
                motorBR.setPower(-pow);
                motorFL.setPower(pow);
                motorFR.setPower(pow);
            }
        }
        else if (squares < 0) {
            while (motorBL.getCurrentPosition() > encoderVal && opModeIsActive()) {
                motorBL.setPower(pow);
                motorBR.setPower(pow);
                motorFL.setPower(-pow);
                motorFR.setPower(-pow);
            }
        }
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
    }

    public void resetEncoders() throws InterruptedException {
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void move(double leftSpeed, double rightSpeed) throws InterruptedException{
        if (!opModeIsActive())
            return;
        motorBL.setPower(-leftSpeed * motorMultiplier);
        motorBR.setPower(-rightSpeed * motorMultiplier);
        motorFL.setPower(leftSpeed * motorMultiplier);
        motorFR.setPower(rightSpeed * motorMultiplier);

        //telemetry.addData("back: ", colorB.alpha());
        //telemetry.addData("front: ", colorF.alpha());
    }

    public void stopMotors() {
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        DbgLog.error("MOTORS ARE STOPPED PLZ STOP");
    }

    public void PDturn(double degTurn, int msTime) throws InterruptedException {

        double kP = 0.055;
        double kd = 1;
        double prevError = 0;
        double currError = 0;
        double prevtime = time.milliseconds();
        double currTime = time.milliseconds();
        double PIDchange;
        double rightSpeed;
        double leftSpeed;
        double angleDiff;

        time.reset();
        while (time.milliseconds() < msTime && opModeIsActive()) {
            telemetry.addData("msTime: ", msTime);
            telemetry.addData("imuYaw: ", imu.getYaw());
            telemetry.update();
            angleDiff = degTurn - imu.getYaw();
            currError = angleDiff;
            currTime = time.milliseconds();
            leftSpeed = 0;
            rightSpeed = 0;

            PIDchange = -angleDiff * kP - (currError - prevError) / (currTime - prevtime) * kd;
            leftSpeed = Range.clip(-(PIDchange / 2), -1, 1);
            rightSpeed = Range.clip(PIDchange / 2, -1, 1);

            prevError = currError;
            prevtime = currTime;
            move(leftSpeed, rightSpeed);
            DbgLog.error("" + (msTime - time.milliseconds()));
        }
        move(0, 0);
        telemetry.addData("turn", " completed");
        telemetry.update();
        sleep(500);
        DbgLog.error("ANGLE: " + imu.getYaw());
    }

    public void turn(double turnAngle) throws InterruptedException { // -179.9999 to 180 deg
        if (!opModeIsActive())
            return;

        if (turnAngle > 0) { //turn right
            DbgLog.error("turnAngle > 0");
            move(.550, -.550);
            while (imu.getYaw() < turnAngle - 3 && opModeIsActive()) {
            }
            move(0, 0);
        }
        else if (turnAngle < 0) {  //turn left
            DbgLog.error("turnAngle < 0");
            move(-.550, .550);
            while (imu.getYaw() > turnAngle + 3 && opModeIsActive()) {
            }
            move(0, 0);
        }
        sleep(400);
        DbgLog.error("Done turning: ");
        DbgLog.error("" + imu.getYaw());
    }

    public void pressBeacon() throws InterruptedException {
        if (!opModeIsActive())
            return;
        DbgLog.error(colorBeacon.red() + "    " + colorBeacon.blue());
        if (colorBeacon.blue() > 3) {
            telemetry.addData("Right:", " is blue");
            servoButtonAuto.setPosition(.48);
            DbgLog.error("BLUE BLUE BLUE BLUE BLUE BLUE");
            sleep(300);
        }

        else if (colorBeacon.red() > 3) {
            telemetry.addData("Right:", " is red");
            servoButtonAuto.setPosition(.15);
            DbgLog.error("RED RED RED RED RED RED");
            sleep(300);
        }
        moveTime(1200, .6, .6);
        sleep(100);
        //moveTime(1200, -.6, -.6);
        moveSquares(-.25, .5);
    }
}