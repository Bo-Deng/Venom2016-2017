package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Bo on 1/4/2017.
 */
public class CustomLinearOpMode extends LinearOpMode {

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
    Servo servoB;
    CRServo servoSweep;
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
    double motorMultiplier = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void initStuff(HardwareMap map) throws InterruptedException {

        motorFR = map.dcMotor.get("motorFR");
        motorFL = map.dcMotor.get("motorFL");
        motorBR = map.dcMotor.get("motorBR");
        motorBL = map.dcMotor.get("motorBL");
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
        motorM = map.dcMotor.get("motorM");
        motorLaunchL = map.dcMotor.get("motorLaunchL");
        motorLaunchR = map.dcMotor.get("motorLaunchR");
        motorLaunchL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLaunchR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorLift = map.dcMotor.get("motorLift");

        servoButtonAuto = map.servo.get("servoButtonAuto");
        servoCapTop = map.servo.get("servoCapTop");
        servoButtonL = map.crservo.get("servoButtonL");
        servoButtonR = map.crservo.get("servoButtonR");
        //servoCapL = map.crservo.get("servoCapL");
        //servoCapR = map.crservo.get("servoCapR");
        servoCapL = map.servo.get("servoCapL");
        servoCapR = map.servo.get("servoCapR");
        servoLaunch = map.servo.get("servoLaunch");
        servoB = hardwareMap.servo.get("servoB");
        servoSweep = hardwareMap.crservo.get("servoSweep");

        colorF = map.colorSensor.get("colorF");
        colorB = map.colorSensor.get("colorB");
        colorB.setI2cAddress(I2cAddr.create8bit(0x42));
        colorBeacon = map.colorSensor.get("colorBeacon");
        colorBeacon.setI2cAddress(I2cAddr.create8bit(0x24));
        colorBeacon.enableLed(false);

        imu = new IMU(map.get(BNO055IMU.class, "IMU"));
        imu.IMUinit(map);

        //motorLaunchL.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        servoButtonAuto.setPosition(.3);
        servoCapTop.setPosition(0);
        servoCapL.setPosition(1);
        servoLaunch.setPosition(.8);
        servoB.setPosition(0.5);

        double voltage = map.voltageSensor.get("Motor Controller 2").getVoltage();
        //motorMultiplier = voltage
        telemetry.addData("init", " complete");
        telemetry.update();
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

    public void move(double leftSpeed, double rightSpeed) throws InterruptedException {
        if (!opModeIsActive())
            return;
        motorBL.setPower(-leftSpeed * motorMultiplier);
        motorBR.setPower(-rightSpeed * motorMultiplier);
        motorFL.setPower(leftSpeed * motorMultiplier);
        motorFR.setPower(rightSpeed * motorMultiplier);

        //telemetry.addData("back: ", colorB.alpha());
        //telemetry.addData("front: ", colorF.alpha());
    }

    public void stopMotors() throws InterruptedException{
        //move(.2, .2);
        //sleep(20);
        motorBL.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);
        DbgLog.error("MOTORS ARE STOPPED PLZ STOP");
    }

    public void sleep(int ms) {
        try {
            Thread.sleep(ms);
        }
        catch (Exception E) {
        }
    }

}
