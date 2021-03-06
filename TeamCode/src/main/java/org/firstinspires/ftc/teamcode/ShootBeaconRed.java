package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Bo on 11/28/2016.
 */
@Disabled
@Autonomous(name = "ShootBeaconRed", group = "Autonomous")
public class ShootBeaconRed extends LinearOpMode {

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

    double targetPower = 0.0;
    double shootPower = 0.0;
    double rRatio = 1;//0.905;
    ElapsedTime time = new ElapsedTime();

    public void initStuff() {

        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        telemetry.addData("init", " complete");
        telemetry.update();
    }

    public void runOpMode() throws InterruptedException {
        initStuff();
        waitForStart();
        servoCapTop.setPosition(.5);
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        targetPower = -0.144 * voltage + 2.65;

        moveTime(1200, .5, .5);
        moveTime(50, .25, .25);
        moveTime(50, .125, .125);
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
        moveTime(900, -.5, -.5);
        turn(-36);
        moveTime(50, .25, .25);
        moveTime(50, .5, .5);
        moveTime(600, 1, 1);
        move(.225, .225);
        while (colorB.alpha() < 3 && opModeIsActive()) {
        }
        move(0, 0);
        DbgLog.error("back sensed white line");
        sleep(2000);
        move(-.42, .42);
        while (colorF.alpha() < 3 && opModeIsActive()) {
        }
        move(0, 0);
        sleep(100);
        move(.275, .275);
        while (colorBeacon.blue() <= 3 && colorBeacon.red() <= 3 && opModeIsActive()) {
        }
        move(0, 0);
        pressBeacon();
        moveTime(750, -1, -1);
        moveTime(2000, 1, -1);
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
        motorBL.setPower(-leftSpeed);
        motorBR.setPower(-rightSpeed * rRatio);
        motorFL.setPower(leftSpeed);
        motorFR.setPower(rightSpeed * rRatio);

        //telemetry.addData("back: ", colorB.alpha());
        //telemetry.addData("front: ", colorF.alpha());
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
            servoButtonAuto.setPosition(.15);
            DbgLog.error("BLUE BLUE BLUE BLUE BLUE BLUE");
        }

        else if (colorBeacon.red() > 3) {
            telemetry.addData("Right:", " is red");
            servoButtonAuto.setPosition(.45);
            DbgLog.error("RED RED RED RED RED RED");
        }
        else {
        }
        sleep(300);
        moveTime(1200, .6, .6);
        sleep(200);
        //moveTime(1200, -.6, -.6);
        moveTime(1200, -.6, -.6);
    }
}
