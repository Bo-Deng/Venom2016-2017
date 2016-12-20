package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "EncoderTest", group = "Autonomous")
public class EncoderTest extends LinearOpMode {
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
    double rRatio = 1;

    double squaresToEncoder = 1084;
    ElapsedTime time = new ElapsedTime();

    public void initStuff() throws InterruptedException{

        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        servoButtonAuto.setPosition(.4);
        servoCapTop.setPosition(0);
        servoCapL.setPosition(1);
        servoLaunch.setPosition(.8);

        double squaresToEncoder = 0.0;

        telemetry.addData("init", " complete");
        telemetry.update();
    }

    public void runOpMode() throws InterruptedException {
        initStuff();
        waitForStart();
        servoCapTop.setPosition(.5);
        servoLaunch.setPosition(.8);
        moveSquares(2, .5);
        //fluffybunny
    /*turn(-38);
    moveTime(50, .25, .25);
    moveTime(50, .5, .5);
    moveTime(300, 1, 1);
    move(.375, .375);
    while (colorB.alpha() < 3 && opModeIsActive()) {
    }
    move(0, 0);
    DbgLog.error("back sensed white line");
    sleep(500);
    move(-.45, .45);
    while (colorF.alpha() < 3 && opModeIsActive()) {
    }
    move(0, 0);
    sleep(100);
    move(.275, .275);
    while (colorBeacon.blue() <= 3 && colorBeacon.red() <= 3) {
    }
    move(0, 0);
    pressBeacon(); */
    }

    public void moveSquares(double squares, double pow) {
        double encoderVal = squares * squaresToEncoder;
        while (motorBL.getCurrentPosition() < encoderVal && opModeIsActive()) {
            motorBL.setPower(-pow);
            motorBR.setPower(-pow);
            motorFL.setPower(pow);
            motorFR.setPower(pow);
        }
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
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

        imu.IMUinit(hardwareMap);                                    // negative is clockwise positive is counter
        if (turnAngle > 0) { //turn left
            DbgLog.error("turnAngle > 0");
            move(.550, -.550);
            while (imu.getYaw() < turnAngle - 3 && opModeIsActive()) {
            }
            move(0, 0);
        }
        else if (turnAngle < 0) {  //turn right
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
            servoButtonAuto.setPosition(.5);
            DbgLog.error("BLUE BLUE BLUE BLUE BLUE BLUE");
        }

        else if (colorBeacon.red() > 3) {
            telemetry.addData("Right:", " is red");
            servoButtonAuto.setPosition(.75);
            DbgLog.error("RED RED RED RED RED RED");
        }
        sleep(100);
        moveTime(1200, .6, .6);
        sleep(200);
        //moveTime(1200, -.6, -.6);
        moveTime(1200, -.6, -.6);
    }
}
