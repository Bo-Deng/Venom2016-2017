package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOpOfficial", group = "TeleOp")
public class TeleOpOFFICIAL extends OpMode {

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

    int warmUpMs = 88;
    double launcherSpeed = 0.0;
    double driveScale = 1.0;
    ElapsedTime time = new ElapsedTime();

    // Maps the motors and sets them in the correct direction.
    public void init() {

        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorM = hardwareMap.dcMotor.get("motorM");
        motorLaunchL = hardwareMap.dcMotor.get("motorLaunchL");
        motorLaunchR = hardwareMap.dcMotor.get("motorLaunchR");
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

        servoButtonAuto.setPosition(.5);
        servoCapTop.setPosition(0);
        servoCapL.setPosition(1);
        servoLaunch.setPosition(.79);
    }

    @Override
    public void loop() {

        double g1_leftY = -gamepad1.left_stick_y;  //up is changed to 1 and down to -1
        double g1_rightY = -gamepad1.right_stick_y;
        double g1_rightTrigger = gamepad1.right_trigger;
        double g1_leftTrigger = gamepad1.left_trigger;

        boolean g1_leftBumper = gamepad1.left_bumper;
        boolean g1_rightBumper = gamepad1.right_bumper;
        boolean g1_y = gamepad1.y;

        double g2_leftY = -gamepad2.left_stick_y;
        double g2_rightY = -gamepad2.right_stick_y;
        double g2_rightTrigger = gamepad2.right_trigger;

        boolean g2_leftBumper = gamepad2.left_bumper;
        boolean g2_rightBumper = gamepad2.right_bumper;
        boolean g2_Dleft = gamepad2.dpad_left;
        boolean g2_Dright = gamepad2.dpad_right;
        boolean g2_Dup = gamepad2.dpad_up;
        boolean g2_Ddown = gamepad2.dpad_down;
        boolean g2_x = gamepad2.x;
        boolean g2_b = gamepad2.b;
        boolean g2_y = gamepad2.y;

        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();

        if (g1_y) {
            if (driveScale == 1) {
                driveScale = 0.5;
                try {
                    wait(350);
                }
                catch (Exception E) {
                }
            }
            else if (driveScale == 0.5) {
                driveScale = 1;
                try {
                    wait(350);
                }
                catch (Exception E){
                }
            }
        }
        if (Math.abs(g1_leftY) > 0.1) {

            motorBL.setPower(-g1_leftY * driveScale);
            motorFL.setPower(g1_leftY * driveScale);
        }
        else {
            motorBL.setPower(0);
            motorFL.setPower(0);
        }

        if (Math.abs(g1_rightY) > 0.1) {

            motorBR.setPower(-g1_rightY * driveScale);
            motorFR.setPower(g1_rightY * driveScale);
        }
        else {
            motorBR.setPower(0);
            motorFR.setPower(0);
        }

        if (Math.abs(g2_rightY) > 0.1) {
            motorLift.setPower(-g2_rightY);
        }
        else {
            motorLift.setPower(0);
        }

        if (g2_leftY < -0.1) {
            motorM.setPower(1);
        }
        else if (g2_leftY > 0.1) {
            motorM.setPower(-1);
        }
        else {
            motorM.setPower(0);
        }

        if (g2_rightTrigger > 0.1) {
            launcherSpeed = Range.clip(launcherSpeed + .1, -1, 1);
            servoLaunchUp();
            try {
                Thread.sleep(50);
            }
            catch (Exception E){
            }
        }
        else if (g2_y) {
            launcherSpeed = Range.clip(launcherSpeed - .1, -1, 1);
            try {
                Thread.sleep(50);
            }
            catch (Exception E){
            }
        }
        else {
            if (launcherSpeed > 0) {
                launcherSpeed = Range.clip(launcherSpeed - .1, 0, 1);
                servoLaunchDown();
                try {
                    Thread.sleep(50);
                } catch (Exception E) {
                }
            }
            else if (launcherSpeed < 0) {
                launcherSpeed = Range.clip(launcherSpeed + .1, -1, 0);
                try {
                    Thread.sleep(50);
                } catch (Exception E) {
                }
            }
            else {
                launcherSpeed = 0;
            }
        }
        motorLaunchL.setPower(launcherSpeed);
        motorLaunchR.setPower(-launcherSpeed);


        if (g1_rightTrigger > 0.1) {
            servoButtonL.setPower(1);
        }
        else if (g1_rightBumper) {
            servoButtonL.setPower(-1);
        }
        else {
            servoButtonL.setPower(0);
        }

        if (g1_leftTrigger > 0.1) {
            servoButtonR.setPower(-1);
        }
        else if (g1_leftBumper) {
            servoButtonR.setPower(1);
        }
        else {
            servoButtonR.setPower(0);
        }

        if (g2_leftBumper) {
            servoLaunch.setPosition(Range.clip(servoLaunch.getPosition() + .01, 0, 1));
        }
        else if (g2_rightBumper) {
            servoLaunch.setPosition(Range.clip(servoLaunch.getPosition() - .01, 0, 1));
        }
        /*if (g2_rightBumper) {
            servoCapL.setPosition(0);
            servoCapR.setPosition(0);
        }
        else if (g2_leftBumper) {
            servoCapL.setPosition(1);
            servoCapR.setPosition(1);
        }
        else {
            servoCapL.setPosition(0.5);
            servoCapR.setPosition(0.5);
        } */

        /*
        if (g2_Dleft) {
            servoCapL.setPower(1);
        }
        else if (g2_Dright) {
            servoCapL.setPower(-1);
        }
        else {
            servoCapL.setPower(0);
        }

        if (g2_x) {
            servoCapR.setPower(1);
        }
        else if (g2_b) {
            servoCapR.setPower(-1);
        }
        else {
            servoCapR.setPower(0);
        }

        if (g2_Dup) {
            servoCapTop.setPosition(1);
        }
        else if (g2_Ddown) {
            servoCapTop.setPosition(0);
        }
        */

        if (g2_Dleft) {
            servoCapL.setPosition(1);
        }
        else if (g2_Dright) {
            servoCapL.setPosition(0);
        }

        if (g2_x) {
            servoCapR.setPosition(1);
        }
        else if (g2_b) {
            servoCapR.setPosition(0);
        }

        if (g2_Dup) {
            servoCapTop.setPosition(1);
        }
        else if (g2_Ddown || servoCapTop.getPosition() == 0) {
            servoCapTop.setPosition(.5);
        }

        telemetry.addData("driveScale: ", driveScale);

        telemetry.addData("launcherSpeed", launcherSpeed);
        telemetry.addData("beaconBlue: ", colorBeacon.blue());
        telemetry.addData("beaconRed: ", colorBeacon.red());
        telemetry.addData("servoLaunch: ", servoLaunch.getPosition());
        telemetry.addData("servoCapL: ", servoCapL.getPosition());
        telemetry.update();

    }

    public void servoLaunchUp() {
        if (servoLaunch.getPosition() > 0.08) {
            motorM.setPower(0);
            servoLaunch.setPosition(0.07);
            try {
                Thread.sleep(250);
            } catch (Exception E) {
            }
        }
    }

    public void servoLaunchDown() {
        if (servoLaunch.getPosition() < .75) {
            motorM.setPower(0);
            servoLaunch.setPosition(.79);
            try {
                Thread.sleep(250);
            } catch (Exception E) {
            }
        }
    }


    public void startShoot() {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        double shootPower = -.25 * voltage + 3.75;
        ElapsedTime time = new ElapsedTime();

        while (time.time() < warmUpMs){
            motorLaunchL.setPower(shootPower/4);
            motorLaunchR.setPower(-shootPower/4);
        }

        time.reset();

        while (time.time() < warmUpMs){
            motorLaunchL.setPower(shootPower/2);
            motorLaunchR.setPower(-shootPower/2);
        }

        time.reset();

        while (time.time() < warmUpMs){
            motorLaunchL.setPower(shootPower);
            motorLaunchR.setPower(-shootPower);
        }

        time.reset();

    }

    public void endShoot() {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        double shootPower = -.25 * voltage + 3.9;
        ElapsedTime time = new ElapsedTime();

        while (time.time() < warmUpMs){
            motorLaunchL.setPower(shootPower/2);
            motorLaunchR.setPower(-shootPower/2);
        }

        time.reset();

        while (time.time() < warmUpMs) {
            motorLaunchL.setPower(shootPower/4);
            motorLaunchR.setPower(-shootPower/4);
        }

        time.reset();

        while (time.time() < warmUpMs){
            motorLaunchL.setPower(0);
            motorLaunchR.setPower(0);
        }

        time.reset();

    }
}
