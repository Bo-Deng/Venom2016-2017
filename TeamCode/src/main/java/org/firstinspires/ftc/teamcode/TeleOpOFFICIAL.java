package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    Servo servoButtonL;
    Servo servoButtonR;
    Servo 

    ColorSensor colorF;
    ColorSensor colorB;
    ColorSensor colorBeacon;
    IMU imu;

    int warmUpMs = 88;
    boolean isShooting = false;
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


    }

    public void loop() {

        double g1_leftY = -gamepad1.left_stick_y;  //up is changed to 1 and down to -1
        double g1_rightY = -gamepad1.right_stick_y;

        boolean g1_leftBumper = gamepad1.left_bumper;
        boolean g1_rightBumper = gamepad1.right_bumper;
        boolean g1_y = gamepad1.y;

        double g2_leftY = -gamepad2.left_stick_y;
        double g2_rightY = -gamepad2.right_stick_y;

        double g2_rightTrigger = gamepad2.right_trigger;

        if (Math.abs(g1_leftY) > 0.1) {

            motorBL.setPower(-g1_leftY);
            motorFL.setPower(g1_leftY);
        }
        else {
            motorBL.setPower(0);
            motorFL.setPower(0);
        }

        if (Math.abs(g1_rightY) > 0.1) {

            motorBR.setPower(-g1_rightY);
            motorFR.setPower(g1_rightY);
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

        if (Math.abs(g2_leftY) > 0.1) {
            motorM.setPower(-g2_leftY);
        }
        else {
            motorM.setPower(0);
        }

        if (g2_rightTrigger > 0.1) {
            motorLaunchL.setPower(1);
            motorLaunchR.setPower(-1);
        }

        else {
            /*time.reset();
            while (time.milliseconds() < 100) {
                motorLaunchL.setPower(.5);
                motorLaunchR.setPower(-.5);
            }
            time.reset();
            while (time.milliseconds() < 100) {
                motorLaunchL.setPower(.25);
                motorLaunchR.setPower(-.25);
            }
            time.reset();
            while (time.milliseconds() < 100) {
                motorLaunchL.setPower(.125);
                motorLaunchR.setPower(-.125);
            }
            time.reset();*/
            motorLaunchL.setPower(0);
            motorLaunchR.setPower(0);
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

        while (time.time() < warmUpMs){
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
