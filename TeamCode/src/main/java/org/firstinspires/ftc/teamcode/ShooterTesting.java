package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ShooterTesting", group = "TeleOp")
public class ShooterTesting extends LinearOpMode {

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

    double shootPower = 0.0;
    ElapsedTime time = new ElapsedTime();

    // Maps the motors and sets them in the correct direction.
    public void initStuff() {

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

        servoButtonAuto.setPosition(.6);
        servoCapTop.setPosition(0);
        servoCapL.setPosition(1);
        servoLaunch.setPosition(.07);
    }

    public void runOpMode() throws InterruptedException {

        initStuff();
        waitForStart();

        while (opModeIsActive()) {
            double g1_leftY = gamepad1.left_stick_y;
            double g1_rightY = gamepad1.right_stick_y;

            boolean g1_leftBumper = gamepad1.left_bumper;
            boolean g1_rightBumper = gamepad1.right_bumper;
            boolean g1_x = gamepad1.x;
            boolean g1_y = gamepad1.y;
            boolean g1_up = gamepad1.dpad_up;
            boolean g1_down = gamepad1.dpad_down;

            double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();

            if (g1_leftY < -0.1) {
                motorM.setPower(1);
            } else if (g1_leftY > 0.1) {
                motorM.setPower(-1);
            } else {
                motorM.setPower(0);
            }

            if (g1_up) {
                shootPower = Range.clip(shootPower + .01, 0, 1);
                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                }
            }
            if (g1_down) {
                shootPower = Range.clip(shootPower - .01, 0, 1);
                try {
                    Thread.sleep(50);
                } catch (Exception e) {
                }
            }

            motorLaunchL.setPower(shootPower);
            motorLaunchR.setPower(-shootPower);

            if (g1_x) {
                motorLaunchL.setPower(0);
                motorLaunchR.setPower(0);
                try {
                    Thread.sleep(5000);
                } catch (Exception E) {
                }
                DbgLog.error("voltage: " + voltage);
                DbgLog.error("shootPower: " + shootPower);
            }

            telemetry.addData("voltage", voltage);
            telemetry.addData("shootPower", shootPower);
            telemetry.update();
        }

    /*public void startShoot(double voltage) {
        double shootPower = -.25 * voltage + 3.75;
        ElapsedTime time = new ElapsedTime();

        while (time.time() < 78){
            motorLaunchL.setPower(shootPower/4);
            motorLaunchR.setPower(-shootPower/4);
        }

        time.reset();

        while (time.time() < 78){
            motorLaunchL.setPower(shootPower/2);
            motorLaunchR.setPower(-shootPower/2);
        }

        time.reset();

        while (time.time() < 78){
            motorLaunchL.setPower(shootPower);
            motorLaunchR.setPower(-shootPower);
        }

        time.reset();

    }

    public void endShoot(double voltage) {
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
    */
    }
}
