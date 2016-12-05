package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@TeleOp(name = "TeleOpLinear", group = "TeleOp")
public class TeleOpOFFICIAL extends LinearOpMode {

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

    double servoCapLPos;
    double servoCapRPos;
    double voltage = 13.0;
    double targetPower = 0.85;

    int warmUpMs = 88;
    double launcherSpeed = 0.0;
    double driveScale = 1.0;
    boolean mDisabled = false;
    boolean readVolt = true;
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
        servoCapTop.setPosition(0.04);
        servoCapL.setPosition(1);
        servoCapLPos = 1;
        servoCapRPos = 0;
        servoLaunch.setPosition(.15);

        telemetry.addData("init", " complete");
        telemetry.update();
    }

    public void runOpMode() throws InterruptedException{
        initStuff();
        waitForStart();
        while (opModeIsActive()){

            double g1_leftY = -gamepad1.left_stick_y;  //up is changed to 1 and down to -1
            double g1_rightY = -gamepad1.right_stick_y;
            double g1_rightTrigger = gamepad1.right_trigger;
            double g1_leftTrigger = gamepad1.left_trigger;

            boolean g1_leftBumper = gamepad1.left_bumper;
            boolean g1_rightBumper = gamepad1.right_bumper;
            boolean g1_y = gamepad1.y;
            boolean g1_Dleft = gamepad1.dpad_left;
            boolean g1_Dright = gamepad1.dpad_right;
            boolean g1_Dup = gamepad1.dpad_up;
            boolean g1_Ddown = gamepad1.dpad_down;

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

            if (readVolt)
                voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();

            targetPower = -0.144 * voltage + 2.65;
            readVolt = true;

            if (g1_y) {
                if (driveScale == 1) {
                    driveScale = 0.5;
                    try {
                        wait(350);
                    } catch (Exception E) {
                    }
                } else if (driveScale == 0.5) {
                    driveScale = 1;
                    try {
                        wait(350);
                    } catch (Exception E) {
                    }
                }
            }
            if (Math.abs(g1_leftY) > 0.1) {

                motorBL.setPower(-g1_leftY * driveScale);
                motorFL.setPower(g1_leftY * driveScale);
                readVolt = false;
            } else {
                motorBL.setPower(0);
                motorFL.setPower(0);
            }

            if (Math.abs(g1_rightY) > 0.1) {

                motorBR.setPower(-g1_rightY * driveScale);
                motorFR.setPower(g1_rightY * driveScale);
                readVolt = false;
            } else {
                motorBR.setPower(0);
                motorFR.setPower(0);
            }

            if (Math.abs(g2_leftY) > 0.1) {
                motorLift.setPower(-g2_leftY);
                readVolt = false;
            } else {
                motorLift.setPower(0);
            }

            if (g2_rightY < -0.1 && !mDisabled) {
                motorM.setPower(1);
                readVolt = false;
            } else if (g2_rightY > 0.1 && !mDisabled) {
                motorM.setPower(-1);
                readVolt = false;
            } else {
                motorM.setPower(0);
            }

            if (g2_rightTrigger > 0.1) {
                launcherSpeed = Range.clip(launcherSpeed + .1, -1, targetPower);
                if (launcherSpeed > .6)
                    servoLaunchUp();
                try {
                    Thread.sleep(50);
                } catch (Exception E) {
                }
            } else if (g2_y) {
                launcherSpeed = Range.clip(launcherSpeed - .1, -1, 1);
                try {
                    Thread.sleep(50);
                } catch (Exception E) {
                }
            } else {
                if (launcherSpeed > 0) {
                    launcherSpeed = Range.clip(launcherSpeed - .05, 0, 1);
                    servoLaunchDown();
                    try {
                        Thread.sleep(50);
                    } catch (Exception E) {
                    }
                } else if (launcherSpeed < 0) {
                    launcherSpeed = Range.clip(launcherSpeed + .05, -1, 0);
                    try {
                        Thread.sleep(50);
                    } catch (Exception E) {
                    }
                } else {
                    launcherSpeed = 0;
                }
            }
            if (launcherSpeed > 0)
                readVolt = false;
            motorLaunchL.setPower(launcherSpeed);
            motorLaunchR.setPower(-launcherSpeed);


            if (g1_rightTrigger > 0.1) {
                servoButtonL.setPower(1);
            } else if (g1_rightBumper) {
                servoButtonL.setPower(-1);
            } else {
                servoButtonL.setPower(0);
            }

            if (g1_leftTrigger > 0.1) {
                servoButtonR.setPower(-1);
            } else if (g1_leftBumper) {
                servoButtonR.setPower(1);
            } else {
                servoButtonR.setPower(0);
            }

            if (g1_Dleft) {
                servoButtonAuto.setPosition(Range.clip(servoButtonAuto.getPosition() - .05, 0, 1));
                sleep(50);
            } else if (g1_Dright) {
                servoButtonAuto.setPosition(Range.clip(servoButtonAuto.getPosition() + .05, 0, 1));
                sleep(50);
            }
            if (g2_rightBumper) {
                servoCapL.setPosition(0);
                servoCapLPos = 0;
                servoCapR.setPosition(1);
                servoCapRPos = 1;
            } else if (g2_leftBumper) {
                servoCapL.setPosition(1);
                servoCapLPos = 1;
                servoCapR.setPosition(0);
                servoCapRPos = 0;
            }

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
                servoCapL.setPosition(Range.clip(servoCapLPos + .05, 0, 1));
                servoCapLPos += .05;
                sleep(25);
            } else if (g2_Dright) {
                servoCapL.setPosition(Range.clip(servoCapLPos - .05, 0, 1));
                servoCapLPos -= .05;
                sleep(25);
            }

            if (g2_x) {
                servoCapR.setPosition(Range.clip(servoCapRPos + .05, 0, 1));
                servoCapRPos += .05;
                sleep(25);
            } else if (g2_b) {
                servoCapR.setPosition(Range.clip(servoCapRPos - .05, 0, 1));
                servoCapRPos -= .05;
                sleep(25);
            }

            if (g2_Dup) {
                servoCapTop.setPosition(Range.clip(servoCapTop.getPosition() + .05, 0, 1));
                if (servoCapTop.getPosition() >= .75) {
                    motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                }
                sleep(50);
            } else if (g2_Ddown) {
                servoCapTop.setPosition(Range.clip(servoCapTop.getPosition() - .05, 0, 1));
                motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                sleep(50);
            }

            /*if (g1_Dup) {
                servoLaunch.setPosition(Range.clip(servoLaunch.getPosition() + .05, 0, 1));
                sleep(25);
            }
            else if (g1_Ddown) {
                servoLaunch.setPosition(Range.clip(servoLaunch.getPosition() - .05, 0, 1));
                sleep(25);
            } */
            telemetry.addData("driveScale: ", driveScale);
            //telemetry.addData("auto; ", servoButtonAuto.getPosition());
            DbgLog.error("voltage: " + voltage);
            DbgLog.error("targetPower: " + targetPower);
            //telemetry.addData("servoLaunch: ", servoLaunch.getPosition());
            //telemetry.addData("top: ", servoCapTop.getPosition());
            //telemetry.addData("Beacon: ", colorBeacon.red() + "              " + colorBeacon.blue());
            telemetry.update();
        }

    }

    public void servoLaunchUp() {
        if (servoLaunch.getPosition() < 0.75) {
            motorM.setPower(-1);
            mDisabled = true;
            servoLaunch.setPosition(.8);
            sleep(500);
            motorM.setPower(0);
            mDisabled = false;
        }
    }

    public void servoLaunchDown() {
        if (servoLaunch.getPosition() > .16) {
            motorM.setPower(0);
            mDisabled = true;
            servoLaunch.setPosition(.15);
            try {
                Thread.sleep(500);
            } catch (Exception E) {
            }
            mDisabled = false;
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

    public void sleep(int ms) {
        try {
            Thread.sleep(ms);
        }
        catch (Exception E) {

        }
    }
}
