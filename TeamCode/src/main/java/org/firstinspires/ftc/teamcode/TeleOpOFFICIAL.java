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

    double servoCapLPos;
    double servoCapRPos;
    double voltage = 13.8;
    double targetPower = 0.6;

    int warmUpMs = 88;
    double launcherSpeed = 0.0;
    int encPerSec = 1650;
    double driveScale = 1.0;
    double sweepDown = .24;
    double sweepUp = .48;
    boolean mDisabled = false;
    boolean readVolt = true;
    ElapsedTime time = new ElapsedTime();

    // Maps the motors and sets them in the correct direction.
    public void initStuff() throws InterruptedException {

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
        idle();

        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//1113
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//-1149
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//1055
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//1084
        //motorLaunchL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorLaunchR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motorM = hardwareMap.dcMotor.get("motorM");
        motorLaunchL = hardwareMap.dcMotor.get("motorLaunchL");
        motorLaunchR = hardwareMap.dcMotor.get("motorLaunchR");
        motorLaunchL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorLaunchR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        motorLaunchL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLaunchR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        servoB = hardwareMap.servo.get("servoB");
        servoSweep = hardwareMap.crservo.get("servoSweep");

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
        servoB.setPosition(0.5);

        //voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        telemetry.addData("init", " complete");
        telemetry.update();
    }

    public void move(double leftSpeed, double rightSpeed) throws InterruptedException {
        if (!opModeIsActive())
            return;
        motorBL.setPower(-leftSpeed);
        motorBR.setPower(-rightSpeed);
        motorFL.setPower(leftSpeed);
        motorFR.setPower(rightSpeed);

        //telemetry.addData("back: ", colorB.alpha());
        //telemetry.addData("front: ", colorF.alpha());
    }

    public void runOpMode() throws InterruptedException {
        initStuff();
        waitForStart();
        while (opModeIsActive()) {
            motorLaunchL.setMaxSpeed(encPerSec);
            motorLaunchR.setMaxSpeed(encPerSec);

            double g1_leftY = -gamepad1.left_stick_y;  //up is changed to 1 and down to -1
            double g1_rightY = -gamepad1.right_stick_y;
            double g1_rightTrigger = gamepad1.right_trigger;
            double g1_leftTrigger = gamepad1.left_trigger;

            boolean g1_leftBumper = gamepad1.left_bumper;
            boolean g1_rightBumper = gamepad1.right_bumper;
            boolean g1_y = gamepad1.y;
            boolean g1_b = gamepad1.b;
            boolean g1_x = gamepad1.x;
            boolean g1_Dleft = gamepad1.dpad_left;
            boolean g1_Dright = gamepad1.dpad_right;
            boolean g1_Dup = gamepad1.dpad_up;
            boolean g1_Ddown = gamepad1.dpad_down;

            boolean g1_start = gamepad1.start;
            boolean g1_back = gamepad1.back;

            double g2_leftY = -gamepad2.left_stick_y;
            double g2_rightY = -gamepad2.right_stick_y;
            double g2_rightTrigger = gamepad2.right_trigger;
            double g2_leftTrigger = gamepad2.left_trigger;

            boolean g2_leftBumper = gamepad2.left_bumper;
            boolean g2_rightBumper = gamepad2.right_bumper;
            boolean g2_Dleft = gamepad2.dpad_left;
            boolean g2_Dright = gamepad2.dpad_right;
            boolean g2_Dup = gamepad2.dpad_up;
            boolean g2_Ddown = gamepad2.dpad_down;
            boolean g2_x = gamepad2.x;
            boolean g2_b = gamepad2.b;
            boolean g2_a = gamepad2.a;
            boolean g2_y = gamepad2.y;
            boolean g2_back = gamepad2.back;
            boolean g2_start = gamepad2.start;

            //readVolt only reads voltage when motors are stopped,
            //because running motors gives inaccurate voltage
            /*if (readVolt) {
                double volt = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
                if (Math.abs(volt - voltage) < .6)
                    voltage = volt;
            }*/

            readVolt = true;

            if (imu.getPitch() > -155.0 && imu.getPitch() < -100.0) {
                while (imu.getPitch() > -155.0 && imu.getPitch() < -100.0)
                    move(1, 1);
            }
            else if (imu.getPitch() < 155.0 && imu.getPitch() > 100.0) {
                while (imu.getPitch() < 155.0 && imu.getPitch() > 100.0)
                    move(-1, -1);
            }

            if (g1_y) {
                if (driveScale == 1) {
                    driveScale = 0.5;
                    sleep(300);
                } else if (driveScale == 0.5) {
                    driveScale = 1;
                    sleep(300);
                }
            }

            if (g1_b) {
                driveScale *= -1;
                sleep(300);
            }

            if (g1_Dup) {
                encPerSec += 50;
                sleep(300);
            }
            if (g1_Ddown) {
                encPerSec -= 50;
                sleep(300);
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

            if (g2_leftY > 0.1) {
                motorLift.setPower(-g2_leftY);
                readVolt = false;
            }
            else if (g2_leftY < -0.1) {
                motorLift.setPower(-g2_leftY/5);
                readVolt = false;
            }
            else {
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

            if (g2_rightTrigger > 0.1) { //accelerates motors by .05 if nothing is pressed
                launcherSpeed = Range.clip(launcherSpeed + .1, -1, 1);
                if (launcherSpeed > .3)
                    servoLaunchUp();
                sleep(50);
            } else if (g2_back) { //launcher can go backwards in emergency situations
                launcherSpeed = Range.clip(launcherSpeed - .1, -1, 1);
                sleep(50);
            } else {
                if (launcherSpeed > 0) { //reduces speed by .05 if nothing is pressed
                    launcherSpeed = Range.clip(launcherSpeed - .05, 0, 1);
                    servoLaunchDown();
                    sleep(50);
                } else if (launcherSpeed < 0) { //if launcher was going backwards, slow to 0
                    launcherSpeed = Range.clip(launcherSpeed + .05, -1, 0);
                    sleep(50);
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
                servoButtonAuto.setPosition(/*Range.clip(servoButtonAuto.getPosition() - .05, 0, 1) */ 0.0);
                sleep(50);
            } else if (g1_Dright) {
                servoButtonAuto.setPosition(/*Range.clip(servoButtonAuto.getPosition() + .05, 0, 1)*/ .58);
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

            if (g2_leftTrigger > .1)
                sweepDown();
            else
                sweepUp();

            if (g2_a) {
                sweepDown = Range.clip(sweepDown - .02, 0, .74);
                sweepUp = Range.clip(sweepUp - .02, .26, 1);
                sleep(100);
            }
            else if (g2_y) {
                sweepDown = Range.clip(sweepDown + .02, 0, .74);
                sweepUp = Range.clip(sweepUp + .02, .26, 1);
                sleep(100);
            }
            else if (g1_x) {
                sweepDown = .24;
                sweepUp = .48;
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
            telemetry.addData("IMU Pitch: ", imu.getPitch());
            telemetry.addData("auto button", servoButtonAuto.getPosition());
            telemetry.addData("launcher encoder: ", encPerSec);
            telemetry.addData("BL BR FL FR", motorBL.getCurrentPosition() + " " + motorBR.getCurrentPosition() + " " + motorFL.getCurrentPosition() + " " + motorFR.getCurrentPosition());
            //telemetry.addData("colorB: ", colorB.alpha());
            //telemetry.addData("colorF: ", colorF.alpha());
            telemetry.addData("sweepDown: ", sweepDown);
            telemetry.addData("sweepUp: ", sweepUp);
            //telemetry.addData("auto; ", servoButtonAuto.getPosition());
            //DbgLog.error("voltage: " + voltage);
            //DbgLog.error("targetPower: " + targetPower);
            //telemetry.addData("servoLaunch: ", servoLaunch.getPosition());
            //telemetry.addData("top: ", servoCapTop.getPosition());
            telemetry.addData("Beacon: ", colorBeacon.red() + "              " + colorBeacon.blue());
            telemetry.addData("IMU: ", imu.getYaw());
            telemetry.update();
        }
    }

    public void servoLaunchUp() throws InterruptedException {
        if (servoLaunch.getPosition() < 0.75) {
            motorM.setPower(-1);
            mDisabled = true;
            servoLaunch.setPosition(.8);
            sleep(300);
            motorM.setPower(0);
            mDisabled = false;
        }
    }

    public void servoLaunchDown() throws InterruptedException {
        if (servoLaunch.getPosition() > .16) {
            motorM.setPower(0);
            mDisabled = true;
            servoLaunch.setPosition(.15);
            sleep(500);
            mDisabled = false;
        }
    }

    public void sweepDown() throws InterruptedException {
        if (Math.abs(servoB.getPosition() - sweepDown) > .002) {
            servoB.setPosition(sweepDown);
            sleep(200);
        }
        servoSweep.setPower(1);
    }

    public void sweepUp() throws InterruptedException {
        servoSweep.setPower(0);
        if (Math.abs(servoB.getPosition() - sweepUp) > .002) {
            servoB.setPosition(sweepUp);
            sleep(200);
        }
    }

    public void sleep(int ms) throws InterruptedException{
        Thread.sleep(ms);
    }
}
