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

@Autonomous(name = "NewShootBeaconBlue", group = "Autonomous")
public class NewShootBeaconBlue extends AutoTemplate {


    double targetPower = 0.0;
    double shootPower = 0.0;
    //double rRatio = 1;//0.905;
    //double motorMultiplier = 1.0;

    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);
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
        PDturn(48, 2850);
        moveSquares(1.55, 1);
        stopMotors();
        sleep(125);
        moveToLine(.148, .148);
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(200);
        alignLineBlue(.396, -.406);
        sleep(100);
        move(.20, .20);
        time.reset();
        while (colorBeacon.blue() < 3 && colorBeacon.red() < 3 && opModeIsActive() && time.milliseconds() <= 1500) {
        }
        if (time.milliseconds() > 1500) {
            moveSquares(-.1, .5);
            while (colorF.alpha() < 3) {
                move(-.400, .400);
            }
        }
        move(0, 0);
        pressBeaconBlue();
        PDturn(0, 2600);
        moveSquares(1.35, 1);
        stopMotors();
        sleep(150);
        moveToLine(.142, .142);
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(200);
        alignLineBlue(.398, -.430); //hackedd
        stopMotors();

        sleep(100);
        move(.218, .218);
        while (colorBeacon.blue() < 3 && colorBeacon.red() < 3 && opModeIsActive()) {
        }
        stopMotors();
        pressBeaconBlue();

        //moveSquares(-1.5, 1);
        //moveTime(2000, 1, -1);
    }

    /*public void moveSquares(double squares, double pow) throws InterruptedException {
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

    /*public void move(double leftSpeed, double rightSpeed) throws InterruptedException{
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
    } */
/*
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


    public void pressBeaconBlue() throws InterruptedException {
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
    */
}
