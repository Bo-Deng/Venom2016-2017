package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Bo on 1/5/2017.
 */
public class AutoTemplate extends CustomLinearOpMode {

    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
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

    public void moveToLine(double leftSpeed, double rightSpeed) throws InterruptedException {
        double speedIncrease = 0;
        double prevEncoder = motorBL.getCurrentPosition();
        int i = 1;
        while (colorB.alpha() < 1 && opModeIsActive()) {
            DbgLog.error("" + prevEncoder);
            move(leftSpeed + speedIncrease, rightSpeed + speedIncrease);
            if (i % 100 == 0) {
                if (!(motorBL.getCurrentPosition() > prevEncoder)) {
                    speedIncrease += .001;
                    DbgLog.error("speed increased");
                    i = 0;
                }
                prevEncoder = motorBL.getCurrentPosition();
            }
            i++;
        }
        stopMotors();
    }

    public void alignLineBlue(double leftSpeed, double rightSpeed) throws InterruptedException {
        //turns right
        double speedIncrease = 0;
        double prevEncoder = motorBL.getCurrentPosition();
        int i = 1;
        while (colorF.alpha() < 3 && opModeIsActive()) {
            DbgLog.error("turn: " + prevEncoder);
            move(leftSpeed + speedIncrease, rightSpeed - speedIncrease);
            if (i % 100 == 0) {
                if (!(motorBL.getCurrentPosition() > prevEncoder + 1)) {
                    speedIncrease += .001;
                    DbgLog.error("turn speed increased");
                    i = 0;
                }
                prevEncoder = motorBL.getCurrentPosition();
            }
            i++;
        }
        stopMotors();
    }

    public void alignLineRed(double leftSpeed, double rightSpeed) throws InterruptedException {
        //turns left
        double speedIncrease = 0;
        double prevEncoder = motorBL.getCurrentPosition();
        int i = 1;
        while (colorF.alpha() < 3 && opModeIsActive()) {
            DbgLog.error("turn: " + prevEncoder);
            move(leftSpeed - speedIncrease, rightSpeed + speedIncrease);
            if (i % 100 == 0) {
                if (!(motorBL.getCurrentPosition() < prevEncoder - 1)) {
                    speedIncrease += .001;
                    DbgLog.error("turn speed increased");
                    i = 0;
                }
                prevEncoder = motorBL.getCurrentPosition();
            }
            i++;
        }
        stopMotors();
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

    public void pressBeaconBlue() throws InterruptedException {
        if (!opModeIsActive())
            return;
        DbgLog.error(colorBeacon.red() + "    " + colorBeacon.blue());
        if (colorBeacon.blue() >= 3) {
            telemetry.addData("Right:", " is blue");
            servoButtonAuto.setPosition(.48);
            DbgLog.error("BLUE BLUE BLUE BLUE BLUE BLUE");
            sleep(300);
        }

        else if (colorBeacon.red() >= 3) {
            telemetry.addData("Right:", " is red");
            servoButtonAuto.setPosition(.15);
            DbgLog.error("RED RED RED RED RED RED");
            sleep(300);
        }
        moveTime(1200, .6, .6);
        sleep(100);
        //moveTime(1200, -.6, -.6);
        moveSquares(-.25, .5);
        servoButtonAuto.setPosition(.31);
    }

    public void pressBeaconRed() throws InterruptedException {
        if (!opModeIsActive())
            return;
        DbgLog.error(colorBeacon.red() + "    " + colorBeacon.blue());
        if (colorBeacon.blue() >= 3) {
            telemetry.addData("Right:", " is blue");
            servoButtonAuto.setPosition(.15);
            DbgLog.error("BLUE BLUE BLUE BLUE BLUE BLUE");
            sleep(300);
            moveTime(1200, .6, .6);
            sleep(200);
            //moveTime(1200, -.6, -.6);
        }

        else if (colorBeacon.red() >= 3) {
            telemetry.addData("Right:", " is red");
            servoButtonAuto.setPosition(.48);
            DbgLog.error("RED RED RED RED RED RED");
            sleep(300);
            moveTime(1200, .6, .6);
            sleep(200);
            //moveTime(1200, -.6, -.6);
        }

        moveSquares(-.25, .5);
        servoButtonAuto.setPosition(.31);
    }

    public void PDturn(double degTurn, int msTime) throws InterruptedException {

        double kP = 0.055;
        double kd = 5;
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
}
