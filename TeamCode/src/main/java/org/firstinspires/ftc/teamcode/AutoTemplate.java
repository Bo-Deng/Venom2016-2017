package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Bo on 1/5/2017.
 */
public class AutoTemplate extends CustomLinearOpMode {

    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable
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

    public void moveToLineFront(double leftSpeed, double rightSpeed) throws InterruptedException {
        time.reset();
        double speedIncrease = 0;
        double prevEncoder = -motorBL.getCurrentPosition();
        double prevTime = time.seconds();
        while (colorF.alpha() < 3 && opModeIsActive()) { //move until line is detected
            move(leftSpeed + speedIncrease, rightSpeed + speedIncrease);
            if (-motorBL.getCurrentPosition() > prevEncoder || time.seconds() - prevTime > .25) {
                double encPerSec = (-motorBL.getCurrentPosition() - prevEncoder) / (time.seconds() - prevTime);
                DbgLog.error("encPerSec: " + encPerSec);
                if (Math.abs(encPerSec) < 375) {
                    speedIncrease += .005;
                    DbgLog.error("speed increased");
                }
                else if (Math.abs(encPerSec) > 700) {
                    speedIncrease -= .005;
                    DbgLog.error("speed decreased");
                }
                prevEncoder = -motorBL.getCurrentPosition();
                prevTime = time.seconds();
            }
        }
        stopMotors();
    }

    public void moveToLineNew(double leftSpeed, double rightSpeed) throws InterruptedException {
        time.reset();
        double speedIncrease = 0;
        double prevEncoder = -motorBL.getCurrentPosition();
        double prevTime = time.seconds();
        while (colorB.alpha() < 1 && opModeIsActive()) { //move until line is detected
            move(leftSpeed + speedIncrease, rightSpeed + speedIncrease);
            if (Math.abs(motorBL.getCurrentPosition()) > prevEncoder && time.seconds() - prevTime > .25) {
                double encPerSec = (Math.abs(motorBL.getCurrentPosition()) - prevEncoder) / (time.seconds() - prevTime);
                DbgLog.error("encPerSec: " + encPerSec);
                if (Math.abs(encPerSec) < 375) {
                    speedIncrease += .005;
                    DbgLog.error("speed increased");
                }
                else if (Math.abs(encPerSec) > 600) {
                    speedIncrease -= .005;
                    DbgLog.error("speed decreased");
                }
                prevEncoder = Math.abs(motorBL.getCurrentPosition());
                prevTime = time.seconds();
            }
        }
        stopMotors();
    }

    public void alignLineBlueNew(double leftSpeed, double rightSpeed) throws InterruptedException {
        //turns right
        double speedIncrease = 0;
        double prevEncoder = -motorBL.getCurrentPosition();
        double prevTime = time.seconds();
        while (colorF.alpha() < 3 && opModeIsActive()) {
            move(leftSpeed + speedIncrease, rightSpeed - speedIncrease);
            if (-motorBL.getCurrentPosition() > prevEncoder || time.seconds() - prevTime > .75) { //check every 100 cycles for change in encoder values
                double encPerSec = (-motorBL.getCurrentPosition() - prevEncoder) / (time.seconds() - prevTime);
                DbgLog.error("encPerSec: " + encPerSec);
                if (Math.abs(encPerSec) < 175) {
                    speedIncrease += .005;
                    DbgLog.error("speed increased");
                }
                else if (Math.abs(encPerSec) > 350) {
                    speedIncrease -= .005;
                    DbgLog.error("speed decreased");
                }
            }
        }
        stopMotors();
    }


    public void alignLineRedBack(double leftSpeed, double rightSpeed) throws InterruptedException {
        //turns right
        double speedIncrease = 0;
        double prevEncoder = -motorBL.getCurrentPosition();
        double prevTime = time.seconds();
        while (colorB.alpha() < 1 && opModeIsActive()) {
            move(leftSpeed, rightSpeed + speedIncrease);
            if (-motorBL.getCurrentPosition() < prevEncoder || time.seconds() - prevTime > .75) {
                double encPerSec = (-motorBL.getCurrentPosition() - prevEncoder) / (time.seconds() - prevTime);
                DbgLog.error("encPerSec: " + encPerSec);
                if (Math.abs(encPerSec) < 225) {
                    speedIncrease += .005;
                    DbgLog.error("speed increased");
                }
                else if (Math.abs(encPerSec) > 350) {
                    speedIncrease -= .005;
                    DbgLog.error("speed decreased");
                }
                prevEncoder = -motorBL.getCurrentPosition();
                prevTime = time.seconds();
            }
        }
        stopMotors();
    }



    public void alignLineBlueBack(double leftSpeed, double rightSpeed) throws InterruptedException {
        //turns right
        double speedIncrease = 0;
        double prevEncoder = -motorBL.getCurrentPosition();
        double prevTime = time.seconds();
        while (colorB.alpha() < 1 && opModeIsActive()) {
            move(leftSpeed + speedIncrease, rightSpeed);
            if (-motorBL.getCurrentPosition() > prevEncoder || time.seconds() - prevTime > .75) { //check every 100 cycles for change in encoder values
                double encPerSec = (-motorBL.getCurrentPosition() - prevEncoder) / (time.seconds() - prevTime);
                DbgLog.error("encPerSec: " + encPerSec);
                if (Math.abs(encPerSec) < 175) {
                    speedIncrease += .005;
                    DbgLog.error("speed increased");
                }
                else if (Math.abs(encPerSec) > 350) {
                    speedIncrease -= .005;
                    DbgLog.error("speed decreased");
                }
            }
        }
        stopMotors();
    }

    public void alignLineRedFront(double leftSpeed, double rightSpeed) throws InterruptedException {
        //turns right
        double speedIncrease = 0;
        double prevEncoder = -motorBL.getCurrentPosition();
        double prevTime = time.seconds();
        while (colorF.alpha() < 5 && opModeIsActive()) {
            move(leftSpeed - speedIncrease, rightSpeed + speedIncrease);
            if (-motorBL.getCurrentPosition() < prevEncoder || time.seconds() - prevTime > .25) {
                double encPerSec = (-motorBL.getCurrentPosition() - prevEncoder) / (time.seconds() - prevTime);
                DbgLog.error("encPerSec: " + encPerSec);
                if (Math.abs(encPerSec) < 60) {
                    speedIncrease += .005;
                    DbgLog.error("speed increased");
                }
                else if (Math.abs(encPerSec) > 100) {
                    speedIncrease -= .005;
                    DbgLog.error("speed decreased");
                }
                prevEncoder = -motorBL.getCurrentPosition();
                prevTime = time.seconds();
            }
        }
        stopMotors();
    }

    public void alignLineBlueFront(double leftSpeed, double rightSpeed) throws InterruptedException {
        //turns right
        double speedIncrease = 0;
        double prevEncoder = -motorBL.getCurrentPosition();
        double prevTime = time.seconds();
        while (colorF.alpha() < 3 && opModeIsActive()) {
            move(leftSpeed + speedIncrease, rightSpeed - speedIncrease);
            if (-motorBL.getCurrentPosition() > prevEncoder || time.seconds() - prevTime > .2) { //check every 100 cycles for change in encoder values
                double encPerSec = (-motorBL.getCurrentPosition() - prevEncoder) / (time.seconds() - prevTime);
                DbgLog.error("encPerSec: " + encPerSec);
                if (Math.abs(encPerSec) < 55) {
                    speedIncrease += .005;
                    DbgLog.error("speed increased");
                }
                else if (Math.abs(encPerSec) > 95) {
                    speedIncrease -= .005;
                    DbgLog.error("speed decreased");
                }
            }
        }
        stopMotors();
    }

    public void alignLineRedNew(double leftSpeed, double rightSpeed) throws InterruptedException {
        //turns right
        double speedIncrease = 0;
        double prevEncoder = -motorBL.getCurrentPosition();
        double prevTime = time.seconds();
        while (colorF.alpha() < 3 && opModeIsActive()) {
            move(leftSpeed - speedIncrease, rightSpeed + speedIncrease);
            if (-motorBL.getCurrentPosition() < prevEncoder || time.seconds() - prevTime > .75) {
                double encPerSec = (-motorBL.getCurrentPosition() - prevEncoder) / (time.seconds() - prevTime);
                DbgLog.error("encPerSec: " + encPerSec);
                if (Math.abs(encPerSec) < 225) {
                    speedIncrease += .005;
                    DbgLog.error("speed increased");
                }
                else if (Math.abs(encPerSec) > 350) {
                    speedIncrease -= .005;
                    DbgLog.error("speed decreased");
                }
                prevEncoder = -motorBL.getCurrentPosition();
                prevTime = time.seconds();
            }
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
            while (motorBL.getCurrentPosition() > -encoderVal && opModeIsActive()) {
                motorBL.setPower(-pow);
                motorBR.setPower(-pow);
                motorFL.setPower(pow);
                motorFR.setPower(pow);
            }
        }
        else if (squares < 0) {
            while (motorBL.getCurrentPosition() < -encoderVal && opModeIsActive()) {
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
        if (colorBeacon.blue() >= 2 && colorBeacon.red() <= 1) {
            telemetry.addData("Right:", " is blue");
            servoButtonAuto.setPosition(.48);
            DbgLog.error("BLUE BLUE BLUE BLUE BLUE BLUE");
            sleep(300);
        }

        else if (colorBeacon.red() >= 2 && colorBeacon.blue() <= 1) {
            telemetry.addData("Right:", " is red");
            servoButtonAuto.setPosition(.17);
            DbgLog.error("RED RED RED RED RED RED");
            sleep(300);
        }
        moveTime(1000, .5, .5);
        sleep(100);
        //moveTime(1200, -.6, -.6);
        moveSquares(-.2, .5);
        servoButtonAuto.setPosition(.31);
    }

    public void pressBeaconRed() throws InterruptedException {
        if (!opModeIsActive())
            return;
        DbgLog.error(colorBeacon.red() + "    " + colorBeacon.blue());
        if (colorBeacon.blue() >= 2) {
            telemetry.addData("Right:", " is blue");
            servoButtonAuto.setPosition(.15);
            DbgLog.error("BLUE BLUE BLUE BLUE BLUE BLUE");
            sleep(100);
            moveTime(900, .6, .6);
            //moveTime(1200, -.6, -.6);
        }

        else if (colorBeacon.red() >= 2) {
            telemetry.addData("Right:", " is red");
            servoButtonAuto.setPosition(.48);
            DbgLog.error("RED RED RED RED RED RED");
            sleep(100);
            moveTime(900, .6, .6);
            //moveTime(1200, -.6, -.6);
        }

        moveSquares(-.21, .5);
        servoButtonAuto.setPosition(.31);
    }

    public void PDturn(double degTurn, int msTime) throws InterruptedException {

        double kP = 0.055; //constants that we tuned to fit our robot
        double kd = 5;
        double prevError = 0;
        double currError = 0;
        double prevtime = time.milliseconds();
        double currTime;
        double PIDchange;
        double rightSpeed;
        double leftSpeed;
        double angleDiff;

        time.reset();
        //turns until time limit is reached
        while (time.milliseconds() < msTime && opModeIsActive()) {
            telemetry.update();
            angleDiff = degTurn - imu.getYaw();
            currError = angleDiff;
            currTime = time.milliseconds();
            leftSpeed = 0;
            rightSpeed = 0;

            //PID change is calculated by adding P and D terms
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

    public void PDturnTest(double degTurn, int msTime) throws InterruptedException {
        double startTime = time.milliseconds();
        //double kP = 0.05; //constants that we tuned to fit our robot
        double kP = .0325;
        //double kd = 1;
        double kd = 5;

        double kI = .0001;
        double prevError = 0;
        double currError = 0;
        double prevtime = time.milliseconds();
        double currTime = time.milliseconds();
        double PIDchange;
        double rightSpeed;
        double leftSpeed;
        double angleDiff = degTurn - imu.getYaw();
        double inteNoE;

        time.reset();
        //turns until time limit is reached
        //if (degTurn < imu.getYaw()) {
            while (time.milliseconds() < msTime && opModeIsActive()) {
                telemetry.update();
                angleDiff = degTurn - imu.getYaw();
                currError = angleDiff;
                currTime = time.milliseconds();
                leftSpeed = 0;
                rightSpeed = 0;
                inteNoE = currTime * kI;

                //PID change is calculated by adding P and D terms
                PIDchange = -angleDiff * kP - (currError - prevError) / (currTime - prevtime) * kd + inteNoE;
                leftSpeed = Range.clip(-(PIDchange / 2), -1, 1);
                rightSpeed = Range.clip(PIDchange / 2, -1, 1);

                prevError = currError;
                prevtime = currTime;
                move(leftSpeed, rightSpeed);
                DbgLog.error("" + PIDchange);
            }
        //}
        /*else {
            while (imu.getYaw() < degTurn - 2) {
                telemetry.update();
                angleDiff = degTurn - imu.getYaw();
                currError = angleDiff;
                currTime = time.milliseconds();
                leftSpeed = 0;
                rightSpeed = 0;
                inteNoE = currTime * kI;

                //PID change is calculated by adding P and D terms
                PIDchange = -angleDiff * kP - (currError - prevError) / (currTime - prevtime) * kd + inteNoE;
                leftSpeed = Range.clip(-(PIDchange / 2), -1, 1);
                rightSpeed = Range.clip(PIDchange / 2, -1, 1);

                prevError = currError;
                prevtime = currTime;
                move(leftSpeed, rightSpeed);
                DbgLog.error("" + imu.getYaw());
            }
        }*/
        move(0, 0);
        telemetry.addData("turn", " completed");
        telemetry.update();
        sleep(500);
        DbgLog.error("ANGLE: " + imu.getYaw());
    }

    public void Pstraight(double desiredAngle, double speed, double squares) throws InterruptedException {

        double kP = 0.042;
        double PIDchange;
        double rightSpeed;
        double leftSpeed;
        double angleDiff;

        resetEncoders();
        time.reset();
        if (squares > 0)
            while (Math.abs(motorBL.getCurrentPosition()) < squares * squaresToEncoder && opModeIsActive()) {
                angleDiff = imu.getYaw() - desiredAngle;
                DbgLog.error("" + angleDiff);
                leftSpeed = speed;
                rightSpeed = speed;

                if (angleDiff < 0) {
                    PIDchange = -angleDiff * kP;
                    rightSpeed -= PIDchange;
                    DbgLog.error("right -= " + PIDchange);
                } else if (angleDiff > 0) {
                    PIDchange = angleDiff * kP;
                    leftSpeed -= PIDchange;
                    DbgLog.error("left -= " + PIDchange);
                }

                move(leftSpeed, rightSpeed);
            }
        else if (squares < 0)
            while (-Math.abs(motorBL.getCurrentPosition()) > squares * squaresToEncoder && opModeIsActive()) {
                angleDiff = imu.getYaw() - desiredAngle;
                DbgLog.error("" + angleDiff);
                leftSpeed = speed;
                rightSpeed = speed;

                if (angleDiff < 0) {
                    PIDchange = -angleDiff * kP;
                    rightSpeed -= PIDchange;
                    DbgLog.error("right -= " + PIDchange);
                } else if (angleDiff > 0) {
                    PIDchange = angleDiff * kP;
                    leftSpeed -= PIDchange;
                    DbgLog.error("left -= " + PIDchange);
                }

                move(leftSpeed, rightSpeed);
            }
        stopMotors();
    }

    public void gyroTurn (  double speed, double angle)
            throws InterruptedException {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
            idle();
        }
    }

    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imu.getYaw();
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        motorBL.setPower(leftSpeed);
        motorFL.setPower(leftSpeed);
        motorBR.setPower(rightSpeed);
        motorFR.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
}
