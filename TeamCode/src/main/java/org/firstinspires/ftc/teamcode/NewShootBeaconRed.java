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

/**
 * Created by Navya on 11/28/2016.
 */

@Autonomous(name = "NewShootBeaconRed", group = "Autonomous")
public class NewShootBeaconRed extends AutoTemplate {

    double targetPower = 0.0;
    double shootPower = 0.0;

    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);
        waitForStart();
        servoCapTop.setPosition(.4);
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        targetPower = -0.144 * voltage + 2.6;

        moveSquares(1, .5);
        while (shootPower < targetPower) {
            shootPower = Range.clip(shootPower + .1, 0, targetPower);
            motorLaunchL.setPower(shootPower);
            motorLaunchR.setPower(-shootPower);
            sleep(50);
            //warm up launcher
        }
        motorM.setPower(1);
        //wait 2.5 sec to launch particles
        sleep(2500);
        motorM.setPower(0);
        motorLaunchL.setPower(0);
        motorLaunchR.setPower(0);

        moveSquares(-.7, .5);
        PDturnTest(-41.75, 1900);
        Pstraight(-41.75, 1, 1.28);
        Pstraight(-41.75, .5, .2);
        stopMotors();
        sleep(100);
        moveToLineFront(.195, .195);
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(100);
        alignLineRedBack(-.001, .405);
        sleep(100);
        //align the front once again
        alignLineRedFront(-.395, .395);
        move(.235, .235);
        time.reset();
        while (colorBeacon.blue() < 3 && colorBeacon.red() < 3
                && opModeIsActive() && time.milliseconds() <= 1500) {
        }
        /*//if robot fails to find beacon, scoot back and turn the other
        //way to look for it
        if (time.milliseconds() > 1500) {
            moveSquares(-.1, .5);
            while (colorF.alpha() < 3) {
                move(-.400, .400);
                I hate my life!!!
            }
        } */
        move(0, 0);
        pressBeaconRed();
        if (Math.abs(imu.getYaw()) < 1) {
            moveSquares(-2, .5);
            stop();
        }
        /*
        PDturnTest(0, 2500);
        Pstraight(0, 1, 1.05);
        stopMotors();
        sleep(150);
        moveToLineFront(.142, .142);
        */
        PDturnTest(-135, 1100);
        Pstraight(-135, -1, -.96);
        PDturnTest(-42, 1500);
        Pstraight(-42, 1, .41);
        moveToLineFront(.196, .196);
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(100);
        alignLineRedBack(-.01, .405);
        stopMotors();
        sleep(100);
        alignLineRedFront(-.405, .405);
        move(.240, .240);
        while (colorBeacon.blue() < 3 && colorBeacon.red() < 3 && opModeIsActive()) {
        }
        stopMotors();
        pressBeaconRed();
        //moveSquares(-1.5, 1);
        //moveTime(2000, 1, -1);
    }
}
