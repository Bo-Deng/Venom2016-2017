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
        PDturn(-42.5, 3000);
        moveSquares(1.55, 1);
        stopMotors();
        sleep(125);
        moveToLineNew(.155, .155);
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(200);
        alignLineRed(-.38, .38);
        stopMotors();
        sleep(100);
        move(.2, .2);
        time.reset();
        while (colorBeacon.blue() < 3 && colorBeacon.red() < 3 && opModeIsActive() && time.milliseconds() < 2500) {
        }
        if (time.milliseconds() > 2500) {
            moveSquares(-.1, .5);
            while (colorF.alpha() < 3) {
                move(.400, -.400);
            }
        }
        move(0, 0);
        pressBeaconRed();
        PDturn(0, 2800);
        moveSquares(1.25, 1);
        stopMotors();
        sleep(125);
        moveToLineNew(.150, .150);
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(200);
        alignLineRed(-.382, .397);
        stopMotors();
        sleep(100);
        move(.200, .200);
        time.reset();
        while (colorBeacon.blue() < 3 && colorBeacon.red() < 3 && opModeIsActive() && time.milliseconds() < 2500) {
        }
        stopMotors();
        pressBeaconRed();
        //moveSquares(-1.5, 1);
        //moveTime(2000, 1, -1);
    }
}
