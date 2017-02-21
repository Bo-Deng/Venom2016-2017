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
 * Created by Bo on 11/28/2016.
 */
@Autonomous(name = "ShootOnly", group = "Autonomous")
public class ShootOnly extends AutoTemplate {

    double targetPower = 0.0;
    double shootPower = 0.0;


    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);
        waitForStart();
        time.reset();
        servoCapTop.setPosition(.4);
        servoLaunch.setPosition(.8);
        servoB.setPosition(0.5);
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        targetPower = -0.144 * voltage + 2.6;
        sleep(5000);
        moveSquares(1.13, .5);
        while (shootPower < targetPower) {
            shootPower = Range.clip(shootPower + .1, 0, targetPower);
            motorLaunchL.setPower(shootPower);
            motorLaunchR.setPower(-shootPower);
            sleep(50);
            //warm up launcher
        }
        motorM.setPower(1);
        sleep(3500);
        motorM.setPower(0);
        motorLaunchL.setPower(0);
        motorLaunchR.setPower(0);
        moveSquares(-.7, .5);
        //moveSquares(1, .75);
        //stopMotors();
        /*turn(-38);
        moveTime(50, .25, .25);
        moveTime(50, .5, .5);
        moveTime(300, 1, 1);
        move(.375, .375);
        while (colorB.alpha() < 3 && opModeIsActive()) {
        }
        move(0, 0);
        DbgLog.error("back sensed white line");
        sleep(500);
        move(-.45, .45);
        while (colorF.alpha() < 3 && opModeIsActive()) {
        }
        move(0, 0);
        sleep(100);
        move(.275, .275);
        while (colorBeacon.blue() <= 3 && colorBeacon.red() <= 3) {
        }
        move(0, 0);
        pressBeacon(); */
    }

}
