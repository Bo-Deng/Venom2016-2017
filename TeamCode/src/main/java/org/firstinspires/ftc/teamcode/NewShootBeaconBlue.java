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
        servoCapTop.setPosition(.7);
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
        //wait 2.5 sec to launch particles
        sleep(2500);
        motorM.setPower(0);
        motorLaunchL.setPower(0);
        motorLaunchR.setPower(0);

        moveSquares(-.725, .5);
        PDturnTest(48, 2400);
        Pstraight(48, 1, 1.42);
        stopMotors();
        sleep(125);
        moveToLineFront(.185, .185);
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(100);
        alignLineBlueBack(.350, .007);
        sleep(100);
        //align the front once again
        alignLineBlueFront(.360, -.360);
        move(.195, .195);
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
        pressBeaconBlue();
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
        PDturnTest(135, 1500);
        Pstraight(135, -1, -.85);
        PDturnTest(45, 2000);
        Pstraight(45, 1, .38);
        moveToLineFront(.170, .170);
        stopMotors();
        DbgLog.error("back sensed white line");
        sleep(100);
        alignLineBlueBack(.395, 0);
        stopMotors();
        sleep(100);
        alignLineBlueFront(.390, -.390);
        move(.218, .218);
        while (colorBeacon.blue() < 3 && colorBeacon.red() < 3 && opModeIsActive()) {
        }
        moveSquares(-.1, .7);
        stopMotors();
        pressBeaconBlue();

        //moveSquares(-1.5, 1);
        //moveTime(2000, 1, -1);
    }
}