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
@Autonomous(name = "INSPECTIONAUTO", group = "Autonomous")
public class SuperBasicAuto extends AutoTemplate {

    double targetPower = 0.0;
    double shootPower = 0.0;


    public void runOpMode() throws InterruptedException {

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

        //motorLaunchL.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorFR.setDirection(DcMotorSimple.Direction.REVERSE);

        servoButtonAuto.setPosition(.6);
        servoCapTop.setPosition(0.04);
        servoCapL.setPosition(1);
        servoLaunch.setPosition(.15);
        servoB.setPosition(0.5);

        //voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        telemetry.addData("init", " complete");
        telemetry.update();

        waitForStart();
        time.reset();
        servoCapTop.setPosition(.4);
        servoLaunch.setPosition(.8);
        servoB.setPosition(0.5);
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        targetPower = -0.144 * voltage + 2.6;
        moveSquares(1.13, .5);
        sleep(500);
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
