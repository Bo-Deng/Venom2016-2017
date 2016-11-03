package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOpTrollBot", group = "TeleOp")
public class TrollBotTeleOp extends OpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    DcMotor motorShootR;
    DcMotor motorShootL;
    boolean isShooting = false;
    //Servo servoArmL;
   // Servo servoArmR;


    public void init() {

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorShootR = hardwareMap.dcMotor.get("motorShootR");
        motorShootL = hardwareMap.dcMotor.get("motorShootL");

        //servoArmL = hardwareMap.servo.get("servoArmL");
        //servoArmR = hardwareMap.servo.get("servoArmR");

        //servoArmL.setPosition(0.5);
        //servoArmR.setPosition(0.5);
        telemetry.addData("Init: ", "finished");
    }

    public void loop(){

        boolean g1_a = gamepad1.a;
        double g1_left_y = gamepad1.left_stick_y;
        double g1_right_y = gamepad1.right_stick_y;

        if (Math.abs(gamepad1.left_stick_y) > 0.1) {
           motorFL.setPower(-g1_left_y);
           motorBL.setPower(-g1_left_y);
        }
        else {
           motorFL.setPower(0);
           motorBL.setPower(0);
        }

        if (Math.abs(gamepad1.right_stick_y) > 0.1) {
            motorFR.setPower(-g1_right_y);
            motorBR.setPower(g1_right_y);
        }
        else {
            motorFR.setPower(0);
            motorBR.setPower(0);
        }

        if (g1_a) {
            if (isShooting){
                //endShoot();
                //isShooting = false;
                motorShootL.setPower(0);
                motorShootR.setPower(0);
            }

            else {
                //startShoot();
                //isShooting = true;
                motorShootL.setPower(.75);
                motorShootR.setPower(.75);
            }
        }
        else {
            motorShootL.setPower(0);
            motorShootR.setPower(0);
        }
       /*if (gamepad2.left_bumper) {
            servoArmL.setPosition(Range.clip(servoArmL.getPosition() + .02, 0, 1));
            servoArmR.setPosition(Range.clip(servoArmR.getPosition() - .02, 0, 1));

        }

        else if (gamepad2.right_bumper) {
            servoArmL.setPosition(Range.clip(servoArmL.getPosition() - .02, 0, 1));
            servoArmR.setPosition(Range.clip(servoArmR.getPosition() + .02, 0, 1));
        } */

        telemetry.addData("left stick: ", g1_left_y);
        telemetry.addData("right stick: ", g1_right_y);
    }

    public void startShoot() {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        double shootPower = -.25 * voltage + 3.75;
        ElapsedTime time = new ElapsedTime();

        time.time();

        while (time.time() < 78){
            motorShootL.setPower(shootPower/4);
            motorShootR.setPower(shootPower/4);
        }

        time.reset();

        while (time.time() < 78){
            motorShootL.setPower(shootPower/2);
            motorShootR.setPower(shootPower/2);
        }

        time.reset();

        while (time.time() < 78){
            motorShootL.setPower(shootPower);
            motorShootR.setPower(shootPower);
        }

        time.reset();

    }

    public void endShoot() {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        double shootPower = -.25 * voltage + 3.9;
        ElapsedTime time = new ElapsedTime();

        time.time();

        while (time.time() < 78){
            motorShootL.setPower(shootPower/2);
            motorShootR.setPower(shootPower/2);
        }

        time.reset();

        while (time.time() < 78){
            motorShootL.setPower(shootPower/4);
            motorShootR.setPower(shootPower/4);
        }

        time.reset();

        while (time.time() < 78){
            motorShootL.setPower(0);
            motorShootR.setPower(0);
        }

        time.reset();

    }

}
