package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/**
 * Created by Bo on 9/12/2016.
 */
@TeleOp(name = "TeleOpOfficial", group = "TeleOp")
public class TeleOpOFFICIAL extends OpMode {

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor manipulator;
    DcMotor motorShootL;
    DcMotor motorShootR;

    int warmUpMs = 88;

    // Maps the motors and sets them in the correct direction.
    public void init() {

        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        manipulator = hardwareMap.dcMotor.get("manipulator");
        motorShootL = hardwareMap.dcMotor.get("motorShootL");
        motorShootR = hardwareMap.dcMotor.get("motorShootR");

        //motorShootL.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {

        double g1_leftY = gamepad1.left_stick_y;
        double g1_rightY = gamepad1.right_stick_y;

        boolean g1_leftBumper = gamepad1.left_bumper;
        boolean g1_rightBumper = gamepad1.right_bumper;
        boolean g1_a = gamepad1.a;
        boolean isShooting = false;

        if (Math.abs(g1_leftY) > 0.1 ) {

            motorBL.setPower(g1_leftY);
            motorFL.setPower(g1_leftY);
        }

        else {
            motorBL.setPower(0);
            motorFL.setPower(0);
        }

        if (Math.abs(g1_rightY) > 0.1) {

            motorBR.setPower(-g1_rightY);
            motorFR.setPower(-g1_rightY);
        }

        else {
            motorBR.setPower(0);
            motorFR.setPower(0);
        }

        if (g1_leftBumper) {
            // code for opening the manipulator
        }

        else if (g1_rightBumper) {
            // code for closing the manipulator
        }

        else {
            manipulator.setPower(0);
        }

        if (g1_a) {
            if (isShooting){
                endShoot();
                isShooting = false;
            }

            else {
                startShoot();
                isShooting = true;
            }
        }
        else {
            motorShootL.setPower(0);
            motorShootR.setPower(0);
        }

    }

    public void startShoot() {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        double shootPower = -.25 * voltage + 3.75;
        ElapsedTime time = new ElapsedTime();

        while (time.time() < 78){
            motorShootL.setPower(shootPower/4);
            motorShootR.setPower(-shootPower/4);
        }

        time.reset();

        while (time.time() < 78){
            motorShootL.setPower(shootPower/2);
            motorShootR.setPower(-shootPower/2);
        }

        time.reset();

        while (time.time() < 78){
            motorShootL.setPower(shootPower);
            motorShootR.setPower(-shootPower);
        }

        time.reset();

    }

    public void endShoot() {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        double shootPower = -.25 * voltage + 3.9;
        ElapsedTime time = new ElapsedTime();

        while (time.time() < warmUpMs){
            motorShootL.setPower(shootPower/2);
            motorShootR.setPower(-shootPower/2);
        }

        time.reset();

        while (time.time() < warmUpMs){
            motorShootL.setPower(shootPower/4);
            motorShootR.setPower(-shootPower/4);
        }

        time.reset();

        while (time.time() < warmUpMs){
            motorShootL.setPower(0);
            motorShootR.setPower(0);
        }

        time.reset();

    }
}
