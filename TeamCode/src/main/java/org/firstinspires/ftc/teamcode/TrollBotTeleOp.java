package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Navya on 9/12/2016.
 */
@TeleOp(name = "TeleOpTrollBot", group = "TeleOp")
public class TrollBotTeleOp extends OpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;


    public void init() {

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Init: ", "finished");
    }

    public void loop(){

        double g1_left_y = gamepad1.left_stick_y;
        double g1_right_y = gamepad1.right_stick_y;

       if (Math.abs(gamepad1.left_stick_y) > 0.1) {
           motorFL.setPower(gamepad1.left_stick_y);
           motorBL.setPower(gamepad1.left_stick_y);
       }
        else {
           motorFL.setPower(0);
           motorBL.setPower(0);
       }

        if (Math.abs(gamepad1.right_stick_y) > 0.1) {
            motorFR.setPower(gamepad1.right_stick_y);
            motorBR.setPower(gamepad1.right_stick_y);
        }
        else {
            motorFR.setPower(0);
            motorBR.setPower(0);
        }

        telemetry.addData("left stick: ", g1_left_y);
        telemetry.addData("right stick: ", g1_right_y);
    }


}
