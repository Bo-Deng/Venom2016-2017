package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Navya on 9/12/2016.
 */
@TeleOp(name = "TeleOpTrollBot", group = "TeleOp")
public class TrollBotTeleOp extends OpMode {

    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;
    Servo servoArmL;
    Servo servoArmR;


    public void init() {

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        servoArmL = hardwareMap.servo.get("servoArmL");
        servoArmR = hardwareMap.servo.get("servoArmR");

        servoArmL.setPosition(0.5);
        servoArmR.setPosition(0.5);
        telemetry.addData("Init: ", "finished");
    }

    public void loop(){

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

        if (gamepad2.left_bumper) {
            servoArmL.setPosition(Range.clip(servoArmL.getPosition() + .02, 0, 1));
            servoArmR.setPosition(Range.clip(servoArmR.getPosition() - .02, 0, 1));

        }

        else if (gamepad2.right_bumper) {
            servoArmL.setPosition(Range.clip(servoArmL.getPosition() - .02, 0, 1));
            servoArmR.setPosition(Range.clip(servoArmR.getPosition() + .02, 0, 1));
        }

        telemetry.addData("left stick: ", g1_left_y);
        telemetry.addData("right stick: ", g1_right_y);
    }


}
