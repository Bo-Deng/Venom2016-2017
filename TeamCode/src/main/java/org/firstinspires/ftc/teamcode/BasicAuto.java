package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Bo on 11/11/2016.
 */

@Autonomous(name = "BasicAuto", group = "Auto")
public class BasicAuto extends LinearOpMode {

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor motorLaunchR;
    DcMotor motorLaunchL;


    @Override
    public void runOpMode() throws InterruptedException{

        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBL");
        motorFL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorBL");
        motorLaunchR = hardwareMap.dcMotor.get("motorBL");
        motorLaunchL = hardwareMap.dcMotor.get("motorBL");



    }

    public void move(int leftSpeed, int rightSpeed){
        motorBL.setPower(0.5);

    }
}
