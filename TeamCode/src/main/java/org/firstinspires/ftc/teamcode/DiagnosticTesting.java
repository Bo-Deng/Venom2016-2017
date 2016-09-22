package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Bo on 9/19/2016.
 */
@TeleOp(name = "DiagnosticTesting", group = "Testing")
public class DiagnosticTesting extends OpMode {

    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;
    ColorSensor colorF;
    ColorSensor colorB;
    ColorSensor colorBeacon;


    public void init() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        //colorF = hardwareMap.colorSensor.get("colorF");
        //colorB = hardwareMap.colorSensor.get("colorB");
        //colorBeacon = hardwareMap.colorSensor.get("colorBeacon");

    }

    public void loop() {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        telemetry.addData("Voltage: ", voltage);
        telemetry.addData("LightF: ", colorF.alpha());
        telemetry.addData("LightB: ", colorB.alpha());
        telemetry.addData("motorFL: ", motorFL.getCurrentPosition());
        telemetry.addData("motorFR: ", motorFR.getCurrentPosition());
        telemetry.addData("motorBL: ", motorBL.getCurrentPosition());
        telemetry.addData("motorFR: ", motorBR.getCurrentPosition());
    }
}
