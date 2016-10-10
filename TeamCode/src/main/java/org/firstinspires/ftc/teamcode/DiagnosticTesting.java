package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    ElapsedTime time;

    public void init() {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        colorF = hardwareMap.colorSensor.get("colorF");
        colorB = hardwareMap.colorSensor.get("colorB");
        colorB.setI2cAddress(I2cAddr.create8bit(0x42));
        colorBeacon = hardwareMap.colorSensor.get("colorBeacon");
        colorBeacon.setI2cAddress(I2cAddr.create8bit(0x24));
        colorBeacon.enableLed(false);


        telemetry.addData("colorF is null", colorF == null);
        telemetry.addData("colorB is null", colorB == null);
        telemetry.addData("init: ", "finished");
        time = new ElapsedTime();
        time.reset();
    }

    public void loop() {

        telemetry.addData("Time", time.seconds());
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        telemetry.addData("Voltage: ", voltage);
        telemetry.addData("LightF: ", colorF.alpha());
        telemetry.addData("LightB: ", colorB.alpha());
        telemetry.addData("RedBeacon", colorBeacon.red());
        telemetry.addData("BlueBeacon", colorBeacon.blue());
        //telemetry.addData("BlueF: ", colorF.blue());
        //telemetry.addData("F: ", colorF.red());
        telemetry.addData("motorFL: ", motorFL.getCurrentPosition());
        telemetry.addData("motorFR: ", motorFR.getCurrentPosition());
        telemetry.addData("motorBL: ", motorBL.getCurrentPosition());
        //Navya was bad and had motorFR instead of motorBR
        telemetry.addData("motorBR: ", motorBR.getCurrentPosition());
        telemetry.update();
        //Good Coding
//        telemetry.deupdate();
//        telemetry.plusItem(Item item);
//        telemetry.plusData("motorFR: ", motorFL.getHereSpot());
//        telemetry.plusData("motorFL: ", motorFR.getThereSpot());
//        telemetry.plusData("motorBR: ", motorBL.getHereSpot());
//        telemetry.plusData("motorBL: ", motorBR.getThereSpot());
//        telemetry.uptodate();
//        teleemetry .addData("Vollie: ", voltyears);
//        telemetry.clearAll();
        //End of Good Coding by Nikhil
    }
}
