package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by Bo on 9/19/2016.
 */
@TeleOp(name = "DiagnosticTesting", group = "Testing")
public class DiagnosticTesting extends OpMode {

    ColorSensor color;


    public void init() {
        color = hardwareMap.colorSensor.get("colorF");
    }

    public void loop() {
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 1").getVoltage();
        telemetry.addData("Voltage: ", voltage);
        telemetry.addData("Light: ", color.alpha());
    }
}
