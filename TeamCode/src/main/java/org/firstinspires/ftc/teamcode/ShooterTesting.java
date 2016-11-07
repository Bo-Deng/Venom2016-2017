package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ShooterTesting", group = "TeleOp")
public class ShooterTesting extends OpMode {

    DcMotor motorFR;
    DcMotor motorFL;
    DcMotor motorBR;
    DcMotor motorBL;
    DcMotor manipulator;
    DcMotor motorLaunchL;
    DcMotor motorLaunchR;

    int warmUpMs = 88;

    // Maps the motors and sets them in the correct direction.
    public void init() {

        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        manipulator = hardwareMap.dcMotor.get("motorM");
        motorLaunchL = hardwareMap.dcMotor.get("motorLaunchL");
        motorLaunchR = hardwareMap.dcMotor.get("motorLaunchR");

        //motorLaunchL.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop() {

        double g1_leftY = gamepad1.left_stick_y;
        double g1_rightY = gamepad1.right_stick_y;

        boolean g1_leftBumper = gamepad1.left_bumper;
        boolean g1_rightBumper = gamepad1.right_bumper;
        boolean g1_x = gamepad1.x;
        boolean g1_y = gamepad1.y;
        boolean g1_up = gamepad1.dpad_up;
        boolean g1_down = gamepad1.dpad_down;

        boolean isShooting = false;
        double shootPower = .75;
        
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();

        if (g1_up) {
            shootPower += .01;
        }
        if (g1_down) {
            shootPower -= .01;
        }

        motorLaunchL.setPower(shootPower);
        motorLaunchR.setPower(shootPower);

        if (g1_x) {
            DbgLog.error("voltage: " + voltage);
            DbgLog.error("shootPower: " + shootPower);
        }

        telemetry.addData("voltage", voltage);
        telemetry.addData("shootPower", shootPower);
    }

    public void startShoot(double voltage) {
        double shootPower = -.25 * voltage + 3.75;
        ElapsedTime time = new ElapsedTime();

        while (time.time() < 78){
            motorLaunchL.setPower(shootPower/4);
            motorLaunchR.setPower(-shootPower/4);
        }

        time.reset();

        while (time.time() < 78){
            motorLaunchL.setPower(shootPower/2);
            motorLaunchR.setPower(-shootPower/2);
        }

        time.reset();

        while (time.time() < 78){
            motorLaunchL.setPower(shootPower);
            motorLaunchR.setPower(-shootPower);
        }

        time.reset();

    }

    public void endShoot(double voltage) {
        double shootPower = -.25 * voltage + 3.9;
        ElapsedTime time = new ElapsedTime();

        while (time.time() < warmUpMs){
            motorLaunchL.setPower(shootPower/2);
            motorLaunchR.setPower(-shootPower/2);
        }

        time.reset();

        while (time.time() < warmUpMs){
            motorLaunchL.setPower(shootPower/4);
            motorLaunchR.setPower(-shootPower/4);
        }

        time.reset();

        while (time.time() < warmUpMs){
            motorLaunchL.setPower(0);
            motorLaunchR.setPower(0);
        }

        time.reset();

    }
}
