package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PIDtest", group = "test")
public class PIDtest extends LinearOpMode {

    DcMotor motorFL;
    DcMotor motorBL;
    DcMotor motorFR;
    DcMotor motorBR;
    IMU imu;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        imu = new IMU(hardwareMap.get(BNO055IMU.class, "IMU"));
        imu.IMUinit(hardwareMap);

        telemetry.addData("Init", " completed");
        DbgLog.error("init complete");
        waitForStart();

        Pstraight(.25, 4);
    }

    public void setMotors(double leftSpeed, double rightSpeed) {
        motorBL.setPower(leftSpeed);
        motorFL.setPower(leftSpeed);
        motorBR.setPower(rightSpeed);
        motorBR.setPower(rightSpeed);
    }

    public void Pstraight(double speed, int msTime) {

        double desiredAngle = imu.getYaw();
        double kP = 0.016;
        double PIDchange;
        double rightSpeed;
        double leftSpeed;
        double angleDiff;

        time.reset();
        while (time.milliseconds() < msTime) {
            angleDiff = imu.getYaw() - desiredAngle;
            leftSpeed = speed;
            rightSpeed = speed;

            if (angleDiff < 0) {
                PIDchange = -angleDiff * kP;
                rightSpeed -= PIDchange;
            } else if (angleDiff > 0) {
                PIDchange = angleDiff * kP;
                leftSpeed -= PIDchange;
            }

            setMotors(leftSpeed, rightSpeed);
        }
        setMotors(0, 0);
    }

    public void Pturn(double degTurn, int msTime) {

        imu.IMUinit(hardwareMap);
        double kP = 0.016;
        double PIDchange;
        double rightSpeed;
        double leftSpeed;
        double angleDiff;

        time.reset();
        while (time.milliseconds() < msTime) {
            angleDiff = degTurn - imu.getYaw();
            leftSpeed = 0;
            rightSpeed = 0;

            if (angleDiff < 0) {
                PIDchange = -angleDiff * kP;
                leftSpeed = -(PIDchange / 2);
                rightSpeed = PIDchange / 2;
            } else if (angleDiff > 0) {
                PIDchange = angleDiff * kP;
                leftSpeed = PIDchange / 2;
                rightSpeed = -(PIDchange / 2);

            }
            setMotors(leftSpeed, rightSpeed);
        }
        setMotors(0, 0);
    }
}