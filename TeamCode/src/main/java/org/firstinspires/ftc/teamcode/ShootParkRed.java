package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Bo on 1/17/2017.
 */
@Autonomous(name = "ShootParkRed", group = "Autonomous")
public class ShootParkRed extends AutoTemplate {
    @Override
    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);
        waitForStart();
        servoCapTop.setPosition(.5);
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        double targetPower = -0.144 * voltage + 2.65;
        double shootPower = 0;

        sleep(5000);
        moveSquares(.25, .5);
        PDturn(-45, 5000);
        moveSquares(2.5, .5);
        while (shootPower < targetPower) {
            shootPower = Range.clip(shootPower + .1, 0, targetPower);
            motorLaunchL.setPower(shootPower);
            motorLaunchR.setPower(-shootPower);
            sleep(50);
            //warm up launcher
        }
        motorM.setPower(1);
        sleep(2500);
        motorM.setPower(0);
        motorLaunchL.setPower(0);
        motorLaunchR.setPower(0);
        sleep(2000);
        moveSquares(1, .5);
        moveTime(5000, -1, 1);
    }
}
