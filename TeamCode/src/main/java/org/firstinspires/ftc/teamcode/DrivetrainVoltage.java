package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Bo on 1/9/2017.
 */
@Autonomous(name = "DriveVolt", group = "Testing")
public class DrivetrainVoltage extends AutoTemplate {
    @Override
    public void runOpMode() throws InterruptedException {
        initStuff(hardwareMap);
        waitForStart();
        double voltage = hardwareMap.voltageSensor.get("Motor Controller 2").getVoltage();
        double powerScale = 1;
        time.reset();
        while (motorBL.getCurrentPosition() < 4000 && opModeIsActive()) {
            move(.25, .25);
        }
        DbgLog.error("seconds: " + time.seconds());
        DbgLog.error("encoders/sec: " + (motorBL.getCurrentPosition() / time.seconds()));
        DbgLog.error("voltage: " + voltage);
        motorBL.setPower(0);
    }
}
