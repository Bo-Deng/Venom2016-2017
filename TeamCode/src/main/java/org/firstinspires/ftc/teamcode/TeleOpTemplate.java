package org.firstinspires.ftc.teamcode;

/**
 * Created by Bo on 1/5/2017.
 */
public class TeleOpTemplate extends CustomLinearOpMode {

    boolean mDisabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
    }

    public void servoLaunchUp() {
        if (servoLaunch.getPosition() < 0.75) {
            motorM.setPower(-1);
            mDisabled = true;
            servoLaunch.setPosition(.8);
            sleep(300);
            motorM.setPower(0);
            mDisabled = false;
        }
    }

    public void servoLaunchDown() {
        if (servoLaunch.getPosition() > .16) {
            motorM.setPower(0);
            mDisabled = true;
            servoLaunch.setPosition(.15);
            try {
                Thread.sleep(500);
            } catch (Exception E) {
            }
            mDisabled = false;
        }
    }
}
