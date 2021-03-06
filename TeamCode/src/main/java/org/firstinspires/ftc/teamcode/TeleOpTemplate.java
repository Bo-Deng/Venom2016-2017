package org.firstinspires.ftc.teamcode;

/**
 * Created by Bo on 1/5/2017.
 */
public class TeleOpTemplate extends CustomLinearOpMode {

    boolean mDisabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        servoLaunchDown();
        servoLaunchUp();
    }

    //This method is called whenever our drivers are about to launch particles into the center vortex.
    //servoLaunch moves to an upwards position while the manipulator is moved backwards slightly,
    //preparing the particles before they are directed into the launcher
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

    //This method is called when our drivers are finished launching particles into the center vortex.
    //servoLaunch moves to an downwards position to block any particles from accidentally being
    //released into the launcher.
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
