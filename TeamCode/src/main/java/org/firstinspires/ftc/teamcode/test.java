package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by Corning Robotics on 12/21/16.
 */
public class test extends TardisOpMode {

    public enum steps {
        ROTATE_TO_VORTEX,
        SHOOT_FIRST_BALL,
        DRIVE_TO_WALL,
    }

    public steps CURRENT_STEP = steps.ROTATE_TO_VORTEX;

    @Override
    public void loop() {

        switch (CURRENT_STEP) {

            case ROTATE_TO_VORTEX:
                if (gyro.getHeading() > 45) {
                    CURRENT_STEP = steps.SHOOT_FIRST_BALL;
                    break;
                }
                m1.setPower(1);
                m2.setPower(1);
                m3.setPower(1);
                m4.setPower(1);

                break;
            case SHOOT_FIRST_BALL:
                break;
            case DRIVE_TO_WALL:
                break;

        }
    }
}
