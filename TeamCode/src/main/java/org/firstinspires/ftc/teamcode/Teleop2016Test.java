package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
@Disabled
public class Teleop2016Test extends OpMode {

    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;
    private DcMotor m5;
    private DcMotor m6;
    private Servo S1;
    private Servo S2;

    @Override
    public void init() {
        m1 = hardwareMap.dcMotor.get("m1");
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");
        m4 = hardwareMap.dcMotor.get("m4");
        m5 = hardwareMap.dcMotor.get("m5");
        m6 = hardwareMap.dcMotor.get("m6");
        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        S1 = hardwareMap.servo.get("S1");
        S2 = hardwareMap.servo.get("S2");

        long setTime = System.currentTimeMillis();
        boolean S2 = false;
    }

    @Override
    public void loop() {
        float LUD = gamepad1.left_stick_y;
        float LRL = -gamepad1.left_stick_x;
        float R = gamepad1.right_stick_x;
        double threshold = 0.1;

        if (gamepad2.a) {
            S1.setPosition(.45);
        } else if (gamepad2.x) {
            S1.setPosition(.55);
        }

        //Shoot ball
        if (gamepad2.left_bumper) {
            S2.setPosition(.45);
            //Thread.sleep(2000);
            S2.setPosition(0);
        }

        //Move Mast, left trigger raise, left bumper lower
        if(gamepad1.left_trigger == 1) {
            m5.setPower(1);
        }
        else if(gamepad1.left_bumper) {
            m5.setPower(-1);
        }
        else{
            m5.setPower(0);
        }

        //Move Sweeper, X forward, A backwards
        if(gamepad1.x) {
            m6.setPower(1);
        }
        else if(gamepad1.a) {
            m6.setPower(-1);
        }
        else{
            m6.setPower(0);
        }

        //If trigger is pressed, go very slow
        if (gamepad1.right_trigger == 1) {
            //If R is not used,then use left Stick (R overrides L)
            if (R == 0) {
                //Controls for Left Stick
                m1.setPower((LRL + LUD) / 8);   //Motor1 is x-axis plus y-axis
                m2.setPower((LUD - LRL) / 8);   //Motor2 is x-axis minus y-axis
                m3.setPower((LUD - LRL) / 8);   //Motor3 is x-axis minus y axis(parallel to M2)
                m4.setPower((LRL + LUD) / 8);   //Motor4 is x-axis plus y-axis(parallel to M1)
            }
            //If R is Outside the threshold, then rotate on the direction of Right Stick
            if (R > threshold || R < -threshold) {
                //Controls for Right Stick
                m1.setPower(-R / 6);    //Set power to negative x-axis (orientation)
                m2.setPower(R / 6);     //Set power to positive x-axis
                m3.setPower(-R / 6);    //Set power to negative x-axis (orientation)
                m4.setPower(R / 6);     //Set power to positive x-axis
            }
        }

        //If no buttons are pressed, control normal
        //If R is not used,then use left Stick (R overrides L)
        if (gamepad1.right_trigger == 0) {
            if (R == 0) {
                //Controls for Left Stick
                m1.setPower((LRL + LUD) / 2);   //Motor1 is x-axis plus y-axis
                m2.setPower((LUD - LRL) / 2);   //Motor2 is x-axis minus y-axis
                m3.setPower((LUD - LRL) / 2);   //Motor3 is x-axis minus y axis(parallel to M2)
                m4.setPower((LRL + LUD) / 2);   //Motor4 is x-axis plus y-axis(parallel to M1)
            }
            //If R is Outside the threshold, then rotate on the direction of Right Stick
            if (R > threshold || R < -threshold) {
                //Controls for Right Stick
                m1.setPower(-R);    //Set power to negative x-axis (orientation)
                m2.setPower(R);     //Set power to positive x-axis
                m3.setPower(-R);    //Set power to negative x-axis (orientation)
                m4.setPower(R);     //Set power to positive x-axis
            }
        }

        //If nothing at all is happening, do nothing
        else {
            m1.setPower(0);    //Set power 0
            m2.setPower(0);     //Set power to 0
            m3.setPower(0);    //Set power to 0
            m4.setPower(0);     //Set power to 0
            m5.setPower(0);    //Set power to 0
            m6.setPower(0);     //Set power to 0
        }
    }
}
