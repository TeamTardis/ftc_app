package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */
import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

@TeleOp(name="TestBed-2016", group="TeleOp")
@Disabled
public class TestBed_Andrew extends OpMode {

    private DcMotor m1;
    private DcMotor m2;
    private DcMotor m3;
    private DcMotor m4;
    //private DcMotor m5;
    //private DcMotor m6;
    private Servo S1;
    private Servo s2;
    private Servo s3;
    boolean LEDState = true;

////////////////////////////////////////////////////////////////////////////////////////////////////


    public TestBed_Andrew() {
    }


    @Override
    public void init() {
        m1 = hardwareMap.dcMotor.get("m1");
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");
        m4 = hardwareMap.dcMotor.get("m4");
        //      m5 = hardwareMap.dcMotor.get("m5");
        //      m6 = hardwareMap.dcMotor.get("m6");
        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        S1 = hardwareMap.servo.get("S1");
        s2 = hardwareMap.servo.get("s2");
        s3 = hardwareMap.servo.get("s3");

        m4.setMaxSpeed(943);//943
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    boolean linearExtention = false;

    @Override
    public void loop() {
        // Speed test for m4
        //1680 is 1 rotation, 111, 106, 52


        int position = m4.getCurrentPosition();
        telemetry.addData("Encoder Position", position);

        if (gamepad2.left_bumper) {
            m4.setPower(-1);
        } else if (!gamepad2.left_bumper) {
            m4.setPower(0);
        }

        if (gamepad2.a) {
            m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        /*

        float LUD = gamepad1.left_stick_y;
        float LRL = -gamepad1.left_stick_x;
        float R = gamepad1.right_stick_x;
        float LTrigger = gamepad1.left_trigger;
        double threshold = 0.1;

        ElapsedTime runtime = new ElapsedTime();
        boolean tss; //teleporterServoState

        //Servo for shooter
        if (gamepad1.right_bumper) {
            s2.setPosition(1);
            runtime.reset();
            while(runtime.seconds() < 2) {
            }
            s2.setPosition(0);
        }

        //Scooper Y is up, a is middle, b is down
        if (gamepad1.a) {
            s3.setPosition(0.08);
        }
        else if (gamepad1.b) {
            s3.setPosition(0.17);
        }
        else if (gamepad1.y) {
            s3.setPosition(0.35);
        }


        //Linear actuator
        if (gamepad1.x) {
            if(linearExtention) {
                S1.setPosition(0.25);
                runtime.reset();
                while(runtime.seconds() < .5) {
                }
                linearExtention = !linearExtention;
            }
            else if (!linearExtention) {
                S1.setPosition(.55);
                runtime.reset();
                while(runtime.seconds() < .5) {
                }
                linearExtention = !linearExtention;
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
        //If trigger is pressed, go very slow
        else if (gamepad1.right_trigger == 1) {
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
            //If nothing at all is happening, do nothing

        }

        //If nothing at all is happening, do nothing
    }
}
*/
    }
}