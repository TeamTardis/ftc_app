//  Controls

//      Rotate Sweeper; X toggle, A manual backwards      (Game pad 1)
//      Driver controls; left stick move, right stick rotate
//      Slow mode; right trigger

//      Linear Actuator; X toggle                         (Game pad 2)
//      Shoot ball; right trigger
//      Shooter Motors; right bumper toggle
//      Move Mast; left trigger raise, left bumper lower

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp-2016", group = "TeleOp")
public class Teleop2016 extends OpMode {
    DcMotor m1;
    DcMotor m2;
    DcMotor m3;
    DcMotor m4;
    DcMotor m5;
    DcMotor m6;
    DcMotor m7;
    DcMotor m8;
    Servo s1;
    Servo s2;
    Servo s3;
    TouchSensor touchSensor1;
    TouchSensor touchSensor2;
    ColorSensor colorSensor1;
    ColorSensor colorSensor2;
    OpticalDistanceSensor odsSensor1;
    OpticalDistanceSensor odsSensor2;
    GyroSensor gyro;

    @Override
    public void init() {
        m1 = hardwareMap.dcMotor.get("m1");
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");
        m4 = hardwareMap.dcMotor.get("m4");
        m5 = hardwareMap.dcMotor.get("m5");
        m6 = hardwareMap.dcMotor.get("m6");
        m7 = hardwareMap.dcMotor.get("m7");
        m8 = hardwareMap.dcMotor.get("m8");
        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        m5.setMaxSpeed(1150);  //Set max speed for shooter
        m6.setMaxSpeed(1150);  //943 (-9)
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        s3 = hardwareMap.servo.get("s3");

        odsSensor1 = hardwareMap.opticalDistanceSensor.get("ods1");
        I2cAddr ods1 = I2cAddr.create8bit(0x61);

        odsSensor2 = hardwareMap.opticalDistanceSensor.get("ods2");
        I2cAddr ods2 = I2cAddr.create8bit(0x62);

        colorSensor1 = hardwareMap.colorSensor.get("c1");
        colorSensor1.enableLed(false); //Turns Color Sensor LED off

        /*
        colorSensor2 = hardwareMap.colorSensor.get("c2");
        colorSensor2.enableLed(true);
        */

        touchSensor1 = hardwareMap.touchSensor.get("t1");
        I2cAddr t1 = I2cAddr.create8bit(0x60);

        touchSensor2 = hardwareMap.touchSensor.get("t2");
        I2cAddr t2 = I2cAddr.create8bit(0x69);

        gyro = hardwareMap.gyroSensor.get("gyro");

        s1.setPosition(0.15);
        s3.setPosition(0.65);

    }

    //Toggle Boolean states
    boolean sweeper = false;
    boolean shooter = false;
    boolean speedB = false;
    boolean linearExtension = true;
    int speed = 1150;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void loop() {
        float LUD = gamepad1.left_stick_y;
        float LRL = -gamepad1.left_stick_x;
        float R = gamepad1.right_stick_x;
        float LUD2 = gamepad2.left_stick_y;
        float RUD = gamepad2.right_stick_y;
        double threshold = 0.1;
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMaxSpeed(speed);  //Set max speed for shooter
        m6.setMaxSpeed(speed);  //943 (-9) 1150


//        //Servo controls
//        //Linear actuator for holding the cap ball after shutdown
//        if (gamepad2.x) {
//            linearExtension = !linearExtension; //Toggle
//            if(!linearExtension) {
//                s1.setPosition(0.15); //In
//            }
//            if(linearExtension) {
//                s1.setPosition(.35); //Out
//            }
//        }

        if (RUD != 0) {
            s1.setPosition((RUD / 2) + 0.5); //Servo controls from joystick
        }


        //Shooter toggle, fast or slow (Game pad 1 left bumper)
        if (gamepad1.left_bumper) {
            shooter = !shooter; //Toggle
            if (shooter) { //                            Tachometer Readings
                m5.setPower(-1); //Full speed (2346 2375 2595 2580 2580 2570 2480 40)
                m6.setPower(1); //            (2095 2616 2330 2300 2322 2450 2480 75)
            } else if (!shooter) {
                m5.setPower(-.1); //Slow
                m6.setPower(.1);
            }
        }

        //Shooter toggle, adjust shooter speed (Game pad 1 A & B)
        if (gamepad1.a) {
            speed = speed + 300; //Faster
        }

        if (gamepad1.b) {
            speed = speed - 300; //Slower
        }


        //Shoot ball (Game pad 1 left trigger)
        if (gamepad1.left_trigger == 1) {
            s2.setPosition(1); //Shoot
            runtime.reset();
        }

        if (runtime.seconds() < 1.5) { //Wait
            s2.setPosition(1);
        } else {
            s2.setPosition(.05); //Reset
        }


        //Mast code with limit touch sensor switches
        if ((LUD2 > threshold) && (!touchSensor2.isPressed())) { //Go up
            m8.setPower(1);
        } else if ((LUD2 < -threshold) && (!touchSensor1.isPressed())) { //Go down
            m8.setPower(-1);
        } else {
            m8.setPower(0);
        }


        //Scooper Y is up, b is middle, a is down
        if (gamepad2.a) {
            s3.setPosition(0.06); //Down
        } else if (gamepad2.b) {
            s3.setPosition(0.18); //Mid
        } else if (gamepad2.y) {
            s3.setPosition(0.35); //Up
        }


        //Rotate Sweeper, right trigger toggle, right bumper manual backwards (Game pad 2)
        if (gamepad2.right_bumper) {
            sweeper = !sweeper; //Toggle
            if (sweeper) {
                m7.setPower(1); //On
            } else if (!sweeper) {
                m7.setPower(0); //Off
            }
        }
        if (gamepad2.right_trigger == 1) {
            sweeper = !sweeper; //Toggle
            if (sweeper) {
                m7.setPower(-1); //Backwards
            } else if (!sweeper) {
                m7.setPower(0); //Off
            }
        }


        //Driver controls (Game pad 1)
        //If right trigger is not pressed, go normal
        if (gamepad1.right_trigger == 0) {
            //If R is not used,then use left Stick (R overrides L)
            if (R == 0) {
                //Controls for Left Stick
                m1.setPower((LRL + LUD));   //Motor1 is x-axis plus y-axis
                m2.setPower((LUD - LRL));   //Motor2 is x-axis minus y-axis
                m3.setPower((LUD - LRL));   //Motor3 is x-axis minus y axis(parallel to m2)
                m4.setPower((LRL + LUD));   //Motor4 is x-axis plus y-axis(parallel to m1)
            }
            //If R is Outside the threshold, then rotate on the direction of Right Stick
            else if (R > threshold || R < -threshold) {
                //Controls for Right Stick
                m1.setPower(R);    //Set power to negative x-axis (orientation)
                m2.setPower(-R);     //Set power to positive x-axis
                m3.setPower(R);    //Set power to negative x-axis (orientation)
                m4.setPower(-R);     //Set power to positive x-axis
            }
        }

        //If right trigger is pressed, control slow
        //If right stick is not used,then use left Stick (R overrides L)
        if (gamepad1.right_trigger == 1) {
            if (R == 0) {
                //Controls for Left Stick
                m1.setPower((LRL + LUD) / 4);   //Motor1 is x-axis plus y-axis
                m2.setPower((LUD - LRL) / 4);   //Motor2 is x-axis minus y-axis
                m3.setPower((LUD - LRL) / 4);   //Motor3 is x-axis minus y axis(parallel to M2)
                m4.setPower((LRL + LUD) / 4);   //Motor4 is x-axis plus y-axis(parallel to M1)
            }
            //If R is Outside the threshold, then rotate on the direction of Right Stick
            else if (R > threshold || R < -threshold) {
                //Controls for Right Stick
                m1.setPower(R / 6);    //Set power to negative x-axis (orientation)
                m2.setPower(-R / 6);     //Set power to positive x-axis
                m3.setPower(R / 6);    //Set power to negative x-axis (orientation)
                m4.setPower(-R / 6);     //Set power to positive x-axis
            }
        }
    }
}
