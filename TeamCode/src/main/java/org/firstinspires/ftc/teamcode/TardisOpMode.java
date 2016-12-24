package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Corning Robotics on 12/21/16.
 */
public abstract class TardisOpMode extends OpMode{

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
}
