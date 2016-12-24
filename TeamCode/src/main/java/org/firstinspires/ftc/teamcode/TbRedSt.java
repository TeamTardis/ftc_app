package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="RED_Steroids", group="Autonomous")

public class TbRedSt extends LinearOpMode { // tb = TestBed
    DcMotor m1; //Front left    Visual Below
    DcMotor m2; //Front right      1/  \2
    DcMotor m3; //Back left        3\  /4
    DcMotor m4; //Back right
    ColorSensor colorSensor;
    OpticalDistanceSensor odsSensor;
    GyroSensor gyro;

    public void HardwareMapping() {}

    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        ModernRoboticsI2cGyro gyro;   // Hardware Device Object

        int angleZ = 0;

        m1 = hardwareMap.dcMotor.get("m1");
        m2 = hardwareMap.dcMotor.get("m2");
        m3 = hardwareMap.dcMotor.get("m3");
        m4 = hardwareMap.dcMotor.get("m4");
        m1.setDirection(DcMotor.Direction.REVERSE);
        m3.setDirection(DcMotor.Direction.REVERSE);

        odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        colorSensor = hardwareMap.colorSensor.get("Color");
        colorSensor.enableLed(false); //Turns Color Sensor LED on
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        angleZ = gyro.getIntegratedZValue();

        HardwareMapping();

        //Start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        //Make sure the gyro is calibrated.
        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        //Show up on phone that gyro is calibrated
        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();


        waitForStart();

        //Setup color sensor
        float hsvValues[] = {0, 0, 0};
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        float red = colorSensor.red(); //Red setup
        float blue = colorSensor.blue(); //Blue setup

        runtime.reset(); //Find the white line
        while (odsSensor.getRawLightDetected() < .3) {
            if (gyro.getHeading() != 0 && gyro.getHeading() < 180) {
                m1.setPower(.5);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(1);
                idle();
            } else if (gyro.getHeading() > 180) {
                m1.setPower(1);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(.5);
                idle();
            } else {
                m1.setPower(1);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(1);
                idle();
            }
        }
        runtime.reset(); //Pan left
        while (runtime.seconds() < .3) {
            m1.setPower(-1);
            m2.setPower(1);
            m3.setPower(1);
            m4.setPower(-1);
            idle();
        }
        runtime.reset(); //Straighten against wall
        while (runtime.seconds() < 0.3) {
            m1.setPower(0.5);
            m2.setPower(0.5);
            m3.setPower(0.5);
            m4.setPower(0.5);
            idle();
        }
        runtime.reset(); //Backwards
        while (runtime.seconds() < .1) {
            m1.setPower(-1);
            m2.setPower(-1);
            m3.setPower(-1);
            m4.setPower(-1);
            idle();
        }
        runtime.reset(); //Go right until red is found
        while (colorSensor.red() < 4) {
            m1.setPower(.3);
            m2.setPower(-.3);
            m3.setPower(-.3);
            m4.setPower(.3);
            idle();
        }
        runtime.reset(); //Go forward slowly to hit button
        while (runtime.seconds() < .4) {
            m1.setPower(.2);
            m2.setPower(.2);
            m3.setPower(.2);
            m4.setPower(.2);
            idle();
        }
        runtime.reset(); //Back up slow
        while (runtime.seconds() < .2) {
            m1.setPower(-1);
            m2.setPower(-1);
            m3.setPower(-1);
            m4.setPower(-1);
            idle();
        }
        runtime.reset(); //Slight right to get sensor off beacon
        while (runtime.seconds() < .4) {
            m1.setPower(1);
            m2.setPower(-1);
            m3.setPower(-1);
            m4.setPower(1);
            idle();
        }
        runtime.reset(); //Find next white line
        while (odsSensor.getRawLightDetected() < .3) {
            if (gyro.getHeading() != 0 && gyro.getHeading() < 180) {
                m1.setPower(.2);
                m2.setPower(-.2);
                m3.setPower(-.8);
                m4.setPower(.8);
                idle();
            } else if (gyro.getHeading() > 180) {
                m1.setPower(.8);
                m2.setPower(-.8);
                m3.setPower(-.2);
                m4.setPower(.2);
                idle();
            } else {
                m1.setPower(.8);
                m2.setPower(-.8);
                m3.setPower(-.8);
                m4.setPower(.8);
                idle();
            }
        }
        runtime.reset(); //Pan left
        while (runtime.seconds() < .5) {
            m1.setPower(-1);
            m2.setPower(.1);
            m3.setPower(.1);
            m4.setPower(-1);
            idle();
        }
        runtime.reset(); //Straighten against wall
        while (runtime.seconds() < .6) {
            m1.setPower(0.5);
            m2.setPower(0.5);
            m3.setPower(0.5);
            m4.setPower(0.5);
            idle();
        }
        runtime.reset(); //Backwards
        while (runtime.seconds() < .15) {
            m1.setPower(-1);
            m2.setPower(-1);
            m3.setPower(-1);
            m4.setPower(-1);
            idle();
        }
        runtime.reset(); //Go right until red is found
        while (colorSensor.red() < 4) {
            m1.setPower(.2);
            m2.setPower(-.2);
            m3.setPower(-.2);
            m4.setPower(.2);
            idle();
        }
        runtime.reset(); //Go forward slowly to hit button
        while (runtime.seconds() < .4) {
            m1.setPower(.2);
            m2.setPower(.2);
            m3.setPower(.2);
            m4.setPower(.2);
            idle();
        }
        runtime.reset(); //Back up slow
        while (runtime.seconds() < .2) {
            m1.setPower(-1);
            m2.setPower(-1);
            m3.setPower(-1);
            m4.setPower(-1);
            idle();
        }
        runtime.reset(); //Go diagonal backwards until ball is knocked off and robot is parked
        while (runtime.seconds() < 2.5) {
            if (gyro.getHeading() != 0 && gyro.getHeading() < 180) {
                m1.setPower(-.7);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(-.1);
                idle();
            } else if (gyro.getHeading() > 180) {
                m1.setPower(-.1);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(-.7);
                idle();
            } else {
                m1.setPower(-.7);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(-.7);
                idle();
            }
        }
    }
}

