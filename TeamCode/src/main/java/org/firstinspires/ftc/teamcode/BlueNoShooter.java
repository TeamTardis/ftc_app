/*  Red antonymous:    Start with back wheels slightly behind center of playing field against red
    (TestBed)          starting wall. Robot then goes diagonal until it senses white line on first
                       beacon. It pans left and forward until it is straight on wall. Then it backs
                       up and goes right until the color sensor senses red. As soon as it senses the
                       right color, it goes forward, at first slow and then fast to make sure it is
                       pressed. After the first beacon is hit, the robot backs up and goes right to
                       find the next white line. The wiggle that is seen is the robot correcting
                       itself based upon the input received by the gyro sensor. When it reaches the
                       white line, the original way of hitting the beacon is repeated. Once this is
                       completed, the robot goes diagonal backwards until the ball is hit off and
                       the robot is parked partially on the center plate.
*/

package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="BLUENoShooter", group="AutoFast")
public class BlueNoShooter extends LinearOpMode { // tb = TestBed
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
    ColorSensor colorSensor;
    OpticalDistanceSensor odsSensor1;
    GyroSensor gyro;

    public void HardwareMapping() {}

    private ElapsedTime     runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        ModernRoboticsI2cGyro gyro;   // Hardware Device Object

        int angleZ = 0;

        m1 = hardwareMap.dcMotor.get("m3");
        m2 = hardwareMap.dcMotor.get("m1");
        m3 = hardwareMap.dcMotor.get("m4");
        m4 = hardwareMap.dcMotor.get("m2");
        m5 = hardwareMap.dcMotor.get("m5");
        m6 = hardwareMap.dcMotor.get("m6");
        m7 = hardwareMap.dcMotor.get("m7");
        m8 = hardwareMap.dcMotor.get("m8");
        m2.setDirection(DcMotor.Direction.REVERSE);
        m4.setDirection(DcMotor.Direction.REVERSE);

        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        s3 = hardwareMap.servo.get("s3");

        odsSensor1 = hardwareMap.opticalDistanceSensor.get("ods1");
        I2cAddr ods1 = I2cAddr.create8bit(0x61);

        colorSensor = hardwareMap.colorSensor.get("c1");
        colorSensor.enableLed(false); //Turns Color Sensor LED off

        /*
        colorSensor2 = hardwareMap.colorSensor.get("c2");
        colorSensor2.enableLed(true);
        */

        touchSensor1 = hardwareMap.touchSensor.get("t1");
        I2cAddr t1 = I2cAddr.create8bit(0x60);

        touchSensor2 = hardwareMap.touchSensor.get("t2");
        I2cAddr t2 = I2cAddr.create8bit(0x69);

        m5.setMaxSpeed(820);  //Set max speed for shooter
        m6.setMaxSpeed(810);  //943 (-9)
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        s1.setPosition(0.4);
        s3.setPosition(0.35); //Up

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
        while (odsSensor1.getRawLightDetected() < .3) {
            if (gyro.getHeading() != 0 && gyro.getHeading() < 180) {
                m1.setPower(0); //0.2
                m2.setPower(0.8);
                m3.setPower(1);
                m4.setPower(0); //-0.2
                idle();
            } else if (gyro.getHeading() > 180) {
                m1.setPower(0); //0.2
                m2.setPower(0.8);
                m3.setPower(1);
                m4.setPower(0); //-0.2
                idle();
            } else {
                m1.setPower(0);
                m2.setPower(1);
                m3.setPower(1);
                m4.setPower(0);
                idle();
            }
        }
//            if (gyro.getHeading() != 0 && gyro.getHeading() < 180) {
//                m1.setPower(0);
//                m2.setPower(.8);
//                m3.setPower(1);
//                m4.setPower(0);
//                idle();
//            } else if (gyro.getHeading() > 180) {
//                m1.setPower(0);
//                m2.setPower(1);
//                m3.setPower(0.8);
//                m4.setPower(0);
//                idle();
//            } else {
//                m1.setPower(0);
//                m2.setPower(1);
//                m3.setPower(1);
//                m4.setPower(0);
//                idle();
//            }
//        }
        runtime.reset(); //Stop
        while (runtime.seconds() < .2) {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
            idle();
        }
        runtime.reset(); //Forward
        while (runtime.seconds() < .1) {
            m1.setPower(.3);
            m2.setPower(.3);
            m3.setPower(.3);
            m4.setPower(.3);
            idle();
        }
        runtime.reset(); //Stop
        while (runtime.seconds() < .2) {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
            idle();
        }
        runtime.reset(); //Pan right
        while (runtime.seconds() < .4) {
            m1.setPower(.6);
            m2.setPower(-.6);
            m3.setPower(-.6);
            m4.setPower(.6);
            idle();
        }
        runtime.reset(); //Go left until blue is found
        while (colorSensor.blue() < 3) {
            if (gyro.getHeading() != 0 && gyro.getHeading() < 180) {
                m1.setPower(-.2);
                m2.setPower(.2);
                m3.setPower(.3);
                m4.setPower(-.3);
                idle();
            } else if (gyro.getHeading() > 180) {
                m1.setPower(-.3);
                m2.setPower(.3);
                m3.setPower(.2);
                m4.setPower(-.2);
                idle();
            } else {
                m1.setPower(-.3);
                m2.setPower(.3);
                m3.setPower(.3);
                m4.setPower(-.3);
                idle();
            }
        }
        runtime.reset(); //Left
        while (runtime.seconds() < .2) {
            m1.setPower(.3);
            m2.setPower(-.3);
            m3.setPower(-.3);
            m4.setPower(.3);
            idle();
        }
        runtime.reset(); //Stop
        while (runtime.seconds() < .2) {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
            idle();
        }
        runtime.reset(); //Backward
        while (runtime.seconds() < .2) {
            m1.setPower(-.2);
            m2.setPower(-.2);
            m3.setPower(-.2);
            m4.setPower(-.2);
            idle();
        }
        runtime.reset(); //Stop
        while (runtime.seconds() < .2) {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
            idle();
        }
        runtime.reset(); //Go forward slowly to hit button
        while (runtime.seconds() < 2) {
            m1.setPower(.3);
            m2.setPower(.3);
            m3.setPower(.3);
            m4.setPower(.3);
            idle();
        }
        runtime.reset(); //Back up slow
        while (runtime.seconds() < 0.7) {
            m1.setPower(-.3);
            m2.setPower(-.3);
            m3.setPower(-.3);
            m4.setPower(-.3);
            idle();
        }
        runtime.reset(); //Slight right to get ODS sensor off white line
        while (runtime.seconds() < .7) {
            m1.setPower(-.6);
            m2.setPower(.6);
            m3.setPower(.6);
            m4.setPower(-.6);
            idle();
        }
        runtime.reset(); //Find next white line
        while (odsSensor1.getRawLightDetected() < .3) {
            if (gyro.getIntegratedZValue() != 0 && gyro.getIntegratedZValue() > 0){//170 {
                m1.setPower(-.3);
                m2.setPower(.3);
                m3.setPower(.6); // .6
                m4.setPower(-.6); // .6
                idle();
            } else if (gyro.getIntegratedZValue() < 0){ //160{
                m1.setPower(-.6); // .6
                m2.setPower(.6); /// .6
                m3.setPower(.3);
                m4.setPower(-.3);
                idle();
            } else {
                m1.setPower(-.6);
                m2.setPower(.6);
                m3.setPower(.6);
                m4.setPower(-.6);
                idle();
            }
        }
        runtime.reset(); //Stop
        while (runtime.seconds() < .2) {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
            idle();
        }
        runtime.reset(); //Pan left
        while (runtime.seconds() < .7) {
            m1.setPower(.5);
            m2.setPower(-.5);
            m3.setPower(-.5);
            m4.setPower(.5);
            idle();
        }
        runtime.reset(); //Go right until blue is found
        while (colorSensor.blue() < 3) {
            if (gyro.getHeading() != 0 && gyro.getHeading() < 180) {
                m1.setPower(-.2);
                m2.setPower(.2);
                m3.setPower(.3);
                m4.setPower(-.3);
                idle();
            } else if (gyro.getHeading() > 180) {
                m1.setPower(-.3);
                m2.setPower(.3);
                m3.setPower(.2);
                m4.setPower(-.2);
                idle();
            } else {
                m1.setPower(-.3);
                m2.setPower(.3);
                m3.setPower(.3);
                m4.setPower(-.3);
                idle();
            }
        }
        runtime.reset(); //Left
        while (runtime.seconds() < .2) {
            m1.setPower(.3);
            m2.setPower(-.3);
            m3.setPower(-.3);
            m4.setPower(.3);
            idle();
        }
        runtime.reset(); //Stop
        while (runtime.seconds() < .2) {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
            idle();
        }
        runtime.reset(); //Backward
        while (runtime.seconds() < .2) {
            m1.setPower(-.2);
            m2.setPower(-.2);
            m3.setPower(-.2);
            m4.setPower(-.2);
            idle();
        }
        runtime.reset(); //Stop
        while (runtime.seconds() < .2) {
            m1.setPower(0);
            m2.setPower(0);
            m3.setPower(0);
            m4.setPower(0);
            idle();
        }
        runtime.reset(); //Go forward slowly to hit button
        while (runtime.seconds() < 2) {
            m1.setPower(.3);
            m2.setPower(.3);
            m3.setPower(.3);
            m4.setPower(.3);
            idle();
        }
        runtime.reset(); //Back up slow
        while (runtime.seconds() < 1) {
            m1.setPower(-.3);
            m2.setPower(-.3);
            m3.setPower(-.3);
            m4.setPower(-.3);
            idle();
        }
        runtime.reset(); //Go diagonal backwards until ball is knocked off and robot is parked
        while (runtime.seconds() < 2) {
            if (gyro.getHeading() != 0 && gyro.getHeading() < 180) {
                m1.setPower(-1);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(-.8);
                idle();
            } else if (gyro.getHeading() > 180) {
                m1.setPower(-.8);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(-1);
                idle();
            } else {
                m1.setPower(0);
                m2.setPower(-1);
                m3.setPower(-1);
                m4.setPower(0);
                idle();
            }
        }
    }
}