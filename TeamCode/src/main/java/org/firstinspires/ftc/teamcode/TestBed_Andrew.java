package org.firstinspires.ftc.teamcode;
/**
 * Created by Corning Robotics on 9/25/16.
 */
import android.hardware.Sensor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.TestBed_Andrew.steps.FORWARD;
import static org.firstinspires.ftc.teamcode.TestBed_Andrew.steps.NORMAL_DRIVE;
import static org.firstinspires.ftc.teamcode.TestBed_Andrew.steps.ROTATE;
import static org.firstinspires.ftc.teamcode.TestBed_Andrew.steps.WHITE_LINE;

@TeleOp(name="TestBed-2016", group="TeleOp")
public class TestBed_Andrew extends TardisOpModeAutonomous {
    public void init() { //Start of the initiation for autonomous
        m1 = hardwareMap.dcMotor.get("m1"); //Sets m1 to m3 in the config
        m2 = hardwareMap.dcMotor.get("m2"); //Sets m2 to m1 in the config
        m3 = hardwareMap.dcMotor.get("m3"); //Sets m3 to m4 in the config
        m4 = hardwareMap.dcMotor.get("m4"); //Sets m4 to m2 in the config
        m2.setDirection(DcMotor.Direction.REVERSE); //Reverses direction of m2
        m4.setDirection(DcMotor.Direction.REVERSE); //Reverses direction of m4

        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range"); //Sets range to range in the config

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro"); //Sets gyro to gyro in the config

        gyro.calibrate(); //Calibrate the gyro sensor
        while (gyro.isCalibrating()) { //Adds telemetry for gyro calibration
            telemetry.addData("<", "Gyro calibrating..."); //Tells the user the gyro is calibrating
            telemetry.update(); //Updates telemetry
        } //End of while statement
        telemetry.addData("<", "Gyro calibrated, good luck!"); //Tells the user the gyro has finished calibrating
        telemetry.update(); //Updates telemetry
    } //Ends initiation

////////////////////////////////////////////////////////////////////////////////////////////////////


    public TestBed_Andrew() {
    }

    public enum steps {
        ROTATE,
        NORMAL_DRIVE,
        WHITE_LINE,
        FORWARD,
    }

    boolean linearExtention = false;
    boolean beacon = false;
    boolean push = false;



    public steps CURRENT_STEP = NORMAL_DRIVE; //Sets the variable CURRENT_STEP to the first step in the sequence

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void loop() {

        float LUD = gamepad1.left_stick_y;
        float LRL = -gamepad1.left_stick_x;
        float R = gamepad1.right_stick_x;
        float LTrigger = gamepad1.left_trigger;
        double threshold = 0.1;

        if (gamepad1.a) {
            CURRENT_STEP = ROTATE;
        }
        if(gamepad1.x) {
            CURRENT_STEP = NORMAL_DRIVE;
        }

        switch (CURRENT_STEP) {

            case NORMAL_DRIVE:

                //If no buttons are pressed, control normal
                //If R is not used,then use left Stick (R overrides L)
                if (gamepad1.right_trigger == 0) {

                    if (R == 0) {
                        //Controls for Left Stick
                        m1.setPower((LRL + LUD) / 2);   //Motor1 is x-axis plus y-axis
                        m2.setPower((LUD - LRL) / 2);   //Motor2 is x-axis minus y-axis
                        m3.setPower((LUD - LRL) / 2);   //Motor3 is x-axis minus y axis(parallel to M2)
                        m4.setPower((LRL + LUD) / 2);   //Motor4 is x-axis plus y-axis(parallel to M1)
                        break;
                    }
                    //If R is Outside the threshold, then rotate on the direction of Right Stick
                    if (R > threshold || R < -threshold) {
                        //Controls for Right Stick
                        m1.setPower(-R);    //Set power to negative x-axis (orientation)
                        m2.setPower(R);     //Set power to positive x-axis
                        m3.setPower(-R);    //Set power to negative x-axis (orientation)
                        m4.setPower(R);     //Set power to positive x-axis
                        break;
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
                        break;
                    }
                    //If R is Outside the threshold, then rotate on the direction of Right Stick
                    if (R > threshold || R < -threshold) {
                        //Controls for Right Stick
                        m1.setPower(-R / 6);    //Set power to negative x-axis (orientation)
                        m2.setPower(R / 6);     //Set power to positive x-axis
                        m3.setPower(-R / 6);    //Set power to negative x-axis (orientation)
                        m4.setPower(R / 6);     //Set power to positive x-axis
                        break;
                    }
                    //If nothing at all is happening, do nothing
                break;
                }

            case ROTATE:

                if(gyro.getIntegratedZValue() < -5) {
                    m1.setPower(.09);
                    m2.setPower(.09);
                    m3.setPower(-.09);
                    m4.setPower(-.09);
                    break;
                }
                if(gyro.getIntegratedZValue() > 5) {
                    m1.setPower(-.09);
                    m2.setPower(-.09);
                    m3.setPower(.09);
                    m4.setPower(.09);
                    break;
                }
                if(gyro.getIntegratedZValue() > -5 && gyro.getIntegratedZValue() < 5) {
                    CURRENT_STEP = WHITE_LINE;
                    break;
                }

            case WHITE_LINE:

                if(odsSensor1.getRawLightDetected() < .5) {
                    m1.setPower(-.1);
                    m2.setPower(.1);
                    m3.setPower(.1);
                    m4.setPower(-.1);
                    break;
                }
                if(odsSensor1.getRawLightDetected() < .5) {
                    CURRENT_STEP = FORWARD;
                    break;
                }

            case FORWARD:

                if (range.getDistance(DistanceUnit.CM) == 14) { //Move the robot closer or farther from the beacon wall to get correct distance to sense colors
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.NORMAL_DRIVE; //Sets next step to NORMAL_DRIVE
                    break; //Exits switch statement
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) < 14) { //If the robot is to close to the beacon, back away
                    m1.setPower(0); //Sets motor 1 to power -.2 to go backward because the robot is too close to the beacon
                    m2.setPower(-.2); //Sets motor 2 to power -.2 to go backward because the robot is too close to the beacon
                    m3.setPower(-.2); //Sets motor 3 to power -.2 to go backward because the robot is too close to the beacon
                    m4.setPower(0); //Sets motor 4 to power -.2 to go backward because the robot is too close to the beacon
                    break; //Exits switch statement
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) > 14) { //If the robot is to far away from the beacon, move closer
                    m1.setPower(.2); //Sets motor 1 to power .2 to go forward because the robot is too far from the beacon
                    m2.setPower(0); //Sets motor 2 to power .2 to go forward because the robot is too far from the beacon
                    m3.setPower(0); //Sets motor 3 to power .2 to go forward because the robot is too far from the beacon
                    m4.setPower(.2); //Sets motor 4 to power .2 to go forward because the robot is too far from the beacon
                    break; //Exits switch statement
                } //End of if statement
                break; //Exits switch statement
        }
        }
    }
