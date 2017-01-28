//TeleOp-2017

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp-2017", group = "TeleOp")
public class Teleop2017 extends TardisOpMode { //Init code in separate program

    public enum steps {
        ROTATE,
        NORMAL_DRIVE,
        WHITE_LINE,
        FORWARD,
        COLOR_FIND,
        PUSH,
        STOP,
        SHOOT_BALL
    }

    public steps CURRENT_STEP = steps.NORMAL_DRIVE; //Sets the variable CURRENT_STEP to the first step in the sequence

    //Toggle Boolean states
    boolean sweeperCurr = false; //Toggle sweeper current state
    boolean sweeperPrev = false; //Toggle sweeper previous state
    boolean sweeperOn = false; //Toggle sweeper on state
    boolean shooterCurr = false; //Toggle shooter current state
    boolean shooterPrev = false; //Toggle shooter previous state
    boolean shooterOn = false; //Toggle shooter on state

    int beaconSelect;

    ElapsedTime runtime = new ElapsedTime(); //Runtime

    float LUD; //Left joystick y axis
    float LRL; //Left joystick x axis
    float R;   //Right joystick x axis
    float LUD2; //Left joystick y axis (Game pad 2)
    float RUD; //Right joystick y axis (Game pad 2)
    double threshold = 0.1; //Threshold
    int shooterSpeed = 3; //Shooter speed toggle variable

    @Override
    public void loop() {

        telemetry.addData(">", "Color Red (Ground): " + colorSensor2.red() + "\nColor Blue (Ground): " + colorSensor2.blue() + "\nzValue: " + gyro.getIntegratedZValue() + "\nShooter Speed" + shooterSpeed); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new information

        LUD = gamepad1.left_stick_y;   //(Game pad 1)
        LRL = -gamepad1.left_stick_x;  //(Game pad 1)
        R = gamepad1.right_stick_x;    //(Game pad 1)
        LUD2 = gamepad2.left_stick_y;  //(Game pad 2)
        RUD = -gamepad2.right_stick_y; //(Game pad 2)

        //Check status of right bumper (Game pad 1)
        shooterCurr = gamepad1.left_bumper;

        //Check for button state transitions.
        if ((shooterCurr == true) && (shooterCurr != shooterPrev)) {

            //Shooter transitioning to toggled state
            shooterOn = !shooterOn;
            if (shooterOn) {
                m5.setPower(1); //Shooter On
                m6.setPower(1);
            } else {
                m5.setPower(0.1); //Shooter Slow
                m6.setPower(0.1);
            }
        }

        shooterPrev = shooterCurr; //Sets back to toggleable state

        if (gamepad1.y)  {
            m5.setMaxSpeed(1850);  //Set max speed high for shooter
            m6.setMaxSpeed(1700);
            shooterSpeed = 3; //Shooter speed variable = 3
        }
        if (gamepad1.b)  {
            m5.setMaxSpeed(1750);  //Set max speed medium for shooter
            m6.setMaxSpeed(1600);
            shooterSpeed = 2; //Shooter speed variable = 2
        }
        if (gamepad1.a)  {
            m5.setMaxSpeed(1650);  //Set max speed low for shooter
            m6.setMaxSpeed(1500);
            shooterSpeed = 1; //Shooter speed variable = 1
        }

        //If the joystick button is pressed, move the servo (Game pad 2)
        if (gamepad2.right_stick_button) {
            s1.setPosition(RUD/3 + 0.2); //Linear actuator based on joystick inputs
        }

        //If the left trigger is pressed hold servos up (Game pad2)
        if (gamepad2.left_trigger == 1) {
            SFront.setPosition(.2); //Up
        } else {
            SFront.setPosition(0); //Down
        }

        //Mast code with limit touch sensor switches (Game pad 2)
        if ((LUD2 > threshold) && (!touchSensor2.isPressed())) { //If the limit 2 is not pressed
            m8.setPower(1); //Go up
        } else if ((LUD2 < -threshold) && (!touchSensor1.isPressed())) { //If the limit 1 is not pressed
            m8.setPower(-1); //Go down
        } else {
            m8.setPower(0); //Do nothing
        }

        //Scooper Y is up, b is middle, a is down
        if (gamepad2.a) {
            s3.setPosition(0.06); //Down
        } else if (gamepad2.b) {
            s3.setPosition(0.18); //Mid
        } else if (gamepad2.y) {
            s3.setPosition(0.35); //Up
        }

        //Check status of right bumper
        sweeperCurr = gamepad2.right_trigger == 1;

        //Check for button state transitions.
        if ((sweeperCurr == true) && (sweeperCurr != sweeperPrev)) {

            //Sweeper transitioning to toggled state
            sweeperOn = !sweeperOn;
            if (sweeperOn) {
                m7.setPower(-1); //Sweep in
            } else {
                m7.setPower(0); //Do nothing
            }
        }
        sweeperPrev = sweeperCurr; //Sets back to toggleable state

        if (!sweeperOn && gamepad2.right_bumper) { //If the toggle is off and the trigger is pressed
            m7.setPower(1); //Sweep out
        } else if (!sweeperOn && !gamepad2.right_bumper) {
            m7.setPower(0); //Set sweeper power to 0
        }

        if (gamepad1.left_trigger == 1) {
            runtime.reset();
            CURRENT_STEP = steps.SHOOT_BALL;
        }
        /*if (gamepad1.b) {
            CURRENT_STEP = ROTATE;
            beaconSelect = 0;
        }
        if (gamepad1.x) {
            CURRENT_STEP = ROTATE;
            beaconSelect = 1;
        }
        */
        if(CURRENT_STEP != steps.NORMAL_DRIVE && R > threshold || R < -threshold || LRL > threshold || LRL < -threshold || LUD > threshold || LUD < -threshold) {
            CURRENT_STEP = steps.NORMAL_DRIVE;
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
                        m1.setPower(R / 2); //Set power to negative x-axis (orientation)
                        m2.setPower(-R / 2); //Set power to positive x-axis
                        m3.setPower(R / 2); //Set power to negative x-axis (orientation)
                        m4.setPower(-R / 2); //Set power to positive x-axis
                        break;
                    }

                }
                //If trigger is pressed, go very slow
                else if (gamepad1.right_trigger == 1) {
                    //If R is not used,then use left Stick (R overrides L)
                    if (R == 0) {
                        //Controls for Left Stick
                        m1.setPower((LRL + LUD) / 4);   //Motor1 is x-axis plus y-axis
                        m2.setPower((LUD - LRL) / 4);   //Motor2 is x-axis minus y-axis
                        m3.setPower((LUD - LRL) / 4);   //Motor3 is x-axis minus y axis(parallel to M2)
                        m4.setPower((LRL + LUD) / 4);   //Motor4 is x-axis plus y-axis(parallel to M1)
                        break;
                    }
                    //If R is Outside the threshold, then rotate on the direction of Right Stick
                    if (R > threshold || R < -threshold) {
                        //Controls for Right Stick
                        m1.setPower(R / 6);    //Set power to negative x-axis (orientation)
                        m2.setPower(-R / 6);     //Set power to positive x-axis
                        m3.setPower(R / 6);    //Set power to negative x-axis (orientation)
                        m4.setPower(-R / 6);     //Set power to positive x-axis
                        break;
                    }
                    //If nothing at all is happening, do nothing
                    break;
                }
                break;

            case SHOOT_BALL:

                if (runtime.seconds() < 1.5) { //Wait
                    s2.setPosition(1);
                    break;
                } else {
                    s2.setPosition(.05);
                    CURRENT_STEP = steps.NORMAL_DRIVE;
                    break;
                }
            /*
            case ROTATE:

                ///////
                //RED//
                ///////

                if(gyro.getIntegratedZValue() < -5 && beaconSelect == 0) {
                    m1.setPower(.1);
                    m2.setPower(-.1);
                    m3.setPower(.1);
                    m4.setPower(-.1);
                    break;
                }
                if(gyro.getIntegratedZValue() > 5 && beaconSelect == 0) {
                    m1.setPower(-.07);
                    m2.setPower(.07);
                    m3.setPower(-.07);
                    m4.setPower(.07);
                    break;
                }
                if(gyro.getIntegratedZValue() > -5 && gyro.getIntegratedZValue() < 5 && beaconSelect == 0) {
                    CURRENT_STEP = steps.WHITE_LINE;
                    break;
                }

                ////////
                //BLUE//
                ////////

                if(gyro.getIntegratedZValue() < -95 && beaconSelect == 1) {
                    m1.setPower(.1);
                    m2.setPower(-.1);
                    m3.setPower(.1);
                    m4.setPower(-.1);
                    break;
                }
                if(gyro.getIntegratedZValue() > -85 && beaconSelect == 1) {
                    m1.setPower(-.1);
                    m2.setPower(.1);
                    m3.setPower(-.1);
                    m4.setPower(.1);
                    break;
                }
                if(gyro.getIntegratedZValue() > -95 && gyro.getIntegratedZValue() < -85 && beaconSelect == 1) {
                    CURRENT_STEP = steps.WHITE_LINE;
                    break;
                }

            case WHITE_LINE:

                if(odsSensor1.getRawLightDetected() < .5) {
                    m1.setPower(-.2);
                    m2.setPower(-.2);
                    m3.setPower(-.2);
                    m4.setPower(-.2);
                    break;
                }
                if(odsSensor1.getRawLightDetected() > .5) {
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
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
                    CURRENT_STEP = COLOR_FIND; //Sets next step to NORMAL_DRIVE
                    break; //Exits switch statement
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) < 14) { //If the robot is to close to the beacon, back away
                    m1.setPower(.2); //Sets motor 1 to power -.2 to go backward because the robot is too close to the beacon
                    m2.setPower(-.2); //Sets motor 2 to power -.2 to go backward because the robot is too close to the beacon
                    m3.setPower(-.2); //Sets motor 3 to power -.2 to go backward because the robot is too close to the beacon
                    m4.setPower(.2); //Sets motor 4 to power -.2 to go backward because the robot is too close to the beacon
                    break; //Exits switch statement
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) > 14) { //If the robot is to far away from the beacon, move closer
                    m1.setPower(-.2); //Sets motor 1 to power .2 to go forward because the robot is too far from the beacon
                    m2.setPower(.2); //Sets motor 2 to power .2 to go forward because the robot is too far from the beacon
                    m3.setPower(.2); //Sets motor 3 to power .2 to go forward because the robot is too far from the beacon
                    m4.setPower(-.2); //Sets motor 4 to power .2 to go forward because the robot is too far from the beacon
                    break; //Exits switch statement
                } //End of if statement
                break; //Exits switch statement

            case COLOR_FIND:

                if (colorSensor2.red() > 1.5) { //Moves right until red is found
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = PUSH; //Sets next step to PUSH_BUTTON_BEACON_ONE
                    break; //Exits switch statement
                }
                if (gyro.getIntegratedZValue() < 0) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(.2); //Sets motor 1 to power .2 to go right to scan for the correct color
                    m2.setPower(.2); //Sets motor 2 to power -.2 to go right to scan for the correct color
                    m3.setPower(.3); //Sets motor 3 to power -.3 to go right to scan for the correct color
                    m4.setPower(.3); //Sets motor 4 to power .3 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > 0) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(.3); //Sets motor 1 to power .3 to go right to scan for the correct color
                    m2.setPower(.3); //Sets motor 2 to power -.3 to go right to scan for the correct color
                    m3.setPower(.2); //Sets motor 3 to power -.2 to go right to scan for the correct color
                    m4.setPower(.2); //Sets motor 4 to power .2 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == 0) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .3
                    m1.setPower(.3); //Sets motor 1 to power .3 to go right to scan for the correct color
                    m2.setPower(.3); //Sets motor 2 to power -.3 to go right to scan for the correct color
                    m3.setPower(.3); //Sets motor 3 to power -.3 to go right to scan for the correct color
                    m4.setPower(.3); //Sets motor 4 to power .3 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                break; //Exits switch statement

            case PUSH:

                if (runtime.seconds() > 2) { //Moves forward to push button for 2 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.NORMAL_DRIVE; //Sets next step to BACK_OFF_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.4); //Sets motor 1 to power .4 to go forward to push button
                m2.setPower(.4); //Sets motor 2 to power .4 to go forward to push button
                m3.setPower(.4); //Sets motor 3 to power .4 to go forward to push button
                m4.setPower(-.4); //Sets motor 4 to power .4 to go forward to push button
                break; //Exits switch statement */
        }
    } //Void loop end
} //OpMode loop end
