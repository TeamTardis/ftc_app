//TeleOp-2017

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOp-2017", group = "TeleOp")
public class Teleop2017 extends TardisOpMode { //Init code in separate program

    //Toggle Boolean states
    boolean sweeperCurr = false; //Toggle sweeper current state
    boolean sweeperPrev = false; //Toggle sweeper previous state
    boolean sweeperOn = false; //Toggle sweeper on state
    boolean shooterCurr = false; //Toggle shooter current state
    boolean shooterPrev = false; //Toggle shooter previous state
    boolean shooterOn = false; //Toggle shooter on state

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

        telemetry.addData("Shooter Speed", shooterSpeed); //Adds telemetry for shooter speed variable
        telemetry.update(); //Updates telemetry with new debug information

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
            m7.setPower(0); //Do nothing
        }



        //Driver controls (Game pad 1)
        //If right trigger is not pressed, go normal
        if (gamepad1.right_trigger == 0) {
            //If R is not used,then use left Stick (R overrides L)
            if (R == 0) {
                //Controls for Left Stick
                m1.setPower((LRL + LUD)); //Motor1 is x-axis plus y-axis
                m2.setPower((LUD - LRL)); //Motor2 is x-axis minus y-axis
                m3.setPower((LUD - LRL)); //Motor3 is x-axis minus y axis(parallel to m2)
                m4.setPower((LRL + LUD)); //Motor4 is x-axis plus y-axis(parallel to m1)
            }
            //If R is Outside the threshold, then rotate on the direction of Right Stick
            else if (R > threshold || R < -threshold) {
                //Controls for Right Stick
                m1.setPower(R);  //Set power to negative x-axis (orientation)
                m2.setPower(-R); //Set power to positive x-axis
                m3.setPower(R);  //Set power to negative x-axis (orientation)
                m4.setPower(-R); //Set power to positive x-axis
            }
        }

        //If right trigger is pressed, control slow
        //If right stick is not used,then use left Stick (R overrides L)
        if (gamepad1.right_trigger == 1) {
            if (R == 0) {
                //Controls for Left Stick
                m1.setPower((LRL + LUD) / 4); //Motor1 is x-axis plus y-axis
                m2.setPower((LUD - LRL) / 4); //Motor2 is x-axis minus y-axis
                m3.setPower((LUD - LRL) / 4); //Motor3 is x-axis minus y axis(parallel to M2)
                m4.setPower((LRL + LUD) / 4); //Motor4 is x-axis plus y-axis(parallel to M1)
            }
            //If R is Outside the threshold, then rotate on the direction of Right Stick
            else if (R > threshold || R < -threshold) {
                //Controls for Right Stick
                m1.setPower(R / 6);  //Set power to negative x-axis (orientation)
                m2.setPower(-R / 6); //Set power to positive x-axis
                m3.setPower(R / 6);  //Set power to negative x-axis (orientation)
                m4.setPower(-R / 6); //Set power to positive x-axis
            }
        }
    } //Void loop end
} //OpMode loop end
