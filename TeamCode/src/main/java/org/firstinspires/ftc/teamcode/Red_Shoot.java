package org.firstinspires.ftc.teamcode; //Use the package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor; //Import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor; //Import com.qualcomm.robotcore.hardware.ColorSensor for the color sensors
import com.qualcomm.robotcore.hardware.DcMotor; //Import com.qualcomm.robotcore.hardware.DcMotor for motors
import com.qualcomm.robotcore.hardware.GyroSensor; //Import com.qualcomm.robotcore.hardware.GyroSensor for the gyro sensor
import com.qualcomm.robotcore.hardware.I2cAddr; //Import com.qualcomm.robotcore.hardware.I2cAddr to allow to change I2c addresses
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor; //Import com.qualcomm.robotcore.hardware.OpticalDistanceSensor for the optical distance sensor
import com.qualcomm.robotcore.hardware.Servo; //Import com.qualcomm.robotcore.hardware.Servo for servos
import com.qualcomm.robotcore.hardware.TouchSensor; //Import com.qualcomm.robotcore.hardware.TouchSensor for touch sensors
import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //Imports com.qualcomm.robotcore.eventloop.opmode.Autonomous for autonomous additions
import com.qualcomm.robotcore.eventloop.opmode.OpMode; //Imports com.qualcomm.robotcore.eventloop.opmode.OpMode for opmode additions
import com.qualcomm.robotcore.util.ElapsedTime; //Imports com.qualcomm.robotcore.util.ElapsedTime for timed events
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; //Imports org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

/**
 * Created by Corning Robotics on 12/21/16. *
 **/

/**
 ////////////////////////////////////////////////////////////////////////////////////////////////////
 //Definitions (just to be sure what you're thinking and what we're talking about are the same :) )//
 ////////////////////////////////////////////////////////////////////////////////////////////////////

 Backup - An emergency step or precaution added for a double check or to allow for a second chance at completing a task.
 Backward - The opposite direction of forward (see definition "forward").
 Beacon one - The beacon closest to the corner vortex is beacon one.
 Beacon two - The beacon farthest from the corner vortex is beacon two.
 Beacon wall - The wall where two corresponding beacons are placed.
 Button pusher - The 3D printed piece of spiked plastic used to push beacon buttons.
 Drive train motors - The motors that have contact with the surface of the mat and help the robot move (motor 1, motor 2, motor 3, and motor 4).
 Forward - The side of the robot where the button pusher (see definition "button pusher") is placed. For example, if the button pusher was facing toward the starting wall and the robot moved forward, the robot would get closer to the wall.
 Launcher motors - The two motors (motor 5 and motor 6) that speed up and allow the ball to be shot.
 Launching position - The position the launching servo (see definition "launching servo") must be in so the ball makes contact with the launcher (see definition "launcher").
 Launching servo - The servo (servo 2) that lifts the ball from the teleporter (see definition "teleporter") into the launcher (see definition "launcher").
 Left - The left side of the robot when facing the same way as the button pusher (see definition "button pusher").
 ODS - The abbreviation for the optical distance sensor.
 Original position (in the case of the launching servo) - Servo position .05. This position allows for the next ball to roll into the launching servo (see definition "launching servo").
 Right - The right side of the robot when facing the same way as the button pusher (see definition "button pusher").
 Launcher - The combination of both launcher motors (see definition "launcher motors").
 Starting wall - The wall that the robot initially starts on.
 Sweeper - The ball collector that also kicks balls into the teleporter (see definition "teleporter").
 Teleporter - The name we use to describe the tube between our sweeperOn (see definition "sweeperOn") and our launching mechanism (see definition "launching servo").
 White line - The white tape on the floor perpendicular to a beacon.
 **/

@Autonomous(name = "RED_SHOOT", group = "AutoFast") //Display name and group found in on controller phone
@Disabled
public class Red_Shoot extends TardisOpModeAutonomous { //Imports presets for initiation from TardisOpModeAutonomous

    public enum steps { //All steps for completing autonomous

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Main steps - These steps run in sequence when the program is gong perfect with no unexpected readings in sensor inputs.//
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        START_UP_LAUNCHER_AND_COLLECTOR_MOTORS, //[Prep] Sets speed of sweeperOn motor to -1 to kick balls into teleporter and sets launcher motors to .1 so they don't go from 0 to 100 real quick (persecution to avoid gear-grinding)
        MOVE_AWAY_FROM_WALL, //[Step 1] Moves away from starting wall to prepare for rotation
        ROTATE_TO_VORTEX, //[Step 2] Rotate about 50° counter-clockwise to face launcher at center vortex
        SPEED_UP_SHOOTERS, //[Step 3] Speeds up launcher motors to proper launching speed (Max speed = 1600 (see TardisOpModeAutonomous for more information))
        SHOOT_FIRST_BALL, //[Step 4] Shoot first ball by lifting ball using launching servo
        LOWER_SERVO_1, //[Step 5] Lowers launching servo (resets it to original position)
        SHOOT_SECOND_BALL, //[Step 6] Shoot second ball by lifting ball using launching servo
        LOWER_SERVO_2, //[Step 7] Lowers launching servo (resets it to original position)- this step also sets the launcher motors' power .1 and turns the sweeperOn off
        ROTATE_BACK, //[Step 8] Rotates back 50° clockwise to face button pusher towards beacon- this step also sets the launcher's motors to power 0 for power conservation
        STRAIGHTEN_ON_WALL, //[Step 9] Pushes against starting wall as backup
        DRIVE_TO_FIRST_BEACON, //[Step 10] Drive diagonally using motor 2, motor 3, and gyro sensor until the beacon one white line is detected
        DRIVE_OFF_WHITE_LINE_BEACON_ONE, //[Step 11] Move off white line to right side of the beacon to prepare for scanning of correct color
        MOVE_CLOSER_TO_BEACON_ONE, //[Step 12] Use the range sensor to move the correct distance away from the wall
        FIND_CORRECT_COLOR_BEACON_ONE, //[Step 13] Drive right to locate and stop at correct color while scanning beacon
        PUSH_BUTTON_BEACON_ONE, //[Step 14] Once correct color is found, drive forward to select color
        BACK_OFF_BEACON_ONE, //[Step 15] After button press, drive backward
        NEXT_BEACON_STEP_ONE, //[Step 16] Moves right to get off white line to prepare to look for the beacon two white line
        NEXT_BEACON_STEP_TWO, //[Step 17] Using gyro for stabilization, begin to start scan for the beacon two white line
        DRIVE_OFF_WHITE_LINE_BEACON_TWO, //[Step 18] Move off white line to left side of the beacon to prepare to scan for correct color
        MOVE_CLOSER_TO_BEACON_TWO, //[Step 19] Use the range sensor to move the correct distance away from the wall
        FIND_CORRECT_COLOR_BEACON_TWO, //[Step 20] Drive right to locate and stop at correct color while scanning beacon
        PUSH_BUTTON_BEACON_TWO, //[Step 21] Once correct color is found, drive forward to select color
        BACK_OFF_BEACON_TWO, //[Step 22] After button press, drive backward
        HIT_CAP_BALL, //[Step 23] Drive diagonally backward using motor 1, motor 4 and gyro sensor to hit cap ball off the base of the center vortex and park there
        STOP, //The title says it all

        ///////////////////////////////////////////////////////////////////////////////////////////////
        //Backup steps - These steps run incase there is an unexpected sensor reading or none at all.//
        ///////////////////////////////////////////////////////////////////////////////////////////////

        UNEXPECTED_NO_COLOR_BEACON_TWO //If the correct color is not found within 3 seconds of the step FIND_CORRECT_COLOR_BEACON_TWO, this step will move the robot closer to the beacon so it can rescan

    } //End of steps for autonomous

    public steps CURRENT_STEP = steps.START_UP_LAUNCHER_AND_COLLECTOR_MOTORS; //Sets the variable CURRENT_STEP to the first step in the sequence

    private ElapsedTime runtime = new ElapsedTime(); //Creates a variable for runtime so we can have timed events

    @Override //Method overrides parent class
    public void loop() { //Starts loop for the program

        ///////////////////////////
        //Telemetry for debugging//
        ///////////////////////////

        telemetry.addData(">", "Heading: " + gyro.getHeading() + "\nRed: " + colorSensor.red() + "\nRuntime variable: " + runtime + "\nStep: " + CURRENT_STEP); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new information

        m5.setMaxSpeed(1750);  //Set max speed medium for shooter
        m6.setMaxSpeed(1600);

        /////////////////////////////
        //Start of switch statement//
        /////////////////////////////

        switch (CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever CURRENT_STEP is set to

            ///////////////////////
            //START OF MAIN STEPS//
            ///////////////////////

            /////////////////////////////////////////////////
            //Prep [START_UP_LAUNCHER_AND_COLLECTOR_MOTORS]//
            /////////////////////////////////////////////////

            case START_UP_LAUNCHER_AND_COLLECTOR_MOTORS: //Beginning of case statement START_UP_LAUNCHER_AND_COLLECTOR_MOTORS

                m5.setPower(.1); //Set launcher motor 5 to .1 for prep
                m6.setPower(.1); //Set launcher motor 6 to .1 for prep
                m7.setPower(-1); //Sweeper motor 7 to -1 for prep
                s3.setPosition(0.35); //Forks pointed up
                runtime.reset(); //Resets time before switching to next step
                CURRENT_STEP = steps.SPEED_UP_SHOOTERS; //Sets next step to MOVE_AWAY_FROM_WALL
                break; //Exits switch statement

            case SPEED_UP_SHOOTERS: //Beginning of case statement SPEED_UP_SHOOTERS

                if (runtime.seconds() > 3) { //Sets power of launcher motors to 1 with their max speed set to 1600 (see TardisOpModeAutonomous) for 1.5 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SHOOT_FIRST_BALL; //Sets next step to SHOOT_FIRST_BALL
                    break; //Exits switch statement
                } //End of if statement
                m5.setPower(1); //Sets launcher motor 5 to 1 to prepare to shoot ball
                m6.setPower(1); //Sets launcher motor 6 to 1 to prepare to shoot ball
                break; //Exits switch statement

            /////////////////////////////
            //Step 4 [SHOOT_FIRST_BALL]//
            /////////////////////////////

            case SHOOT_FIRST_BALL: //Beginning of case statement SHOOT_FIRST_BALL

                if (runtime.seconds() > 1) { //Raises ball into launcher by setting servo position to launching position for 1 second
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.LOWER_SERVO_1; //Sets next step to LOWER_SERVO_1
                    break; //Exits switch statement
                } //End of if statement
                s2.setPosition(1); //Sets launching servo to launching position
                break; //Exits switch statement

            //////////////////////////
            //Step 5 [LOWER_SERVO_1]//
            //////////////////////////

            case LOWER_SERVO_1: //Beginning of case statement LOWER_SERVO_1

                if (runtime.seconds() > 3) { //Lowers launching servo to original position for 1.5 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SHOOT_SECOND_BALL; //Sets next step to SHOOT_SECOND_BALL
                    break; //Exits switch statement
                } //End of if statement
                m5.setPower(1); //Sets launcher motor 5 to 1 to prepare to shoot ball
                m6.setPower(1); //Sets launcher motor 6 to 1 to prepare to shoot ball
                s2.setPosition(.05); //Sets launching servo to original position
                break; //Exits switch statement

            //////////////////////////////
            //Step 6 [SHOOT_SECOND_BALL]//
            //////////////////////////////

            case SHOOT_SECOND_BALL: //Beginning of case statement SHOOT_SECOND_BALL

                if (runtime.seconds() > 2) { //Raises ball into launcher by setting servo position to launching position for 1 second
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.LOWER_SERVO_2; //Sets next step to LOWER_SERVO_2
                    break; //Exits switch statement
                } //End of if statement
                s2.setPosition(1); //Sets launching servo to launching position
                break; //Exits switch statement

            //////////////////////////
            //Step 7 [LOWER_SERVO_2]//
            //////////////////////////

            case LOWER_SERVO_2: //Beginning of case statement LOWER_SERVO_2

                if (runtime.seconds() > 1) { //Lowers launching servo to original position, set the launcher motors' power to .1 and .1 and stops the sweeper for 1 second
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.ROTATE_BACK; //Sets next step to ROTATE_BACK
                    break; //Exits switch statement
                } //End of if statement
                m5.setPower(.1); //Sets launcher motor's power to .1 to slow it down
                m6.setPower(.1); //Sets launcher motor's power to .1 to slow it down
                m7.setPower(0); //Sets sweeper motor's power to 0 to turn it off
                s2.setPosition(.05); //Sets launching servo to original position
                break; //Exits switch statement

            case STOP: //Beginning of the case statement STOP

                m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                m5.setPower(0); //Sets motor 5 power to 0 to make sure it is not moving
                m6.setPower(0); //Sets motor 6 power to 0 to make sure it is not moving
                m7.setPower(0); //Sets motor 7 power to 0 to make sure it is not moving
                m8.setPower(0); //Sets motor 8 power to 0 to make sure it is not moving
                break; //Exits switch statement

        } //End of loop
    } //End of program
}