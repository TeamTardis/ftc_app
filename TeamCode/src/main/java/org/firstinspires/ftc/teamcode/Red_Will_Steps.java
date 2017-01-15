package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //Imports com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode; //Imports com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime; //Imports com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Corning Robotics on 12/21/16. *
 **/

/**
 ////////////////////////////////////////////////////////////////////////////////////////////////////
 //Definitions (just to be sure what you're thinking and what we're talking about are the same :) )//
 ////////////////////////////////////////////////////////////////////////////////////////////////////

 Backup - An emergency step or percaution added for a double check or to allow for a second chance at completing a task.
 Backward - The opposite direction of forward (see definition "forward").
 Beacon one - The beacon closest to the corner vortex is beacon one.
 Beacon two - The becaon farthest from the corner vortex is beacon two.
 Beacon wall - The wall where two corresponding beacons are placed.
 Button pusher - The 3D printed piece of spiked plastic used to push beacon buttons.
 Drive train motors - The motors that have contact with the surface of the mat and help the robot move (motor 1, motor 2, motor 3, and motor 4).
 Forward - The side of the robot where the button pusher (see definition "button pusher") is placed. For example, if the button pusher was facing toward the starting wall and the robot moved forward, the robot would get closer to the wall.
 Launcher motors - The two motors (motor 5 and motor 6) that speed up and allow the ball to be shot.
 Launching position - The position the launching servo (see definition "launching servo") must be in so the ball makes contact with the launcher (see definition "launcher").
 Launching servo - The servo (servo 2) that lifts the ball from the teleporter (see definition "teleporter") into the launcher (see definition "launcher").
 Left - The left side of the robot when facing the same way as the button pusher (see definition "button pusher").
 Original position (in the case of the launching servo) - Servo position .05. This postition allows for the next ball to roll into the launching servo (see definition "launching servo").
 Right - The right side of the robot when facing the same way as the button pusher (see definition "button pusher").
 Launcher - The combination of both launcher motors (see definition "launcher motors").
 Starting wall - The wall that the robot initially starts on.
 Sweeper - The ball collector that also kicks balls into the teleporter (see definition "teleporter").
 Teleporter - The name we use to describe the tube between our sweeperOn (see definition "sweeperOn") and our launching mechinism (see definition "launching servo").
 White line - The white tape on the floor perpendicular to a beacon.
 **/
@Disabled
@Autonomous(name = "Red Will", group = "AutoFast")
//Display name and group found in on controller phone
public class Red_Will_Steps extends TardisOpModeAutonomous { //Imports presets for initiation from TardisOpModeAutonomus

    public enum steps { //All steps for completing autonomous

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Main steps - These steps run in sequence when the program is gong perfect with no unexpected readings in sensor inputs.//
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        MOVE_CLOSER_TO_BEACON,
        MOVE_CLOSER_TO_BEACON_2,

        START_UP_LAUNCHER_AND_COLLECTOR_MOTORS, //[Prep] Sets speed of sweeperOn motor to -1 to kick balls into teleporter and sets launcher motors to .1 so they don't go from 0 to 100 real quick (percauction to avoid gear-grinding)
        MOVE_AWAY_FROM_WALL, //[Step 1] Moves away from starting wall to prepare for rotation
        ROTATE_TO_VORTEX, //[Step 2] Rotate about 25° clockwise to face launcher at center vortex
        SPEED_UP_SHOOTERS, //[Step 3] Speeds up launcher motors to proper launching speed (Max speed = 1050 (see TardisOpModeAutonomus for more information))
        SHOOT_FIRST_BALL, //[Step 4] Shoot first ball by lifting ball using launching servo
        LOWER_SERVO_1, //[Step 5] Lowers launching servo (resets it to original position)
        SHOOT_SECOND_BALL, //[Step 6] Shoot second ball by lifting ball using launching servo
        LOWER_SERVO_2, //[Step 7] Lowers launching servo (resets it to original position)- this step also sets the launcher motors' to power .1 and turns the sweeperOn off
        ROTATE_BACK, //[Step 8] Rotates back 25° counter-clockwise to face button pusher towards beacon- this step also sets the launcher's motors to power 0 for power conservation
        STRAIGHTEN_ON_WALL, //[Step 9] Pushes against starting wall as backup
        DRIVE_TO_FIRST_BEACON, //[Step 10] Drive diagonally using motor 1, motor 4, and gyro sensor until the beacon one white line is detected
        DRIVE_OFF_WHITE_LINE_BEACON_ONE, //[Step 11] Move off white line to left side of the beacon to prepare for scanning of correct color
        FIND_CORRECT_COLOR_BEACON_ONE, //[Step 12] Drive right to locate and stop at correct color while scanning beacon
        PUSH_BUTTON_BEACON_ONE, //[Step 13] Once correct color is found, drive forward to select color
        BACK_OFF_BEACON_ONE, //[Step 14] After button press, drive backward
        NEXT_BEACON_STEP_ONE, //[Step 15] Moves right to get off white line to prepare to look for the beacon two white line
        NEXT_BEACON_STEP_TWO, //[Step 16] Using gyro for stablization, begin to start scan for the beacon two white line
        DRIVE_OFF_WHITE_LINE_BEACON_TWO, //[Step 17] Move off white line to left side of the beacon to prepare to scan for correct color
        FIND_CORRECT_COLOR_BEACON_TWO, //[Step 18] Drive right to locate and stop at correct color while scanning beacon
        PUSH_BUTTON_BEACON_TWO, //[Step 19] Once correct color is found, drive forward to select color
        BACK_OFF_BEACON_TWO, //[Step 20] After button press, drive backward
        HIT_CAP_BALL, //[Step 21] Drive diagonally backward using motor 1, motor 4, and gyro sensor to hit cap ball off the base of the center vortex and park there
        STOP, //The title says it all

        ///////////////////////////////////////////////////////////////////////////////////////////////
        //Backup steps - These steps run incase there is an unexpected sensor reading or none at all.//
        ///////////////////////////////////////////////////////////////////////////////////////////////

        UNEXPECTED_NO_COLOR_BEACON_ONE, //If the correct color is not found within 3 seconds of the step FIND_CORRECT_COLOR_BEACON_ONE, this step will move the robot closer to the beacon so it can rescan
        UNEXPECTED_NO_COLOR_BEACON_TWO //Same as above, but this time for beacon two.

    } //End of steps for autonomus

    public steps CURRENT_STEP = steps.START_UP_LAUNCHER_AND_COLLECTOR_MOTORS; //Sets the varible CURRENT_STEP to the first step in the sequence

    private ElapsedTime runtime = new ElapsedTime(); //Creates a varible for runtime so we can have timed events

    @Override //Method overrides parent class
    public void loop() { //Starts loop for the program

        ///////////////////////////
        //Telemetry for debugging//
        ///////////////////////////

        telemetry.addData(">", "Heading: " + gyro.getHeading() + "\nRed: " + colorSensor.red() + "\nRuntime varible: " + runtime + "\nStep: " + CURRENT_STEP); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new debug information

        /////////////////////////////
        //Start of switch statement//
        /////////////////////////////

        switch (CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever the CURRENT_STEP varible is set to

            ///////////////////////
            //START OF MAIN STEPS//
            ///////////////////////

            /////////////////////////////////////////////////
            //Prep [START_UP_LAUNCHER_AND_COLLECTOR_MOTORS]//
            /////////////////////////////////////////////////

            case START_UP_LAUNCHER_AND_COLLECTOR_MOTORS:

                m5.setPower(.1); //Set launcher motor 5 to -.1 for prep
                m6.setPower(.1); //Set launcher motor 6 to .1 for prep
                m7.setPower(-1); //Sweeper motor 7 to -1 for prep
                runtime.reset(); //Resets time before switching to next step
                CURRENT_STEP = steps.MOVE_AWAY_FROM_WALL; //Sets next step to MOVE_AWAY_FROM_WALL
                break; //Exits switch statement

            ////////////////////////////////
            //Step 1 [MOVE_AWAY_FROM_WALL]//
            ////////////////////////////////

            case MOVE_AWAY_FROM_WALL:

                if (runtime.seconds() > .3) { //Moves robot away from wall for .3 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.ROTATE_TO_VORTEX; //Sets next step to ROTATE_TO_VORTEX
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.5); //Sets motor 1 to power .5 to move the robot toward the center vortex
                m2.setPower(-.5); //Sets motor 2 to power -.5 to move the robot toward the center vortex
                m3.setPower(-.5); //Sets motor 3 to power -.5 to move the robot toward the center vortex
                m4.setPower(.5); //Sets motor 4 to power .5 to move the robot toward the center vortex
                break; //Exits switch statement

            /////////////////////////////
            //Step 2 [ROTATE_TO_VORTEX]//
            /////////////////////////////

            case ROTATE_TO_VORTEX:

                if (gyro.getHeading() > 25 && gyro.getHeading() < 200) { //Rotates robot about 25° clockwise so the launcher faces the center vortex
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SPEED_UP_SHOOTERS; //Sets next step to SPEED_UP_SHOOTERS
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.1); //Sets motor 1 to power .1 to rotate the robot clockwise
                m2.setPower(-.1); //Sets motor 2 to power -.1 to rotate the robot clockwise
                m3.setPower(.1); //Sets motor 3 to power .1 to rotate the robot clockwise
                m4.setPower(-.1); //Sets motor 4 to power -.1 to rotate the robot clockwise
                break; //Exits switch statement

            //////////////////////////////
            //Step 3 [SPEED_UP_SHOOTERS]//
            //////////////////////////////

            case SPEED_UP_SHOOTERS:

                if (runtime.seconds() > .5) { //Sets power of launcher motors to 1 with their max speed set to 1050 (see TardisOpModeAutonomus) for .5 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SHOOT_FIRST_BALL; //Sets next step to SHOOT_FIRST_BALL
                    break; //Exits switch statement
                } //End of if statement
                m5.setPower(1); //Sets launcher motor 5 to -1 to prepare to shoot ball
                m6.setPower(1); //Sets launcher motor 6 to 1 to prepare to shoot ball
                break; //Exits switch statement

            /////////////////////////////
            //Step 4 [SHOOT_FIRST_BALL]//
            /////////////////////////////

            case SHOOT_FIRST_BALL:

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

            case LOWER_SERVO_1:

                if (runtime.seconds() > 1.5) { //Lowers launching servo to original postition for 1.5 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SHOOT_SECOND_BALL; //Sets next step to SHOOT_SECOND_BALL
                    break; //Exits switch statement
                } //End of if statement
                s2.setPosition(.05); //Launching servo to original position
                break; //Exits switch statement

            //////////////////////////////
            //Step 6 [SHOOT_SECOND_BALL]//
            //////////////////////////////

            case SHOOT_SECOND_BALL:

                if (runtime.seconds() > 1) { //Raises ball into launcher by setting servo position to launching position for 1 second
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

            case LOWER_SERVO_2:

                if (runtime.seconds() > 1) { //Lowers launching servo to original postition, set the launcher motors' power to -.1 and .1 and stops the sweeperOn for 1 second
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.ROTATE_BACK; //Sets next step to ROTATE_BACK
                    break; //Exits switch statement
                } //End of if statement
                m5.setPower(.1); //Sets launcher motor's power to -.1 to slow it down
                m6.setPower(.1); //Sets launcher motor's power to .1 to slow it down
                m7.setPower(0); //Sets sweeperOn motor's power to 0 to turn it off
                s2.setPosition(.05); //Launching servo to original position
                break; //Exits switch statement

            ////////////////////////
            //Step 8 [ROTATE_BACK]//
            ////////////////////////

            case ROTATE_BACK:

                if (gyro.getHeading() < 8) { //Rotate counter-clockwise to face starting direction- beacon pusher toward beacons and turn off launcher motors
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.STRAIGHTEN_ON_WALL; //Sets next step to STRAIGHTEN_ON_WALL
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.1); //Sets motor 1 to power -.1 to rotate the robot counter-clockwise
                m2.setPower(.1); //Sets motor 2 to power .1 to rotate the robot counter-clockwise
                m3.setPower(-.1); //Sets motor 3 to power -.1 to rotate the robot counter-clockwise
                m4.setPower(.1); //Sets motor 4 to power .1 to rotate the robot counter-clockwise
                m5.setPower(0); //Sets lanucher motor 5 to power 0 to stop it
                m6.setPower(0); //Sets lanucher motor 6 to power 0 to stop it
                break; //Exits switch statement

            ///////////////////////////////
            //Step 9 [STRAIGHTEN_ON_WALL]//
            ///////////////////////////////

            case STRAIGHTEN_ON_WALL:

                if (runtime.seconds() > 1) { //Moves back to starting wall, pushing up against it to align with it for 1 second
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.DRIVE_TO_FIRST_BEACON; //Sets next step to DRIVE_TO_FIRST_BEACON
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.3); //Sets motor 1 to power -.3 to go left to straighten against wall
                m2.setPower(.3); //Sets motor 2 to power .3 to go left to straighten against wall
                m3.setPower(.3); //Sets motor 3 to power .3 to go left to straighten against wall
                m4.setPower(-.3); //Sets motor 4 to power -.3 to go left to straighten against wall
                break; //Exits switch statement

            ///////////////////////////////////
            //Step 10 [DRIVE_TO_FIRST_BEACON]//
            ///////////////////////////////////

            case DRIVE_TO_FIRST_BEACON: //Possible reverse

                if (odsSensor1.getRawLightDetected() > .3) { //Moves diagonally towards first beacon and stops when ODS sences a white line
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.DRIVE_OFF_WHITE_LINE_BEACON_ONE; //Sets next step to MOVE_CLOSER_TO_BEACON
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() < 180) { //If gyro senses a tilt, it lowers the speed of motor 4 to correct itself
                    m1.setPower(.8); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(0); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(0); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(1); //Sets motor 4 to power .8 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() > 180) { //If gyro senses a tilt, it lowers the speed of motor 1 to correct itself
                    m1.setPower(1); //Sets motor 1 to power .8 to go diagonally toward beacon one
                    m2.setPower(0); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(0); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(.8); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() == 0) { //If gyro senses no tilt, it will continue to move at full power with both motor 1 and 4
                    m1.setPower(1); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(0); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(0); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(1); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement

                /////////////////////////////////////////////
                //Step 11 [DRIVE_OFF_WHITE_LINE_BEACON_ONE]//
                /////////////////////////////////////////////

            case DRIVE_OFF_WHITE_LINE_BEACON_ONE:

                if (runtime.seconds() > .8) { //Moves left in order to prepare for scan of colors for .8 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.MOVE_CLOSER_TO_BEACON; //Sets next step to FIND_CORRECT_COLOR_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.3); //Sets motor 1 to power -.3 to move left
                m2.setPower(.3); //Sets motor 2 to power .3 to move left
                m3.setPower(.3); //Sets motor 3 to power .3 to move left
                m4.setPower(-.3); //Sets motor 4 to power -.3 to move left
                break; //Exits switch statement

            ///////////////////////////////////////////
            //Step 12 [FIND_CORRECT_COLOR_BEACON_ONE]//
            ///////////////////////////////////////////

            case MOVE_CLOSER_TO_BEACON:

                if (range.getDistance(DistanceUnit.CM) == 16) { //Moves right in order to prepare for scan of colors for .8 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.FIND_CORRECT_COLOR_BEACON_ONE; //Sets next step to FIND_CORRECT_COLOR_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) < 16) {
                    m1.setPower(-.2); //Sets motor 1 to power -.2 to go backward because the robot is too close to the beacon
                    m2.setPower(-.2); //Sets motor 2 to power -.2 to go backward because the robot is too close to the beacon
                    m3.setPower(-.2); //Sets motor 3 to power -.2 to go backward because the robot is too close to the beacon
                    m4.setPower(-.2); //Sets motor 4 to power -.2 to go backward because the robot is too close to the beacon
                    break;
                }
                if (range.getDistance(DistanceUnit.CM) > 16) {
                    m1.setPower(.2); //Sets motor 1 to power .2 to go forward because the robot is too far from the beacon
                    m2.setPower(.2); //Sets motor 2 to power .2 to go forward because the robot is too far from the beacon
                    m3.setPower(.2); //Sets motor 3 to power .2 to go forward because the robot is too far from the beacon
                    m4.setPower(.2); //Sets motor 4 to power .2 to go forward because the robot is too far from the beacon
                    break;
                }
                break; //Exits switch statement

            case FIND_CORRECT_COLOR_BEACON_ONE:

                if (colorSensor.red() > 2) { //Moves right until red is found
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.PUSH_BUTTON_BEACON_ONE; //Sets next step to PUSH_BUTTON_BEACON_ONE
                    break; //Exits switch statement
                } else if (runtime.seconds() > 3) { //This is for backup only- if this step doesn't complete in 3 seconds, go into backup mode and run the steps
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.UNEXPECTED_NO_COLOR_BEACON_ONE; //Sets next step a backup step- UNEXPECTED_NO_COLOR_BEACON_ONE
                    break; //Exits switch statement
                } //End of else if statement
                if (gyro.getHeading() > 180) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(.1); //Sets motor 1 to power .2 to go right to scan for the correct color
                    m2.setPower(-.1); //Sets motor 2 to power -.2 to go right to scan for the correct color
                    m3.setPower(-.2); //Sets motor 3 to power -.1 to go right to scan for the correct color
                    m4.setPower(.2); //Sets motor 4 to power .1 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() < 180) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(.2); //Sets motor 1 to power .1 to go right to scan for the correct color
                    m2.setPower(-.2); //Sets motor 2 to power -.1 to go right to scan for the correct color
                    m3.setPower(-.1); //Sets motor 3 to power -.2 to go right to scan for the correct color
                    m4.setPower(.1); //Sets motor 4 to power .2 to go right to scan for the correct color
                    break; //Exits switch statement
                }
                if (gyro.getHeading() == 0) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .3
                    m1.setPower(.2); //Sets motor 1 to power .2 to go right to scan for the correct color
                    m2.setPower(-.2); //Sets motor 2 to power -.2 to go right to scan for the correct color
                    m3.setPower(-.2); //Sets motor 3 to power -.2 to go right to scan for the correct color
                    m4.setPower(.2); //Sets motor 4 to power .2 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                break; //Exits switch statement

            ////////////////////////////////////
            //Step 13 [PUSH_BUTTON_BEACON_ONE]//
            ////////////////////////////////////

            case PUSH_BUTTON_BEACON_ONE:

                if (runtime.seconds() > 2) { //Moves forward slowly to push button for 2 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.BACK_OFF_BEACON_ONE; //Sets next step to BACK_OFF_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.2); //Sets motor 1 to power .2 to go forward to push button
                m2.setPower(.2); //Sets motor 2 to power .2 to go forward to push button
                m3.setPower(.2); //Sets motor 3 to power .2 to go forward to push button
                m4.setPower(.2); //Sets motor 4 to power .2 to go forward to push button
                break; //Exits switch statement

            /////////////////////////////////
            //Step 14 [BACK_OFF_BEACON_ONE]//
            /////////////////////////////////

            case BACK_OFF_BEACON_ONE:

                if (runtime.seconds() > .5) { //Back away from beacon slowly for .5 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.NEXT_BEACON_STEP_ONE; //Sets next step to NEXT_BEACON_STEP_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.2); //Sets motor 1 to power -.2 to back away from beacon
                m2.setPower(-.2); //Sets motor 2 to power -.2 to back away from beacon
                m3.setPower(-.2); //Sets motor 3 to power -.2 to back away from beacon
                m4.setPower(-.2); //Sets motor 4 to power -.2 to back away from beacon
                break; //Exits switch statement

            //////////////////////////////////
            //Step 15 [NEXT_BEACON_STEP_ONE]//
            //////////////////////////////////

            case NEXT_BEACON_STEP_ONE:

                if (runtime.seconds() > 1) { //Moves right for 1 second to get ODS off beacon one white line
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.NEXT_BEACON_STEP_TWO; //Sets next step to NEXT_BEACON_STEP_TWO
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.2); //Sets motor 1 to power .2 to move right so OBS gets off beacon one white line
                m2.setPower(-.2); //Sets motor 2 to power -.2 to move right so OBS gets off beacon one white line
                m3.setPower(-.2); //Sets motor 3 to power -.2 to move right so OBS gets off beacon one white line
                m4.setPower(.2); //Sets motor 4 to power .2 to move right so OBS gets off beacon one white line
                break; //Exits switch statement

            //////////////////////////////////
            //Step 16 [NEXT_BEACON_STEP_TWO]//
            //////////////////////////////////

            case NEXT_BEACON_STEP_TWO:

                if (odsSensor1.getRawLightDetected() > .3) { //Moves right until next white line is found
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.DRIVE_OFF_WHITE_LINE_BEACON_TWO; //Sets next step to DRIVE_OFF_WHITE_LINE_BEACON_TWO
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() < 180) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(.1); //Sets motor 1 to power .2 to go right to scan for the correct color
                    m2.setPower(-.1); //Sets motor 2 to power -.2 to go right to scan for the correct color
                    m3.setPower(-.2); //Sets motor 3 to power -.1 to go right to scan for the correct color
                    m4.setPower(.2); //Sets motor 4 to power .1 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() > 180) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(.2); //Sets motor 1 to power .1 to go right to scan for the correct color
                    m2.setPower(-.2); //Sets motor 2 to power -.1 to go right to scan for the correct color
                    m3.setPower(-.1); //Sets motor 3 to power -.2 to go right to scan for the correct color
                    m4.setPower(.1); //Sets motor 4 to power .2 to go right to scan for the correct color
                    break; //Exits switch statement
                }
                if (gyro.getHeading() == 0) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .3
                    m1.setPower(.2); //Sets motor 1 to power .2 to go right to scan for the correct color
                    m2.setPower(-.2); //Sets motor 2 to power -.2 to go right to scan for the correct color
                    m3.setPower(-.2); //Sets motor 3 to power -.2 to go right to scan for the correct color
                    m4.setPower(.2); //Sets motor 4 to power .2 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                break; //Exits switch statement

            /////////////////////////////////////////////
            //Step 17 [DRIVE_OFF_WHITE_LINE_BEACON_TWO]//
            /////////////////////////////////////////////

            case DRIVE_OFF_WHITE_LINE_BEACON_TWO:

                if (runtime.seconds() > .8) { //Moves left in order to prepare for scan of colors for .8 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.MOVE_CLOSER_TO_BEACON_2; //Sets next step to FIND_CORRECT_COLOR_BEACON_TWO
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.3); //Sets motor 1 to power -.3 to move left to prepare for color scan
                m2.setPower(.3); //Sets motor 2 to power .3 to move left to prepare for color scan
                m3.setPower(.3); //Sets motor 3 to power .3 to move left to prepare for color scan
                m4.setPower(-.3); //Sets motor 4 to power -.3 to move left to prepare for color scan
                break; //Exits switch statement

            ///////////////////////////////////////////
            //Step 18 [FIND_CORRECT_COLOR_BEACON_TWO]//
            ///////////////////////////////////////////

            case MOVE_CLOSER_TO_BEACON_2:

                if (range.getDistance(DistanceUnit.CM) == 16) { //Moves right in order to prepare for scan of colors for .8 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.FIND_CORRECT_COLOR_BEACON_TWO; //Sets next step to FIND_CORRECT_COLOR_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) < 16) {
                    m1.setPower(-.2); //Sets motor 1 to power -.2 to go backward because the robot is too close to the beacon
                    m2.setPower(-.2); //Sets motor 2 to power -.2 to go backward because the robot is too close to the beacon
                    m3.setPower(-.2); //Sets motor 3 to power -.2 to go backward because the robot is too close to the beacon
                    m4.setPower(-.2); //Sets motor 4 to power -.2 to go backward because the robot is too close to the beacon
                    break;
                }
                if (range.getDistance(DistanceUnit.CM) > 16) {
                    m1.setPower(.2); //Sets motor 1 to power .2 to go forward because the robot is too far from the beacon
                    m2.setPower(.2); //Sets motor 2 to power .2 to go forward because the robot is too far from the beacon
                    m3.setPower(.2); //Sets motor 3 to power .2 to go forward because the robot is too far from the beacon
                    m4.setPower(.2); //Sets motor 4 to power .2 to go forward because the robot is too far from the beacon
                    break;
                }
                break; //Exits switch statement

            case FIND_CORRECT_COLOR_BEACON_TWO:

                if (colorSensor.red() > 2) { //Moves right until red is found
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.PUSH_BUTTON_BEACON_TWO; //Sets next step to PUSH_BUTTON_BEACON_TWO
                    break; //Exits switch statement
                } else if (runtime.seconds() > 3) { //This is for backup only- if this step doesn't complete in 3 seconds, go into backup mode and run the steps
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.UNEXPECTED_NO_COLOR_BEACON_TWO; //Sets next step a backup step- UNEXPECTED_NO_COLOR_BEACON_TWO
                    break; //Exits switch statement
                } //End of else if statement
                if (gyro.getHeading() < 180) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(.1); //Sets motor 1 to power .2 to go right to scan for the correct color
                    m2.setPower(-.1); //Sets motor 2 to power -.2 to go right to scan for the correct color
                    m3.setPower(-.2); //Sets motor 3 to power -.1 to go right to scan for the correct color
                    m4.setPower(.2); //Sets motor 4 to power .1 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() > 180) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(.2); //Sets motor 1 to power .1 to go right to scan for the correct color
                    m2.setPower(-.2); //Sets motor 2 to power -.1 to go right to scan for the correct color
                    m3.setPower(-.1); //Sets motor 3 to power -.2 to go right to scan for the correct color
                    m4.setPower(.1); //Sets motor 4 to power .2 to go right to scan for the correct color
                    break; //Exits switch statement
                }
                if (gyro.getHeading() == 0) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .3
                    m1.setPower(.2); //Sets motor 1 to power .2 to go right to scan for the correct color
                    m2.setPower(-.2); //Sets motor 2 to power -.2 to go right to scan for the correct color
                    m3.setPower(-.2); //Sets motor 3 to power -.2 to go right to scan for the correct color
                    m4.setPower(.2); //Sets motor 4 to power .2 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                break; //Exits switch statement

            /////////////////////////////////////
            //Step 19 [PUSH_BUTTON_BEACON_TWO:]//
            /////////////////////////////////////

            case PUSH_BUTTON_BEACON_TWO:

                if (runtime.seconds() > 2) { //Moves forward slowly to push button for 2 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.BACK_OFF_BEACON_TWO; //Sets next step to BACK_OFF_BEACON_TWO
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.2); //Sets motor 1 to power .2 to go forward to push button
                m2.setPower(.2); //Sets motor 2 to power .2 to go forward to push button
                m3.setPower(.2); //Sets motor 3 to power .2 to go forward to push button
                m4.setPower(.2); //Sets motor 4 to power .2 to go forward to push button
                break; //Exits switch statement

            /////////////////////////////////
            //Step 20 [BACK_OFF_BEACON_TWO]//
            /////////////////////////////////

            case BACK_OFF_BEACON_TWO:

                if (runtime.seconds() > 2) { //Back away from beacon
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.HIT_CAP_BALL; //Sets next step to HIT_CAP_BALL
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.2); //Sets motor 1 to power -.2 to back away from beacon
                m2.setPower(-.2); //Sets motor 2 to power -.2 to back away from beacon
                m3.setPower(-.2); //Sets motor 3 to power -.2 to back away from beacon
                m4.setPower(-.2); //Sets motor 4 to power -.2 to back away from beacon
                break; //Exits switch statement

            //////////////////////////
            //Step 21 [HIT_CAP_BALL]//
            //////////////////////////

            case HIT_CAP_BALL:

                if (runtime.seconds() > 2.5) { //Moves diagonally backwards towards the base of the center vortex, bumps the cap ball, and parks
                    CURRENT_STEP = steps.STOP; //Sets next step to STOP
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() > 180) { //If gyro senses a tilt, it lowers the speed of motor 4 to correct itself
                    m1.setPower(-.5); //Sets motor 1 to power -.5 to go backwards diagonally toward the center vortex and cap ball
                    m2.setPower(0); //Sets motor 2 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m3.setPower(0); //Sets motor 3 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m4.setPower(-1); //Sets motor 4 to power -1 to go backwards diagonally toward the center vortex and cap ball
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() < 180) { //If gyro senses a tilt, it lowers the speed of motor 1 to correct itself
                    m1.setPower(-1); //Sets motor 1 to power -1 to go backwards diagonally toward the center vortex and cap ball
                    m2.setPower(0); //Sets motor 2 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m3.setPower(0); //Sets motor 3 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m4.setPower(-.5); //Sets motor 4 to power -.5 to go backwards diagonally toward the center vortex and cap ball
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getHeading() == 0) { //If gyro senses no tilt, it will continue to move at full power with both motor 1 and 4
                    m1.setPower(-1); //Sets motor 1 to power -1 to go backwards diagonally toward the center vortex and cap ball
                    m2.setPower(0); //Sets motor 2 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m3.setPower(0); //Sets motor 3 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m4.setPower(-1); //Sets motor 4 to power -1 to go backwards diagonally toward the center vortex and cap ball
                    break; //Exits switch statement
                } //End of if statement

                ////////
                //STOP//
                ////////

            case STOP:

                m1.setPower(0); //Sets motor 1 power to 0 to make sure it is not moving
                m2.setPower(0); //Sets motor 2 power to 0 to make sure it is not moving
                m3.setPower(0); //Sets motor 3 power to 0 to make sure it is not moving
                m4.setPower(0); //Sets motor 4 power to 0 to make sure it is not moving
                m5.setPower(0); //Sets motor 5 power to 0 to make sure it is not moving
                m6.setPower(0); //Sets motor 6 power to 0 to make sure it is not moving
                m7.setPower(0); //Sets motor 7 power to 0 to make sure it is not moving
                m8.setPower(0); //Sets motor 8 power to 0 to make sure it is not moving
                break; //Exits switch statement

            ///////////////////////////////////////////////////
            //END OF MAIN STEPS AND BEGINNING OF BACKUP STEPS//
            ///////////////////////////////////////////////////

            ///////////////////////////////////////////
            //Backup [UNEXPECTED_NO_COLOR_BEACON_ONE]//
            ///////////////////////////////////////////

            case UNEXPECTED_NO_COLOR_BEACON_ONE: //Backup incase the correct color isn't found on beacon one within 3 seconds

                if (runtime.seconds() < .2) { //Move forward (toward beacon) for .2 seconds
                    m1.setPower(.5); //Sets motor 1 power to .5 to move toward beacon
                    m2.setPower(.5); //Sets motor 2 power to .5 to move toward beacon
                    m3.setPower(.5); //Sets motor 3 power to .5 to move toward beacon
                    m4.setPower(.5); //Sets motor 4 power to .5 to move toward beacon
                    break; //Exits switch statement
                } else if (runtime.seconds() > .2 && odsSensor1.getRawLightDetected() < .3) { //After moving towards the beacon, begin the search for the white line again
                    if (gyro.getHeading() > 180) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                        m1.setPower(-.2); //Sets motor 1 to power -.2 to go left and look for the beacon one white line
                        m2.setPower(.2); //Sets motor 2 to power .2 to go left and look for the beacon one white line
                        m3.setPower(.15); //Sets motor 3 to power .15 to go left and look for the beacon one white line
                        m4.setPower(-.15); //Sets motor 4 to power -.15 to go left and look for the beacon one white line
                        break; //Exits switch statement
                    } //End of if statement
                    if (gyro.getHeading() < 180) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                        m1.setPower(-.15); //Sets motor 1 to power -.15 to go left and look for the beacon one white line
                        m2.setPower(.15); //Sets motor 2 to power .15 to go left and look for the beacon one white line
                        m3.setPower(.2); //Sets motor 3 to power .2 to go left and look for the beacon one white line
                        m4.setPower(-.2); //Sets motor 4 to power -.2 to go left and look for the beacon one white line
                        break; //Exits switch statement
                    } //End of if statement
                    if (gyro.getHeading() == 0) { //If gyro senses no tilt, it continues to go left with all drive train motors set to a power of .2
                        m1.setPower(-.2); //Sets motor 1 to power -.2 to go left and look for the beacon one white line
                        m2.setPower(.2); //Sets motor 2 to power .2 to go left and look for the beacon one white line
                        m3.setPower(.2); //Sets motor 3 to power .2 to go left and look for the beacon one white line
                        m4.setPower(-.2); //Sets motor 4 to power -.2 to go left and look for the beacon one white line
                        break; //Exits switch statement
                    } //End of if statement
                    break; //Exits switch statement
                } else if (odsSensor1.getRawLightDetected() > .3) { //The white line has been found- go back and repeat step DRIVE_OFF_WHITE_LINE_BEACON_ONE
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.DRIVE_OFF_WHITE_LINE_BEACON_ONE; //Sets next step to DRIVE_OFF_WHITE_LINE_BEACON_ONE
                    break; //Exits switch statement
                } //End of else if statement

                ///////////////////////////////////////////
                //Backup [UNEXPECTED_NO_COLOR_BEACON_TWO]//
                ///////////////////////////////////////////

            case UNEXPECTED_NO_COLOR_BEACON_TWO: //Backup incase the correct color isn't found on beacon two within 3 seconds

                if (runtime.seconds() < .2) { //Move forward (toward beacon) for .2 seconds
                    m1.setPower(.5); //Sets motor 1 power to .5 to move toward beacon
                    m2.setPower(.5); //Sets motor 2 power to .5 to move toward beacon
                    m3.setPower(.5); //Sets motor 3 power to .5 to move toward beacon
                    m4.setPower(.5); //Sets motor 4 power to .5 to move toward beacon
                    break; //Exits switch statement
                } else if (runtime.seconds() > .2 && odsSensor1.getRawLightDetected() < .3) { //After moving towards the beacon, begin the search for the white line again
                    if (gyro.getHeading() > 200) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                        m1.setPower(.2); //Sets motor 1 to power -.2 to go left and look for the beacon two white line
                        m2.setPower(-.2); //Sets motor 2 to power .2 to go left and look for the beacon two white line
                        m3.setPower(-.15); //Sets motor 3 to power .15 to go left and look for the beacon two white line
                        m4.setPower(.15); //Sets motor 4 to power -.15 to go left and look for the beacon two white line
                        break; //Exits switch statement
                    } else if (runtime.seconds() > .2 && colorSensor.red() > 1.5) { //Moves left until blue is found
                        m1.setPower(0); //Sets motor 1 to power 0 before next step
                        m2.setPower(0); //Sets motor 2 to power 0 before next step
                        m3.setPower(0); //Sets motor 3 to power 0 before next step
                        m4.setPower(0); //Sets motor 4 to power 0 before next step
                        runtime.reset(); //Resets time before switching to next step
                        CURRENT_STEP = steps.PUSH_BUTTON_BEACON_TWO; //Sets next step to PUSH_BUTTON_BEACON_ONE
                        break; //Exits switch statement
                    } else if (gyro.getHeading() > 200) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                        m1.setPower(-.35); //Sets motor 1 to power -.2 to go left to scan for the correct color
                        m2.setPower(.35); //Sets motor 2 to power .2 to go left to scan for the correct color
                        m3.setPower(.25); //Sets motor 3 to power .1 to go left to scan for the correct color
                        m4.setPower(-.25); //Sets motor 4 to power -.1 to go left to scan for the correct color
                        break; //Exits switch statement
                    } //End of if statement
                    else if (gyro.getHeading() > 0 && gyro.getHeading() < 200) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                        m1.setPower(-.25); //Sets motor 1 to power -.1 to go left to scan for the correct color
                        m2.setPower(.25); //Sets motor 2 to power .1 to go left to scan for the correct color
                        m3.setPower(.35); //Sets motor 3 to power .2 to go left to scan for the correct color
                        m4.setPower(-.35); //Sets motor 4 to power -.2 to go left to scan for the correct color
                        break; //Exits switch statement
                    } else if (gyro.getHeading() == 0) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .3
                        m1.setPower(-.25); //Sets motor 1 to power -.2 to go left to scan for the correct color
                        m2.setPower(.25); //Sets motor 2 to power .2 to go left to scan for the correct color
                        m3.setPower(.25); //Sets motor 3 to power .2 to go left to scan for the correct color
                        m4.setPower(-.25); //Sets motor 4 to power -.2 to go left to scan for the correct color
                        break; //Exits switch statement
                    } //End of if statement
                } //End of switch statement
        } //End of loop
    } //End of program

}