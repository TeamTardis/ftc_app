package org.firstinspires.ftc.teamcode; //Use the package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor; //Import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor
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
//TEST GIT
/**
 * Created by Corning Robotics on 12/21/16. *
 **/

/**
 * ////////////////////////////////////////////////////////////////////////////////////////////////////
 * //Definitions (just to be sure what you're thinking and what we're talking about are the same :) )//
 * ////////////////////////////////////////////////////////////////////////////////////////////////////
 * <p>
 * Backup - An emergency step or precaution added for a double check or to allow for a second chance at completing a task.
 * Backward - The opposite direction of forward (see definition "forward").
 * Beacon one - The beacon closest to the corner vortex is beacon one.
 * Beacon two - The beacon farthest from the corner vortex is beacon two.
 * Beacon wall - The wall where two corresponding beacons are placed.
 * Button pusher - The 3D printed piece of spiked plastic used to push beacon buttons.
 * Drive train motors - The motors that have contact with the surface of the mat and help the robot move (motor 1, motor 2, motor 3, and motor 4).
 * Forward - The side of the robot where the button pusher (see definition "button pusher") is placed. For example, if the button pusher was facing toward the starting wall and the robot moved forward, the robot would get closer to the wall.
 * Launcher motors - The two motors (motor 5 and motor 6) that speed up and allow the ball to be shot.
 * Launching position - The position the launching servo (see definition "launching servo") must be in so the ball makes contact with the launcher (see definition "launcher").
 * Launching servo - The servo (servo 2) that lifts the ball from the teleporter (see definition "teleporter") into the launcher (see definition "launcher").
 * Left - The left side of the robot when facing the same way as the button pusher (see definition "button pusher").
 * ODS - The abbreviation for the optical distance sensor.
 * Original position (in the case of the launching servo) - Servo position .05. This position allows for the next ball to roll into the launching servo (see definition "launching servo").
 * Right - The right side of the robot when facing the same way as the button pusher (see definition "button pusher").
 * Launcher - The combination of both launcher motors (see definition "launcher motors").
 * Starting wall - The wall that the robot initially starts on.
 * Sweeper - The ball collector that also kicks balls into the teleporter (see definition "teleporter").
 * Teleporter - The name we use to describe the tube between our sweeperOn (see definition "sweeperOn") and our launching mechanism (see definition "launching servo").
 * White line - The white tape on the floor perpendicular to a beacon.
 **/

@Autonomous(name = "BLUE", group = "AutoFast") //Display name and group found in on controller phone
public class Blue_NEW_FULL extends TardisOpModeAutonomous { //Imports presets for initiation from TardisOpModeAutonomous

    public enum steps { //All steps for completing autonomous

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Main steps - These steps run in sequence when the program is gong perfect with no unexpected readings in sensor inputs.//
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        START_RESET,
        MOVE_TOWARDS_BEACON,
        MOVE_AWAY_FROM_WALL, //[Step 1] Moves away from starting wall to prepare for rotation
        ROTATE_TO_VORTEX, //[Step 2] Rotate about 50° counter-clockwise to face launcher at center vortex
        SHOOT_FIRST_BALL, //[Step 4] Shoot first ball by lifting ball using launching servo
        LOWER_SERVO_1, //[Step 5] Lowers launching servo (resets it to original position)
        WAIT,
        MOTORS_OFF,
        MOTORS_ON,
        SHOOT_SECOND_BALL, //[Step 6] Shoot second ball by lifting ball using launching servo
        ROTATE_BACK, //[Step 8] Rotates back 50° clockwise to face button pusher towards beacon- this step also sets the launcher's motors to power 0 for power conservation
        FORWARD,
        STRAIGHTEN_ON_WALL, //[Step 9] Pushes against starting wall as backup
        DRIVE_TO_FIRST_BEACON, //[Step 10] Drive diagonally using motor 2, motor 3, and gyro sensor until the beacon one white line is detected
        DRIVE_TO_FIRST_BEACON_TWO, //[Step 10] Drive diagonally using motor 2, motor 3, and gyro sensor until the beacon one white line is detected
        FIND_WHITE_LINE_BEACON_ONE, //[Step 11] Move off white line to right side of the beacon to prepare for scanning of correct color
        MOVE_CLOSER_TO_BEACON_ONE, //[Step 12] Use the range sensor to move the correct distance away from the wall
        FIND_CORRECT_COLOR_BEACON_ONE, //[Step 13] Drive right to locate and stop at correct color while scanning beacon
        PUSH_BUTTON_BEACON_ONE, //[Step 14] Once correct color is found, drive forward to select color
        BACK_BEACON_ONE,
        SECOND_PUSH_BEACON_ONE,
        BACK_OFF_BEACON_ONE, //[Step 15] After button press, drive backward
        NEXT_BEACON_STEP_ONE, //[Step 16] Moves right to get off white line to prepare to look for the beacon two white line
        NEXT_BEACON_STEP_TWO, //[Step 17] Using gyro for stabilization, begin to start scan for the beacon two white line
        DRIVE_OFF_WHITE_LINE_BEACON_TWO, //[Step 18] Move off white line to left side of the beacon to prepare to scan for correct color
        MOVE_CLOSER_TO_BEACON_TWO, //[Step 19] Use the range sensor to move the correct distance away from the wall
        FIND_CORRECT_COLOR_BEACON_TWO, //[Step 20] Drive right to locate and stop at correct color while scanning beacon
        PUSH_BUTTON_BEACON_TWO, //[Step 21] Once correct color is found, drive forward to select color
        BACK_BEACON_TWO,
        SECOND_PUSH_BEACON_TWO,
        BACK_OFF_BEACON_TWO, //[Step 22] After button press, drive backward
        HIT_CAP_BALL, //[Step 23] Drive diagonally backward using motor 1, motor 4 and gyro sensor to hit cap ball off the base of the center vortex and park there
        ROTATE_CAP_BALL,
        STOP, //The title says it all

    } //End of steps for autonomous

    public steps CURRENT_STEP = steps.START_RESET; //Sets the variable CURRENT_STEP to the first step in the sequence

    private ElapsedTime runtime = new ElapsedTime(); //Creates a variable for runtime so we can have timed events

    @Override //Method overrides parent class
    public void loop() { //Starts loop for the program

        ///////////////////////////
        //Telemetry for debugging//
        ///////////////////////////

        telemetry.addData("", "Range (CM): " + range.getDistance(DistanceUnit.CM) + "\nzValue: " + gyro.getIntegratedZValue() + "\nRed: " + colorSensor.blue() + "\nRuntime variable: " + runtime + "\nStep: " + CURRENT_STEP); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new information

        /////////////////////////////
        //Start of switch statement//
        /////////////////////////////

        switch (CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever CURRENT_STEP is set to

            ///////////////////////
            //START OF MAIN STEPS//
            ///////////////////////

            case START_RESET:

                runtime.reset();
                s3.setPosition(0.32); //Forks pointed up
                m5.setMaxSpeed(1600);  //Set max speed medium for shooter
                m6.setMaxSpeed(1600);
                CURRENT_STEP = steps.MOVE_TOWARDS_BEACON; //Sets next step to MOVE_AWAY_FROM_WALL
                break;

            ////////////////////////////////
            //Step 1 [MOVE_AWAY_FROM_WALL]//
            ////////////////////////////////

            case MOVE_TOWARDS_BEACON: //Beginning of case statement MOVE_AWAY_FROM_WALL

                if (range.getDistance(DistanceUnit.CM) > 70 && range.getDistance(DistanceUnit.CM) < 80 && runtime.seconds() > 1) { //Moves diagonally towards first beacon and stops when ODS scenes a white line
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.ROTATE_TO_VORTEX; //Sets next step to FIND_WHITE_LINE_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > 0) { //If gyro senses a tilt, it lowers the speed of motor 1 to correct itself
                    m1.setPower(-.5); //Sets motor 1 to power .8 to go diagonally toward beacon one
                    m2.setPower(0); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(0); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(-1); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    m5.setPower(1);
                    m6.setPower(1);
                    m7.setPower(-1);
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() < 0) { //If gyro senses a tilt, it lowers the speed of motor 4 to correct itself
                    m1.setPower(-1); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(0); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(0); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(-.5); //Sets motor 4 to power .8 to go diagonally toward beacon one
                    m5.setPower(1);
                    m6.setPower(1);
                    m7.setPower(-1);
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == 0) { //If gyro senses no tilt, it will continue to move at full power with both motors 1 and 4
                    m1.setPower(-1); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(0); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(0); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(-1); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    m5.setPower(1);
                    m6.setPower(1);
                    m7.setPower(-1);
                    break; //Exits switch statement
                } //End of if statement

            case ROTATE_TO_VORTEX:

                if (gyro.getIntegratedZValue() < -60) { //Rotates robot about 20° clockwise so the launcher faces the center vortex
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SHOOT_FIRST_BALL; //Sets next step to SPEED_UP_SHOOTERS
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.2); //Sets motor 1 to power .1 to rotate the robot clockwise
                m2.setPower(-.2); //Sets motor 2 to power -.1 to rotate the robot clockwise
                m3.setPower(.2); //Sets motor 3 to power .1 to rotate the robot clockwise
                m4.setPower(-.2); //Sets motor 4 to power -.1 to rotate the robot clockwise
                break; //Exits switch statement

            case SHOOT_FIRST_BALL: //Beginning of case statement SHOOT_FIRST_BALL

                if (runtime.seconds() > .3) { //Raises ball into launcher by setting servo position to launching position for 1 second
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    m5.setPower(1); //Set launcher motor 5 to .1 for prep
                    m6.setPower(1); //Set launcher motor 6 to .1 for prep
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
                    m5.setPower(1); //Set launcher motor 5 to .1 for prep
                    m6.setPower(1); //Set launcher motor 6 to .1 for prep
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SHOOT_SECOND_BALL; //Sets next step to SHOOT_SECOND_BALL
                    break; //Exits switch statement
                } //End of if statement
                s2.setPosition(.05); //Sets launching servo to original position
                m7.setPower(-1);
                break; //Exits switch statement

            //////////////////////////////
            //Step 6 [SHOOT_SECOND_BALL]//
            //////////////////////////////

            case SHOOT_SECOND_BALL: //Beginning of case statement SHOOT_SECOND_BALL

                if (runtime.seconds() > .6) { //Raises ball into launcher by setting servo position to launching position for 1 second
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    m5.setPower(1); //Set launcher motor 5 to .1 for prep
                    m6.setPower(1); //Set launcher motor 6 to .1 for prep
                    m7.setPower(-1);
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.ROTATE_BACK; //Sets next step to LOWER_SERVO_2
                    break; //Exits switch statement
                } //End of if statement
                s2.setPosition(1); //Sets launching servo to launching position
                break; //Exits switch statement

            case ROTATE_BACK: //Beginning of case statement ROTATE_BACK

                if (gyro.getIntegratedZValue() < -250) { //Rotates robot about 20° clockwise so the launcher faces the center vortex
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.DRIVE_TO_FIRST_BEACON_TWO; //Sets next step to SPEED_UP_SHOOTERS
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.2); //Sets motor 1 to power .1 to rotate the robot clockwise
                m2.setPower(-.2); //Sets motor 2 to power -.1 to rotate the robot clockwise
                m3.setPower(.2); //Sets motor 3 to power .1 to rotate the robot clockwise
                m4.setPower(-.2); //Sets motor 4 to power -.1 to rotate the robot clockwise
                m5.setPower(0);
                m6.setPower(0);
                m7.setPower(0);
                s2.setPosition(.05); //Sets launching servo to original position
                break; //Exits switch statement

            case DRIVE_TO_FIRST_BEACON_TWO: //Beginning of case statement DRIVE_TO_FIRST_BEACON

                if (odsSensor1.getRawLightDetected() > .5) { //Moves diagonally towards first beacon and stops when ODS scenes a white line
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.FORWARD; //Sets next step to FIND_WHITE_LINE_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() < -270) { //If gyro senses a tilt, it lowers the speed of motor 1 to correct itself
                    m1.setPower(0); //Sets motor 1 to power .8 to go diagonally toward beacon one
                    m2.setPower(.5); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(.2); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(0); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > -270) { //If gyro senses a tilt, it lowers the speed of motor 4 to correct itself
                    m1.setPower(0); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(.2); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(.5); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(0); //Sets motor 4 to power .8 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == -270) { //If gyro senses no tilt, it will continue to move at full power with both motors 1 and 4
                    m1.setPower(0); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(.5); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(.5); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(0); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement

            case FORWARD:

                if (range.getDistance(DistanceUnit.CM) <= 25) { //Move the robot closer or farther from the beacon wall to get correct distance to sense colors
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.MOVE_CLOSER_TO_BEACON_ONE; //Sets next step to FIND_CORRECT_COLOR_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) > 25) { //If the robot is to close to the beacon, back away
                    m1.setPower(.3);
                    m2.setPower(.3);
                    m3.setPower(.3);
                    m4.setPower(.3);
                    break;
                } //End of if statement
//                if (range.getDistance(DistanceUnit.CM) < 16) { //If the robot is to far away from the beacon, move closer
//                    m1.setPower(-.3);
//                    m2.setPower(-.3);
//                    m3.setPower(-.3);
//                    m4.setPower(-.3);
//                    break;
//                } //End of if statement
                break; //Exits switch statement

            case MOVE_CLOSER_TO_BEACON_ONE: //Beginning of case statement MOVE_CLOSER_TO_BEACON_ONE

                if (range.getDistance(DistanceUnit.CM) <= 14) { //Moves diagonally towards first beacon and stops when ODS scenes a white line
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.FIND_CORRECT_COLOR_BEACON_ONE; //Sets next step to FIND_WHITE_LINE_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > -270) { //If gyro senses a tilt, it lowers the speed of motor 1 to correct itself
                    m1.setPower(0); //Sets motor 1 to power .8 to go diagonally toward beacon one
                    m2.setPower(.3); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(.5); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(0); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() < -270) { //If gyro senses a tilt, it lowers the speed of motor 4 to correct itself
                    m1.setPower(0); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(.5); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(.3); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(0); //Sets motor 4 to power .8 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == -270) { //If gyro senses no tilt, it will continue to move at full power with both motors 1 and 4
                    m1.setPower(0); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(.5); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(.5); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(0); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement

                break; //Exits switch statement

            case FIND_CORRECT_COLOR_BEACON_ONE: //Beginning of case statement FIND_CORRECT_COLOR_BEACON_ONE

                if (colorSensor.blue() > 200) { //Moves right until red is found
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.PUSH_BUTTON_BEACON_ONE; //Sets next step to PUSH_BUTTON_BEACON_ONE
                    break; //Exits switch statement
                }
                if (gyro.getIntegratedZValue() < -270 && range.getDistance(DistanceUnit.CM) == 10) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(.2); //Sets motor 1 to power .3 to go right to scan for the correct color
                    m2.setPower(-.2); //Sets motor 2 to power -.3 to go right to scan for the correct color
                    m3.setPower(-.3); //Sets motor 3 to power -.4 to go right to scan for the correct color
                    m4.setPower(.3); //Sets motor 4 to power .4 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > -270 && range.getDistance(DistanceUnit.CM) == 10) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(.3); //Sets motor 1 to power .4 to go right to scan for the correct color
                    m2.setPower(-.3); //Sets motor 2 to power -.4 to go right to scan for the correct color
                    m3.setPower(-.2); //Sets motor 3 to power -.3 to go right to scan for the correct color
                    m4.setPower(.2); //Sets motor 4 to power .3 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == -270 && range.getDistance(DistanceUnit.CM) == 10) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .3
                    m1.setPower(.3); //Sets motor 1 to power .4 to go right to scan for the correct color
                    m2.setPower(-.3); //Sets motor 2 to power -.4 to go right to scan for the correct color
                    m3.setPower(-.3); //Sets motor 3 to power -.4 to go right to scan for the correct color
                    m4.setPower(.3); //Sets motor 4 to power .4 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) > 10) { //If the robot is to close to the beacon, back away
                    m1.setPower(.3);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(.3);
                    break;
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) < 10) { //If the robot is to close to the beacon, back away
                    m1.setPower(0);
                    m2.setPower(-.3);
                    m3.setPower(-.3);
                    m4.setPower(0);
                    break;
                } //End of if statement

                ////////////////////////////////////
                //Step 14 [PUSH_BUTTON_BEACON_ONE]//
                ////////////////////////////////////

            case PUSH_BUTTON_BEACON_ONE: //Beginning of case statement PUSH_BUTTON_BEACON_ONE

                if (runtime.seconds() > .4) { //Moves forward to push button for 1 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.BACK_BEACON_ONE; //Sets next step to BACK_OFF_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.6); //Sets motor 1 to power .4 to go forward to push button
                m2.setPower(.6); //Sets motor 2 to power .4 to go forward to push button
                m3.setPower(.6); //Sets motor 3 to power .4 to go forward to push button
                m4.setPower(.6); //Sets motor 4 to power .4 to go forward to push button
                break; //Exits switch statement


            case BACK_BEACON_ONE: //Beginning of case statement PUSH_BUTTON_BEACON_ONE

                if (runtime.seconds() > .2) { //Moves forward to push button for 1 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SECOND_PUSH_BEACON_ONE; //Sets next step to BACK_OFF_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.4); //Sets motor 1 to power .4 to go forward to push button
                m2.setPower(-.4); //Sets motor 2 to power .4 to go forward to push button
                m3.setPower(-.4); //Sets motor 3 to power .4 to go forward to push button
                m4.setPower(-.4); //Sets motor 4 to power .4 to go forward to push button
                break; //Exits switch statement


            case SECOND_PUSH_BEACON_ONE: //Beginning of case statement PUSH_BUTTON_BEACON_ONE

                if (runtime.seconds() > .4) { //Moves forward to push button for 1 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.BACK_OFF_BEACON_ONE; //Sets next step to BACK_OFF_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.4); //Sets motor 1 to power .4 to go forward to push button
                m2.setPower(0); //Sets motor 2 to power .4 to go forward to push button
                m3.setPower(0); //Sets motor 3 to power .4 to go forward to push button
                m4.setPower(.4); //Sets motor 4 to power .4 to go forward to push button
                break; //Exits switch statement


            /////////////////////////////////
            //Step 15 [BACK_OFF_BEACON_ONE]//
            /////////////////////////////////

            case BACK_OFF_BEACON_ONE: //Beginning of case statement BACK_OFF_BEACON_ONE

                if (runtime.seconds() > .7) { //Back away from beacon slowly for .5 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.NEXT_BEACON_STEP_ONE; //Sets next step to NEXT_BEACON_STEP_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.5); //Sets motor 1 to power -.3 to back away from beacon
                m2.setPower(0); //Sets motor 2 to power -.3 to back away from beacon
                m3.setPower(0); //Sets motor 3 to power -.3 to back away from beacon
                m4.setPower(-.5); //Sets motor 4 to power -.3 to back away from beacon
                break; //Exits switch statement


            case NEXT_BEACON_STEP_ONE: //Beginning of case statement NEXT_BEACON_STEP_ONE

                if (runtime.seconds() > 1.8) { //Moves right for 1 second to get ODS off beacon one white line
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.NEXT_BEACON_STEP_TWO; //Sets next step to NEXT_BEACON_STEP_TWO
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() < -270 && range.getDistance(DistanceUnit.CM) > 16 && range.getDistance(DistanceUnit.CM) < 20) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(-.7); //Sets motor 1 to power .25 to go right and look for the beacon two white line
                    m2.setPower(.7); //Sets motor 2 to power -.25 to go right and look for the beacon two white line
                    m3.setPower(.5); //Sets motor 3 to power -.35 to go right and look for the beacon two white line
                    m4.setPower(-.5); //Sets motor 4 to power .35 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > -270 && range.getDistance(DistanceUnit.CM) > 16 && range.getDistance(DistanceUnit.CM) < 20) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(-.5); //Sets motor 1 to power .35 to go right and look for the beacon two white line
                    m2.setPower(.5); //Sets motor 2 to power -.35 to go right and look for the beacon two white line
                    m3.setPower(.7); //Sets motor 3 to power -.25 to go right and look for the beacon two white line
                    m4.setPower(-.7); //Sets motor 4 to power .25 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == -270 && range.getDistance(DistanceUnit.CM) > 16 && range.getDistance(DistanceUnit.CM) < 20) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .35
                    m1.setPower(-.7); //Sets motor 1 to power .35 to go right and look for the beacon two white line
                    m2.setPower(.7); //Sets motor 2 to power -.35 to go right and look for the beacon two white line
                    m3.setPower(.7); //Sets motor 3 to power -.35 to go right and look for the beacon two white line
                    m4.setPower(-.7); //Sets motor 4 to power .35 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement.
                if (range.getDistance(DistanceUnit.CM) < 16) { //If the robot is to close to the beacon, back away
                    m1.setPower(-.7);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(-.7);
                    break;
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) > 20) { //If the robot is to close to the beacon, back away
                    m1.setPower(0);
                    m2.setPower(.7);
                    m3.setPower(.7);
                    m4.setPower(0);
                    break;
                } //End of if statement
                break; //Exits switch statement

            //////////////////////////////////
            //Step 17 [NEXT_BEACON_STEP_TWO]//
            //////////////////////////////////

            case NEXT_BEACON_STEP_TWO: //Beginning of case statement NEXT_BEACON_STEP_TWO

                if (odsSensor1.getRawLightDetected() > .5) { //Moves right until next white line is found
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.MOVE_CLOSER_TO_BEACON_TWO; //Sets next step to DRIVE_OFF_WHITE_LINE_BEACON_TWO
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() < -270 && range.getDistance(DistanceUnit.CM) > 16 && range.getDistance(DistanceUnit.CM) < 20) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(-.3); //Sets motor 1 to power .25 to go right and look for the beacon two white line
                    m2.setPower(.3); //Sets motor 2 to power -.25 to go right and look for the beacon two white line
                    m3.setPower(.2); //Sets motor 3 to power -.35 to go right and look for the beacon two white line
                    m4.setPower(-.2); //Sets motor 4 to power .35 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > -270 && range.getDistance(DistanceUnit.CM) > 16 && range.getDistance(DistanceUnit.CM) < 20) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(-.2); //Sets motor 1 to power .35 to go right and look for the beacon two white line
                    m2.setPower(.2); //Sets motor 2 to power -.35 to go right and look for the beacon two white line
                    m3.setPower(.3); //Sets motor 3 to power -.25 to go right and look for the beacon two white line
                    m4.setPower(-.3); //Sets motor 4 to power .25 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == -270 && range.getDistance(DistanceUnit.CM) > 16 && range.getDistance(DistanceUnit.CM) < 20) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .35
                    m1.setPower(-.3); //Sets motor 1 to power .35 to go right and look for the beacon two white line
                    m2.setPower(.3); //Sets motor 2 to power -.35 to go right and look for the beacon two white line
                    m3.setPower(.3); //Sets motor 3 to power -.35 to go right and look for the beacon two white line
                    m4.setPower(-.3); //Sets motor 4 to power .35 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement.
                if (range.getDistance(DistanceUnit.CM) < 16) { //If the robot is to close to the beacon, back away
                    m1.setPower(-.3);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(-.3);
                    break;
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) > 20) { //If the robot is to close to the beacon, back away
                    m1.setPower(0);
                    m2.setPower(.3);
                    m3.setPower(.3);
                    m4.setPower(0);
                    break;
                } //End of if statement
                break; //Exits switch statement


            case MOVE_CLOSER_TO_BEACON_TWO: //Beginning of case statement MOVE_CLOSER_TO_BEACON_ONE

                if (range.getDistance(DistanceUnit.CM) <= 14) { //Moves diagonally towards first beacon and stops when ODS scenes a white line
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.FIND_CORRECT_COLOR_BEACON_TWO; //Sets next step to FIND_WHITE_LINE_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > -270) { //If gyro senses a tilt, it lowers the speed of motor 1 to correct itself
                    m1.setPower(0); //Sets motor 1 to power .8 to go diagonally toward beacon one
                    m2.setPower(.3); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(.5); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(0); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() < -270) { //If gyro senses a tilt, it lowers the speed of motor 4 to correct itself
                    m1.setPower(0); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(.5); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(.3); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(0); //Sets motor 4 to power .8 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == -270) { //If gyro senses no tilt, it will continue to move at full power with both motors 1 and 4
                    m1.setPower(0); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(.5); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(.5); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(0); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement

                break; //Exits switch statement

            case FIND_CORRECT_COLOR_BEACON_TWO: //Beginning of case statement FIND_CORRECT_COLOR_BEACON_ONE

                if (colorSensor.blue() > 200) { //Moves right until red is found
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.PUSH_BUTTON_BEACON_TWO; //Sets next step to PUSH_BUTTON_BEACON_ONE
                    break; //Exits switch statement
                }
                if (gyro.getIntegratedZValue() < -270 && range.getDistance(DistanceUnit.CM) == 10) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(.2); //Sets motor 1 to power .3 to go right to scan for the correct color
                    m2.setPower(-.2); //Sets motor 2 to power -.3 to go right to scan for the correct color
                    m3.setPower(-.3); //Sets motor 3 to power -.4 to go right to scan for the correct color
                    m4.setPower(.3); //Sets motor 4 to power .4 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > -270 && range.getDistance(DistanceUnit.CM) == 10) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(.3); //Sets motor 1 to power .4 to go right to scan for the correct color
                    m2.setPower(-.3); //Sets motor 2 to power -.4 to go right to scan for the correct color
                    m3.setPower(-.2); //Sets motor 3 to power -.3 to go right to scan for the correct color
                    m4.setPower(.2); //Sets motor 4 to power .3 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == -270 && range.getDistance(DistanceUnit.CM) == 10) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .3
                    m1.setPower(.3); //Sets motor 1 to power .4 to go right to scan for the correct color
                    m2.setPower(-.3); //Sets motor 2 to power -.4 to go right to scan for the correct color
                    m3.setPower(-.3); //Sets motor 3 to power -.4 to go right to scan for the correct color
                    m4.setPower(.3); //Sets motor 4 to power .4 to go right to scan for the correct color
                    break; //Exits switch statement
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) > 10) { //If the robot is to close to the beacon, back away
                    m1.setPower(.3);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(.3);
                    break;
                } //End of if statement
                if (range.getDistance(DistanceUnit.CM) < 10) { //If the robot is to close to the beacon, back away
                    m1.setPower(0);
                    m2.setPower(-.3);
                    m3.setPower(-.3);
                    m4.setPower(0);
                    break;
                } //End of if statement

                ////////////////////////////////////
                //Step 14 [PUSH_BUTTON_BEACON_ONE]//
                ////////////////////////////////////

            case PUSH_BUTTON_BEACON_TWO: //Beginning of case statement PUSH_BUTTON_BEACON_ONE

                if (runtime.seconds() > .4) { //Moves forward to push button for 1 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.BACK_BEACON_TWO; //Sets next step to BACK_OFF_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.6); //Sets motor 1 to power .4 to go forward to push button
                m2.setPower(.6); //Sets motor 2 to power .4 to go forward to push button
                m3.setPower(.6); //Sets motor 3 to power .4 to go forward to push button
                m4.setPower(.6); //Sets motor 4 to power .4 to go forward to push button
                break; //Exits switch statement

            case BACK_BEACON_TWO: //Beginning of case statement PUSH_BUTTON_BEACON_ONE

                if (runtime.seconds() > .2) { //Moves forward to push button for 1 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SECOND_PUSH_BEACON_TWO; //Sets next step to BACK_OFF_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.4); //Sets motor 1 to power .4 to go forward to push button
                m2.setPower(-.4); //Sets motor 2 to power .4 to go forward to push button
                m3.setPower(-.4); //Sets motor 3 to power .4 to go forward to push button
                m4.setPower(-.4); //Sets motor 4 to power .4 to go forward to push button
                break; //Exits switch statement

            case SECOND_PUSH_BEACON_TWO: //Beginning of case statement PUSH_BUTTON_BEACON_ONE

                if (runtime.seconds() > .4) { //Moves forward to push button for 1 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.BACK_OFF_BEACON_TWO; //Sets next step to BACK_OFF_BEACON_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(.4); //Sets motor 1 to power .4 to go forward to push button
                m2.setPower(0); //Sets motor 2 to power .4 to go forward to push button
                m3.setPower(0); //Sets motor 3 to power .4 to go forward to push button
                m4.setPower(.4); //Sets motor 4 to power .4 to go forward to push button
                break; //Exits switch statement

            /////////////////////////////////
            //Step 15 [BACK_OFF_BEACON_ONE]//
            /////////////////////////////////

            case BACK_OFF_BEACON_TWO: //Beginning of case statement BACK_OFF_BEACON_ONE

                if (runtime.seconds() > .7) { //Back away from beacon slowly for .5 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.HIT_CAP_BALL; //Sets next step to NEXT_BEACON_STEP_ONE
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-.5); //Sets motor 1 to power -.3 to back away from beacon
                m2.setPower(-.5); //Sets motor 2 to power -.3 to back away from beacon
                m3.setPower(-.5); //Sets motor 3 to power -.3 to back away from beacon
                m4.setPower(-.5); //Sets motor 4 to power -.3 to back away from beacon
                break; //Exits switch statement

            case HIT_CAP_BALL: //Beginning of case statement HIT_CAP_BALL

                if (runtime.seconds() > 2.7) { //Moves diagonally backwards towards the base of the center vortex for 2.5 seconds, bumps the cap ball, and parks
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset();
                    CURRENT_STEP = steps.ROTATE_CAP_BALL; //Sets next step to STOP
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() < -270) { //If gyro senses a tilt, it lowers the speed of motor 1 to correct itself
                    m1.setPower(0); //Sets motor 1 to power -.5 to go backwards diagonally toward the center vortex and cap ball
                    m2.setPower(-.8); //Sets motor 2 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m3.setPower(-1); //Sets motor 3 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m4.setPower(0); //Sets motor 4 to power -1 to go backwards diagonally toward the center vortex and cap ball
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > -270) { //If gyro senses a tilt, it lowers the speed of motor 4 to correct itself
                    m1.setPower(0); //Sets motor 1 to power -1 to go backwards diagonally toward the center vortex and cap ball
                    m2.setPower(-1); //Sets motor 2 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m3.setPower(-.8); //Sets motor 3 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m4.setPower(0); //Sets motor 4 to power -.5 to go backwards diagonally toward the center vortex and cap ball
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == -270) { //If gyro senses no tilt, it will continue to move at full power with both motor 1 and 4
                    m1.setPower(0); //Sets motor 1 to power -1 to go backwards diagonally toward the center vortex and cap ball
                    m2.setPower(-1); //Sets motor 2 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m3.setPower(-1); //Sets motor 3 to power 0 to go backwards diagonally toward the center vortex and cap ball
                    m4.setPower(0); //Sets motor 4 to power -1 to go backwards diagonally toward the center vortex and cap ball
                    break; //Exits switch statement
                } //End of if statement

            case ROTATE_CAP_BALL:

                if (runtime.seconds() > 1) { //Moves diagonally backwards towards the base of the center vortex for 2.5 seconds, bumps the cap ball, and parks
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset();
                    CURRENT_STEP = steps.STOP; //Sets next step to STOP
                    break; //Exits switch statement
                } //End of if statement
                m1.setPower(-1);
                m2.setPower(1);
                m3.setPower(-1);
                m4.setPower(1);

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
        }
    }
}