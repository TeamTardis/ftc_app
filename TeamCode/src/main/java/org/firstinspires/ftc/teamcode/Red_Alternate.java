package org.firstinspires.ftc.teamcode; //Use the package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//TEST GIT
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

@Autonomous(name = "RED_ALT", group = "AutoFast") //Display name and group found in on controller phone
public class Red_Alternate extends TardisOpModeAutonomous { //Imports presets for initiation from TardisOpModeAutonomous

    public enum steps { //All steps for completing autonomous

        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        //Main steps - These steps run in sequence when the program is gong perfect with no unexpected readings in sensor inputs.//
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        START_RESET,
        SPEED_UP_SHOOTERS,
        SHOOT_FIRST_BALL,
        LOWER_SERVO_1, //[Step 5] Lowers launching servo (resets it to original position)
        SHOOT_SECOND_BALL, //[Step 6] Shoot second ball by lifting ball using launching servo
        DRIVE_TO_CAP_BALL,
        ROTATE,
        WAIT_CROSS,
        DRIVE_TO_WALL,
        WAIT_PARK,
        PARK,
        STOP

    } //End of steps for autonomous

    public steps CURRENT_STEP = steps.START_RESET; //Sets the variable CURRENT_STEP to the first step in the sequence

    private ElapsedTime runtime = new ElapsedTime(); //Creates a variable for runtime so we can have timed events

    private ElapsedTime matchRuntime = new ElapsedTime(); //Creates a variable for runtime so we can have timed events

    @Override //Method overrides parent class
    public void loop() { //Starts loop for the program

        ///////////////////////////
        //Telemetry for debugging//
        ///////////////////////////

        telemetry.addData(">", "Range (CM): " + range.getDistance(DistanceUnit.CM) + "\nzValue: " + gyro.getIntegratedZValue() + "\nRed: " + colorSensor.red() + "\nMatch Runtime variable: " + matchRuntime + "\nStep: " + CURRENT_STEP); //Adds telemetry to debug
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
                matchRuntime.reset();
                m5.setMaxSpeed(1800);  //Set max speed medium for shooter
                m6.setMaxSpeed(1800);
                s3.setPosition(0.35); //Forks pointed up
                CURRENT_STEP = steps.SPEED_UP_SHOOTERS; //Sets next step to DRIVE_TO_CAP_BALL
                break;

            case SPEED_UP_SHOOTERS:

                if (runtime.seconds() > 2.5) { //Sets power of launcher motors to 1 with their max speed set to 1600 (see TardisOpModeAutonomous) for 1.5 seconds
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

            case SHOOT_FIRST_BALL: //Beginning of case statement SHOOT_FIRST_BALL

                if (runtime.seconds() > .5) { //Raises ball into launcher by setting servo position to launching position for 1 second
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

                if (runtime.seconds() > 2.5) { //Lowers launching servo to original position for 1.5 seconds
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.SHOOT_SECOND_BALL; //Sets next step to SHOOT_SECOND_BALL
                    break; //Exits switch statement
                } //End of if statement
                s2.setPosition(.05); //Sets launching servo to original position
                break; //Exits switch statement

            //////////////////////////////
            //Step 6 [SHOOT_SECOND_BALL]//
            //////////////////////////////

            case SHOOT_SECOND_BALL: //Beginning of case statement SHOOT_SECOND_BALL

                if (runtime.seconds() > .5) { //Raises ball into launcher by setting servo position to launching position for 1 second
                    m1.setPower(0); //Sets motor 1 to power 0 before next step
                    m2.setPower(0); //Sets motor 2 to power 0 before next step
                    m3.setPower(0); //Sets motor 3 to power 0 before next step
                    m4.setPower(0); //Sets motor 4 to power 0 before next step
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.DRIVE_TO_CAP_BALL; //Sets next step to LOWER_SERVO_2
                    break; //Exits switch statement
                } //End of if statement
                s2.setPosition(1); //Sets launching servo to launching position
                break; //Exits switch statement

            case DRIVE_TO_CAP_BALL:

                if (runtime.seconds() > 3) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    s2.setPosition(.05); //Sets launching servo to launching position
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.ROTATE; //Sets next step to FIND_WHITE_LINE_BEACON_ONE
                    break; //Exits switch statement
                }
                if (gyro.getIntegratedZValue() < 0) { //If gyro senses a tilt, it lowers the speed of motor 1 to correct itself
                    m1.setPower(.8); //Sets motor 1 to power .8 to go diagonally toward beacon one
                    m2.setPower(0); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(0); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(1); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > 0) { //If gyro senses a tilt, it lowers the speed of motor 4 to correct itself
                    m1.setPower(1); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(0); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(0); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(.8); //Sets motor 4 to power .8 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == 0) { //If gyro senses no tilt, it will continue to move at full power with both motors 1 and 4
                    m1.setPower(1); //Sets motor 1 to power 1 to go diagonally toward beacon one
                    m2.setPower(0); //Sets motor 2 to power 0 to go diagonally toward beacon one
                    m3.setPower(0); //Sets motor 3 to power 0 to go diagonally toward beacon one
                    m4.setPower(1); //Sets motor 4 to power 1 to go diagonally toward beacon one
                    break; //Exits switch statement
                } //End of if statement

            case ROTATE:

                if (gyro.getIntegratedZValue() > -80) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.WAIT_CROSS; //Sets next step to STOP
                    break; //Exits switch statement
                }
                m1.setPower(.2); //Sets motor 1 to power .5 to move the robot toward the center vortex
                m2.setPower(-.2); //Sets motor 2 to power -.5 to move the robot toward the center vortex
                m3.setPower(.2); //Sets motor 3 to power -.5 to move the robot toward the center vortex
                m4.setPower(-.2); //Sets motor 4 to power .5 to move the robot toward the center vortex
                break;

            case WAIT_CROSS:

                if (matchRuntime.seconds() > 10) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.DRIVE_TO_WALL; //Sets next step to STOP
                    break; //Exits switch statement
                }
                m1.setPower(0);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(0);
                break;

            case DRIVE_TO_WALL:

                if (range.getDistance(DistanceUnit.CM) < 30) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.WAIT_PARK; //Sets next step to STOP
                    break; //Exits switch statement
                }
                if (gyro.getIntegratedZValue() < -90) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(.5); //Sets motor 1 to power .25 to go right and look for the beacon two white line
                    m2.setPower(-.4); //Sets motor 2 to power -.25 to go right and look for the beacon two white line
                    m3.setPower(-.5); //Sets motor 3 to power -.35 to go right and look for the beacon two white line
                    m4.setPower(.4); //Sets motor 4 to power .35 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > -90) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(.4); //Sets motor 1 to power .35 to go right and look for the beacon two white line
                    m2.setPower(-.5); //Sets motor 2 to power -.35 to go right and look for the beacon two white line
                    m3.setPower(-.4); //Sets motor 3 to power -.25 to go right and look for the beacon two white line
                    m4.setPower(.5); //Sets motor 4 to power .25 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == -90) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .35
                    m1.setPower(.5); //Sets motor 1 to power .35 to go right and look for the beacon two white line
                    m2.setPower(-.5); //Sets motor 2 to power -.35 to go right and look for the beacon two white line
                    m3.setPower(-.5); //Sets motor 3 to power -.35 to go right and look for the beacon two white line
                    m4.setPower(.5); //Sets motor 4 to power .35 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement.
                break; //Exits switch statement

            case WAIT_PARK:

                if (matchRuntime.seconds() > 27) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.DRIVE_TO_WALL; //Sets next step to STOP
                    break; //Exits switch statement
                }
                m1.setPower(0);
                m2.setPower(0);
                m3.setPower(0);
                m4.setPower(0);
                break;

            case PARK:

                if (range.getDistance(DistanceUnit.CM) < 30) {
                    m1.setPower(0);
                    m2.setPower(0);
                    m3.setPower(0);
                    m4.setPower(0);
                    runtime.reset(); //Resets time before switching to next step
                    CURRENT_STEP = steps.WAIT_PARK; //Sets next step to STOP
                    break; //Exits switch statement
                }
                if (gyro.getIntegratedZValue() < 90) { //If gyro senses a tilt, it lowers the speed of motor 1 and motor 2 to correct itself
                    m1.setPower(-.5); //Sets motor 1 to power .25 to go right and look for the beacon two white line
                    m2.setPower(.4); //Sets motor 2 to power -.25 to go right and look for the beacon two white line
                    m3.setPower(.5); //Sets motor 3 to power -.35 to go right and look for the beacon two white line
                    m4.setPower(-.4); //Sets motor 4 to power .35 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() > 90) { //If gyro senses a tilt, it lowers the speed of motor 3 and motor 4 to correct itself
                    m1.setPower(-.4); //Sets motor 1 to power .35 to go right and look for the beacon two white line
                    m2.setPower(.5); //Sets motor 2 to power -.35 to go right and look for the beacon two white line
                    m3.setPower(.4); //Sets motor 3 to power -.25 to go right and look for the beacon two white line
                    m4.setPower(-.5); //Sets motor 4 to power .25 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement
                if (gyro.getIntegratedZValue() == 90) { //If gyro senses no tilt, it continues to go right with all drive train motors set to a power of .35
                    m1.setPower(-.5); //Sets motor 1 to power .35 to go right and look for the beacon two white line
                    m2.setPower(.5); //Sets motor 2 to power -.35 to go right and look for the beacon two white line
                    m3.setPower(.5); //Sets motor 3 to power -.35 to go right and look for the beacon two white line
                    m4.setPower(-.5); //Sets motor 4 to power .35 to go right and look for the beacon two white line
                    break; //Exits switch statement
                } //End of if statement.
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
        } //End of switch statement
    } //End of loop
} //End of program