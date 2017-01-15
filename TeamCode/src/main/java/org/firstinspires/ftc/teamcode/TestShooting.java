package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous; //Imports com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode; //Imports com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime; //Imports com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous(name = "Test", group = "AutoFast") //Display name and group found in on controller phone
public class TestShooting extends TardisOpModeAutonomous { //Imports presets for initiation from TardisOpModeAutonomus
    int m5previous;
    int m6previous;
    double m5power = .5;
    double m6power = .5;

    public enum steps { //All steps for completing autonomus
        START_UP_LAUNCHER_AND_COLLECTOR_MOTORS, //[Prep] Sets speed of sweeperOn motor to -1 to kick balls into teleporter and sets launcher motors to .1 so they don't go from 0 to 100 real quick (percauction to avoid gear-grinding)
    } //End of steps for autonomus

    public steps CURRENT_STEP = steps.START_UP_LAUNCHER_AND_COLLECTOR_MOTORS; //Sets the varible CURRENT_STEP to the first step in the sequence

    private ElapsedTime runtime = new ElapsedTime(); //Creates a varible for runtime so we can have timed events

    @Override //Method overrides parent class

    public void loop() { //Starts loop for the program

        telemetry.addData(">", "Motor 5 RoC: " + "\nMotor 6 RoC: "); //Adds telemetry to debug
        telemetry.update(); //Updates telemetry with new debug information

        switch (CURRENT_STEP) { //Beginning of the switch- this sets the current step to whatever the CURRENT_STEP varible is set to
            case START_UP_LAUNCHER_AND_COLLECTOR_MOTORS:
                m5.setPower(1); //Set launcher motor 5 to -.1 for prep
                m6.setPower(1); //Set launcher motor 6 to .1 for prep
                break; //Exits switch statement

        }
        m5previous = m5.getCurrentPosition();
        m6previous = m6.getCurrentPosition();
         //End of loop
    } //End of program
}