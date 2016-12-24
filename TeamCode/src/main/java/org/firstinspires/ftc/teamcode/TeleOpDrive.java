/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
@TeleOp(name="TeleOp-2015", group="TeleOp")
@Disabled
    public class TeleOpDrive extends OpMode {

    private DcMotor mR;
    private DcMotor mL;
    private DcMotor mT1;
    private DcMotor mT2;
    /*
    Servo Claw;
    Servo ClawL;
    Servo ClawR;
    Servo Climb;*/
    Servo ZipR;
    Servo ZipL;

////////////////////////////////////////////////////////////////////////////////////////////////////

    public TeleOpDrive() {
    }

    @Override

    public void init() {
        mL = hardwareMap.dcMotor.get("L");
        mR = hardwareMap.dcMotor.get("R");
        mT1 = hardwareMap.dcMotor.get("TUD"); //arm up down
        mT2 = hardwareMap.dcMotor.get("TE"); //Arm extend
        ZipR = hardwareMap.servo.get("ZipR");
        ZipL = hardwareMap.servo.get("ZipL");
//        Claw = hardwareMap.servo.get("Claw");
//        ClawL = hardwareMap.servo.get("ClawL");
//        ClawR = hardwareMap.servo.get("ClawR");
//        Climb = hardwareMap.servo.get("Climb");
        mL.setDirection(DcMotor.Direction.REVERSE);
    }

////////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void loop() {

        //////////////////////////////////////////////////////
        //  UD: left_stick_y ranges from -1 to 1, where     //
        //  1 is full up                                    //
        //  -1 is full down                                 //
        //  LR: left_stick_x ranges from -1 to 1, where     //
        //  -1 is full left                                 //
        //  1 is full right                                 //
        //////////////////////////////////////////////////////

        float UD = gamepad1.left_stick_y;
        float LR = -gamepad1.right_stick_x;
        float UD2 = gamepad2.left_stick_y;
        float UD3 = -gamepad2.right_stick_y;
        double threshold = .2;

////////////////////////////////////////////////////////////////////////////////////////////////////


        //Forward
        if ((UD < -threshold) && (LR == 0)) {
            mL.setPower(1);
            mR.setPower(1);
        }
        //Forward and Left
        else if ((UD < -threshold) && (LR < -threshold)) {
            mL.setPower(1);
            mR.setPower(.01);
        }
        //Forward and Right
        else if ((UD < -threshold) && (LR > threshold)) {
            mL.setPower(.01);
            mR.setPower(1);
        }
        //Backward
        else if ((UD > threshold) && (LR == 0)) {
            mL.setPower(-1);
            mR.setPower(-1);
        }
        //Left
        else if ((LR < -threshold) && (UD == 0)) {
            mL.setPower(1);
            mR.setPower(-1);
        }
        //Right
        else if ((LR > threshold) && (UD == 0)) {
            mL.setPower(-1);
            mR.setPower(1);
        }

        //No move
        else {
            mL.setPower(0);
            mR.setPower(0);
        }

/////////////////////////////////////////////  Tower ///////////////////////////////////////////////

        //UP
        if (UD2 < -threshold) {
            mT1.setPower(1);
        }

        //DOWN
        else if (UD2 > threshold) {
            mT1.setPower(-1);
        }

        //NOTHING
        else {
            mT1.setPower(0);
        }



        //Extend
        if (UD3 < -threshold) {
            mT2.setPower(-1);
        }

        //Retract
        else if (UD3 > threshold) {
            mT2.setPower(1);
        }

        //NOTHING
        else {
            mT2.setPower(0);
        }

//////////////////////////////////////////////  Claw  //////////////////////////////////////////////
/*

        if (gamepad2.a) {
            Claw.setPosition(0);
        }
        else if (gamepad2.b) {
            Claw.setPosition(.33);
        }
        else if (gamepad2.x){
            Claw.setPosition(.66);
        }
        else if (gamepad2.y) {
            Claw.setPosition(1);
        }

//Rip harambe
/////////////////////////////////////////////  Climb  //////////////////////////////////////////////

        //Climb
        if (gamepad1.a) {
            Climb.setPosition(0);
        }

        else {
            Climb.setPosition(1);
        }


//////////////////////////////////////////////  ZIP  ///////////////////////////////////////////////
*/
        int RightZipLock = 1;
        int LeftZipLock = 1;

//////////////////Zipline Slapper left side//////////////////
/*        if ((gamepad1.left_trigger == 1)&&(LeftZipLock == 1)) {
            LeftZipLock = 2;
        }
        else{
                LeftZipLock = 2;
    }

        if ((gamepad1.left_bumper)&&(LeftZipLock == 2)) {
            LeftZipLock = 1;
        }
        else {
            LeftZipLock = 1;
        }



        if (LeftZipLock == 2) {
            ZipL.setPosition(.25);
        }
        else if (LeftZipLock == 1) {
            ZipL.setPosition(0.75);
        }
*/

        if ((gamepad2.left_trigger == 1)&&(LeftZipLock == 1)) {
            LeftZipLock = 2;
        }
        else if ((gamepad2.left_bumper)&&(LeftZipLock == 2)) {
            LeftZipLock = 1;
        }


        if (LeftZipLock == 2) {
            ZipL.setPosition(0.1);
        } else if (LeftZipLock == 1) {
            ZipL.setPosition(0.75);
        }




//////////////////Zipline Slapper right side//////////////////
        if ((gamepad2.right_trigger == 1)&&(RightZipLock == 1)) {
           RightZipLock = 2;
        }
        else if ((gamepad2.right_bumper)&&(RightZipLock == 2)) {
            RightZipLock = 1;
        }


        if (RightZipLock == 2) {
            ZipR.setPosition(.75);
        } else if (RightZipLock == 1) {
            ZipR.setPosition(0.25);
        }

/*
////////////////////////////////////////////////////////////////////////////////////////////////////


        if (gamepad2.right_trigger == 1) {
            ClawR.setPosition(1);
            ClawL.setPosition(1);
        }
        else
        {
            ClawR.setPosition(.5);
            ClawL.setPosition(.5);
        }


////////////////////////////////////////////////////////////////////////////////////////////////////
    }





    //@Override
    //public void stop() { }




*/
}}