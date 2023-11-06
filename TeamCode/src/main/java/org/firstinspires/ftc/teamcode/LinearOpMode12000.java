/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import android.provider.Telephony;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="OpMode", group="Linear Opmode")
//@Disabled
public class LinearOpMode12000 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //Motors F = Front B = Back
    Robot12000 RobotFunctions = new Robot12000(this);

    public Thread slowIntakeThread = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        RobotFunctions.init();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
       while (opModeIsActive()) {

            // Joystick set-up
            double driveY = -gamepad1.left_stick_y; //Y Direction
            double driveX = Range.clip(gamepad1.left_stick_x, -0.5, 0.5) ; //X Direction
            double turn  =  gamepad1.right_stick_x; //Turn stick

           // Calculates Power needed for the wheels based on Joystick positions
            double RightPower = Range.clip(driveY - driveX - turn, -1.0, 1.0);
            double LeftPower = Range.clip(driveY + driveX + turn, -1.0, 1.0);

            // Runs Move Function In Robot12000 Script -- Constant
            RobotFunctions.Move(LeftPower, RightPower);

            //Gets Trigger Position and runs intake off of it
            if(gamepad1.right_trigger > 0.25)
            {
                RobotFunctions.Intake(gamepad1.right_trigger); //Calls upon Intake Function (sets power of intake motor)
            } else if (gamepad1.left_trigger > 0.25) {
                RobotFunctions.Intake(-gamepad1.left_trigger); //Calls upon Intake Function
            } else if(gamepad1.right_bumper)
            {
                RobotFunctions.Intake(0.25); //Base 0.25 speed for when you need it to go slow
            } else if (gamepad1.left_bumper)
            {
                RobotFunctions.Intake(-0.25);
            } else
            {
                RobotFunctions.Intake(0); //Stops Motor if none of the triggers are pressed
            }

            if(gamepad1.dpad_down)
            {
                RobotFunctions.HingeMotor(1);
            }else if(gamepad1.dpad_up)
            {
                RobotFunctions.HingeMotor(-1);
            }else
            {
                RobotFunctions.HingeMotor(0);
            }

            if(gamepad1.dpad_left)
            {
                RobotFunctions.Cheese(0.1);
            }if(gamepad1.dpad_right)
            {
                RobotFunctions.Cheese(1);
            }
            if(gamepad1.a)
            {
                //RobotFunctions.Reverse();
            }


           // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
       }
    }
}
