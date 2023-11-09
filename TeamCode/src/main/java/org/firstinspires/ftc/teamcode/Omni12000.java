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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;


@TeleOp(name="Omni", group="Linear Opmode")
//@Disabled
public class Omni12000 extends LinearOpMode {

    private DcMotor LeftOmni = null;
    private DcMotor LeftWheel = null;
    private DcMotor RightOmni = null;
    private DcMotor RightWheel = null;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //Motors F = Front B = Back
    //Robot12000 RobotFunctions = new Robot12000(this);

    public Thread slowIntakeThread = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //RobotFunctions.init();
        LeftOmni = hardwareMap.get(DcMotor.class, "left_omni");
        RightOmni = hardwareMap.get(DcMotor.class, "right_omni");
        LeftWheel = hardwareMap.get(DcMotor.class, "left_wheel");
        RightWheel = hardwareMap.get(DcMotor.class, "right_wheel");
        LeftOmni.setDirection(DcMotor.Direction.FORWARD);
        LeftWheel.setDirection(DcMotor.Direction.REVERSE);
        RightOmni.setDirection(DcMotor.Direction.FORWARD);
        RightWheel.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
       while (opModeIsActive()) {
           double max;

           // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
           double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
           double lateral =  gamepad1.left_stick_x;
           double yaw     =  gamepad1.right_stick_x;

           double leftFrontPower  = axial + lateral + yaw;
           double rightFrontPower = axial - lateral - yaw;
           double leftBackPower   = axial - lateral + yaw;
           double rightBackPower  = axial + lateral - yaw;

           // Normalize the values so no wheel power exceeds 100%
           // This ensures that the robot maintains the desired motion.
           max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
           max = Math.max(max, Math.abs(leftBackPower));
           max = Math.max(max, Math.abs(rightBackPower));

           if (max > 1.0) {
               leftFrontPower  /= max;
               rightFrontPower /= max;
               leftBackPower   /= max;
               rightBackPower  /= max;
           }


            // Runs Move Function In Robot12000 Script -- Constant
            //RobotFunctions.MoveOmni(leftBackPower, leftFrontPower, rightBackPower, rightFrontPower);
           RightOmni.setPower(rightBackPower);
           RightWheel.setPower(rightFrontPower);
           LeftOmni.setPower(leftBackPower);
           LeftWheel.setPower(leftFrontPower);
           // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
       }
    }
}
