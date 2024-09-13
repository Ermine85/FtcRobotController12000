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

package org.firstinspires.ftc.robotcontroller.internal;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Robot12000;

//import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;


@TeleOp(name="Omni", group="Linear Opmode")
//@Disabled
public class AileenTestScript extends LinearOpMode {

    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private Robot12000 Functions = null;
    private boolean FlippedDrive = false;
    private boolean BackUpDrive = false;

    //private boolean ArmMode = false;
    private double armSpeed = 1;
    private boolean toggleReady = true;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double RobotStartAngle = 0;
    private double FieldAngle = 0;
    private double CurrentRobotAngle = 0;

    private String ArmPosition = "NORMAL"; //DOWN, DRONE, NORMAL
    private IMU Imu = null;


    //Motors F = Front B = Back
    //Robot12000 RobotFunctions = new Robot12000(this);

    public Thread slowIntakeThread = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Imu = hardwareMap.get(IMU.class, "imu");
        telemetry.update();
        Functions = new Robot12000(this);
        Functions.init();
        RobotStartAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
       while (opModeIsActive()) {
           double max;
           CurrentRobotAngle = RobotStartAngle - Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
           // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate. //strafe means going side to side
           double axial   =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value --
           double lateral =  gamepad1.left_stick_x;
           double yaw     =  gamepad1.right_stick_x;
            //yaw is the spinning from side to side
           double BleftFrontPower  = axial + lateral - yaw ;
           double BrightFrontPower = -axial + lateral - yaw;
           double BleftBackPower   = axial - lateral - yaw ;
           double BrightBackPower  = -axial - lateral - yaw;

           double backUpMax;

           backUpMax = Math.max(Math.abs(BleftFrontPower), Math.abs(BrightFrontPower));
           backUpMax = Math.max(backUpMax, Math.abs(BleftBackPower));
           backUpMax = Math.max(backUpMax, Math.abs(BrightBackPower));

           if(backUpMax > 1.0)
           {
               BleftFrontPower /= backUpMax;
               BrightFrontPower /= backUpMax;
               BleftBackPower /= backUpMax;
               BrightBackPower /= backUpMax;
           }

           double FieldAngle = 0;
           telemetry.addData("yaw", yaw);
           double Speed = Math.sqrt(Math.pow(axial, 2) + Math.pow(lateral, 2));

           //FieldAngle = Math.atan(lateral/axial);
            if (axial == 0)  axial = 0.001;
            FieldAngle = Math.atan(lateral / axial);
            if (axial < 0) {
                FieldAngle = FieldAngle + Math.PI;
            }
            if (axial > 0){
                //Do nothing
            }

            FieldAngle = FieldAngle + Math.PI;


           // double FieldAngle = Math.atan(axial/lateral);



           //if(lateral == 0)
           //{
           //    FieldAngle = 0;
           //}
/*
           if(axial < 0){
               FieldAngle += Math.PI;
           }

           if (360*FieldAngle/(2 * Math.PI) == 90)
           {
               FieldAngle = (270 * 2 * Math.PI) / 360;
           }

           if (360*FieldAngle/(2 * Math.PI) == -90)
           {
               FieldAngle = (90 * 2 * Math.PI) / 360;
           }

           if (360*FieldAngle/(2 * Math.PI) < 0)
           {
               double temp = 360 + 360*FieldAngle/(2 * Math.PI);
               FieldAngle = (temp * 2 * Math.PI) / 360;
               //FieldAngle = 360 + FieldAngle;
           }
            //Odom wheels 47 mm or 1 + 7/8
            */


           telemetry.addData("Angle", 360*FieldAngle/(2 * Math.PI));

           double RobotAngle = FieldAngle - CurrentRobotAngle + Math.PI;
           double leftFrontPower = (((Math.sin(RobotAngle) + Math.cos(RobotAngle)) * Speed) - yaw); //LF
           double rightFrontPower = (((Math.sin(RobotAngle) - Math.cos(RobotAngle)) * Speed) - yaw); //RF
           double leftBackPower = (((-Math.sin(RobotAngle) + Math.cos(RobotAngle)) * Speed) - yaw); //LB
           double rightBackPower = (((-Math.sin(RobotAngle) - Math.cos(RobotAngle)) * Speed) - yaw); //RB



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

           if(!BackUpDrive)
           {
               Functions.Move(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
           } else if(BackUpDrive)
           {
               Functions.Move(BleftFrontPower, BrightFrontPower, BleftBackPower, BrightBackPower);
           }



           if(gamepad1.left_bumper)
           {
               Functions.Arm(1);
           }else if(gamepad1.right_bumper)
           {
               Functions.Arm(-1);
           }else {
               Functions.Arm(0);
           }

           //Flip Drive-Direction


           //ARM MODE


           if(!gamepad1.x && !gamd  epad1.left_stick_button)
           {
               toggleReady = true;
           }

           /*if(gamepad1.y && toggleReady)
           {
               toggleReady = false;
               if(armSpeed == 1) {
                   armSpeed = 0.1;
               }else {
                   armSpeed = 1;
               }
           }*/

           if(gamepad1.x && gamepad1.x)
           {
               BackUpDrive = true;
           }


           if(gamepad1.a)
           {
               Functions.Drone(1);
           }

           if(gamepad1.dpad_down)
           {
               Functions.PixelPlacer(1);
           }
           if(gamepad1.dpad_up)
           {
               Functions.PixelPlacer(-1);
           }


           if(gamepad1.dpad_right)
           {
               ArmPosition = "DRONE";
               Functions.ArmTarget(ArmPosition);
           }
           if(gamepad1.dpad_left)
           {
               ArmPosition = "DOWN";
               Functions.ArmTarget(ArmPosition);
           }

           if(gamepad1.b) //NOT NEEDED
           {
               ArmPosition = "NORMAL";
               Functions.ArmTarget(ArmPosition);
           }



           Functions.Intake(gamepad1.right_trigger - (gamepad1.left_trigger/3));
           //Functions.Intake(gamepad1.left_trigger);



           if(gamepad1.start && !gamepad1.x) // Camera Pointing Away
           {
               RobotStartAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
           }

           // Show the elapsed game time and wheel power.
            //telemetry.addData("ArmSpeed", armSpeed);
            telemetry.addData("Back Up", BackUpDrive);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
       }
    }
}
