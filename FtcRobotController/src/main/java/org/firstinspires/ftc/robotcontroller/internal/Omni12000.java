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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;

//import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.lang.reflect.Field;

//import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;


@TeleOp(name="Omni", group="Linear Opmode")
//@Disabled
public class Omni12000 extends LinearOpMode {

    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    private Robot12000 Functions = null;
    private boolean FlippedDrive = false;
    private boolean BackUpDrive = false;

    //private boolean ArmMode = false;
    private double armSpeed = 1;

    private int ArmTarget = 0;
    private boolean toggleReady = true;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double RobotStartAngle = 0;
    private double FieldAngle = 0;
    private boolean MoveToTarget = true;
    private double CurrentRobotAngle = 0;

    //    private String ArmPosition = "NORMAL"; //DOWN, PLANE, NORMAL
    private IMU Imu = null;


    //Motors F = Front B = Back
    //Robot12000 RobotFunctions = new Robot12000(this);

    public Thread slowIntakeThread = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        Imu = hardwareMap.get(IMU.class, "Eimu");
        telemetry.update();
        Functions = new Robot12000(this);
        Functions.init();
        RobotStartAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
       while (opModeIsActive()) {

           double max; //Used to compare wheel power
           CurrentRobotAngle = RobotStartAngle - Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
           // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
           double axial   =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value --
           double lateral =  gamepad1.left_stick_x;
           double yaw     =  -gamepad1.right_stick_x;

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

            if(gamepad1.y)
            {
                BackUpDrive = true;
            }


                //ARM
            /*if(gamepad1.dpad_up)
            {
                ArmTarget += 10; //Add a max being max height
            }else if(gamepad1.dpad_down)
            {
                if(ArmTarget > 0){ArmTarget -= 10;} //Only lowers ArmTarget if not already 0 or lower
            }

            Functions.Arm(0.5, ArmTarget);


            /*if(gamepad1.dpad_left)
            {
                ArmTarget = 0;
            }else if(gamepad1.dpad_right)
            {
                //Set Max Arm Height
            }*/

            telemetry.addData("Arm location", ArmTarget);

            //Old Arm Movement
            /*if(gamepad1.dpad_up)
            {
                Functions.ArmPower(0.4);
            } else if(gamepad1.dpad_down)
            {
                Functions.ArmPower(-0.4);
            }else
            {
                Functions.ArmPower(0);
            }*/


           telemetry.addData("IMU Angle", Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
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
           //Flip Drive-Direction


           //Arm (2 part arm, arm + extension)
           if(gamepad1.left_bumper && gamepad1.y) //Arm Out
           {
               Functions.SetIntakeArm(0.5);
           } else if(gamepad1.left_bumper && gamepad1.b) //Arm In
           {
               Functions.SetIntakeArm(-0.5);
           } else { //Arm Null
               Functions.SetIntakeArm(0);
           }


           if(gamepad1.right_bumper && gamepad1.y) //Extension Out
           {
               Functions.SetIntakeExtender(1);
           } else if(gamepad1.right_bumper && gamepad1.b) //Extension In
           {
               Functions.SetIntakeExtender(-1);
           } else { //Extension Null
               Functions.SetIntakeExtender(0);
           }


           if(!gamepad1.x && !gamepad1.left_stick_button)
           {
               toggleReady = true;
           }


           if(gamepad1.left_trigger > 0)
           {
               Functions.SetIntake(-gamepad1.left_trigger);
           }
           if(gamepad1.right_trigger > 0)
           {
               Functions.SetIntake(gamepad1.right_trigger);
           }




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
