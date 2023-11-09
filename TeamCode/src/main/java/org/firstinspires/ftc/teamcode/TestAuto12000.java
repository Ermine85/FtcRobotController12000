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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;



@Autonomous(name="12000 Test Auto", group="Robot")
public class TestAuto12000 extends LinearOpMode {

    public int newRightTarget = 0;
    public int newLeftTarget = 0;
    private DcMotor LeftOmni = null; //left_omni
    private DcMotor RightOmni = null; //right_omni
    private DcMotor LeftWheel = null; //left_wheel
    private DcMotor RightWheel = null; //right_wheel

    private IMU RobotIMU = null;
    private Servo Stick = null;
    Robot12000 RobotFunctions = new Robot12000(this);
    //New wheel diameter of 12 cm
    static final double     COUNTS_PER_INCH  = (560 * 1) / (4 * 3.1415); //Counts per motor rev * Gear Reduction / Wheel diameter * pi
    @Override
    public void runOpMode() {

        LeftOmni = hardwareMap.get(DcMotor.class, "left_omni");
        RightOmni = hardwareMap.get(DcMotor.class, "right_omni");
        LeftWheel = hardwareMap.get(DcMotor.class, "left_wheel");
        RightWheel = hardwareMap.get(DcMotor.class, "right_wheel");
        Stick = hardwareMap.get(Servo.class, "cheese_stick");
        RobotIMU = hardwareMap.get(IMU.class, "imu");

        LeftOmni.setDirection(DcMotor.Direction.FORWARD);
        LeftWheel.setDirection(DcMotor.Direction.FORWARD);
        RightOmni.setDirection(DcMotor.Direction.REVERSE);
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        //Test Move
        Stick.setPosition(1);
        sleep(1000);
        Move(0.5, 45);
        Turn(90.0);

    } // :)
    public void Turn(double degrees)
    {
        RobotIMU.resetYaw();
        double Current = RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        while(Current < degrees)
        {
            Current = RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            RightWheel.setPower(0.5);
            LeftWheel.setPower(-0.5);

        }

        LeftOmni.setPower(0);
        RightOmni.setPower(0);
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

 //       while(Current < degrees)
 //       {
            RightWheel.setPower(-0.5);
            LeftWheel.setPower(0.5);
 //       }

    }

    public void Move(double speed, double distanceInch)
    {
        //WheelEncoder(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //WheelEncoder(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newLeftTarget =  (int)(distanceInch * COUNTS_PER_INCH);
        newRightTarget =  (int)(distanceInch * COUNTS_PER_INCH);

        //LeftWheel.setTargetPosition(newLeftTarget);
        RightWheel.setTargetPosition(newRightTarget);

        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightWheel.setPower(speed);
        //LeftWheel.setPower(speed);
        while (RightWheel.isBusy())
        {
            //LeftWheel.setPower(speed/1.14);
            LeftWheel.setPower(speed);

        }

        RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftOmni.setPower(0);
        RightOmni.setPower(0);
        LeftWheel.setPower(0);
        RightWheel.setPower(0);
    }




}
