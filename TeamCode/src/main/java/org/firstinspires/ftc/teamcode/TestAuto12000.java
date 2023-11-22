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
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;



@Autonomous(name="12000", group="Robot")
public class TestAuto12000 extends LinearOpMode {

    public int newTarget = 0;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor RightEncoder = null;
    private DcMotor LeftEncoder = null;

    private double FinalDistance = 0;
    private IMU RobotIMU = null;

    //Robot12000 RobotFunctions = new Robot12000(this);
    //New wheel diameter of 12 cm
    static final double     COUNTS_PER_INCH  = (819.2 * 1) / (2 * 3.1415); //Counts per motor rev * Gear Reduction / Wheel diameter * pi
    @Override
    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");
        RightEncoder = hardwareMap.get(DcMotor.class, "right_odom");
        LeftEncoder = hardwareMap.get(DcMotor.class, "left_odom");

        RobotIMU = hardwareMap.get(IMU.class, "imu");

        LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        //Test Move
        //Stick.setPosition(1);
        sleep(1000);
        Move(0.2, 1, RightEncoder, LeftEncoder); // Speed, Distance, Encoders Used


    } // :)
    /*public void Turn(double degrees)
    {
        RobotIMU.resetYaw();
        double Current = RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Start loop", 0);
        telemetry.update();

        while(Current < degrees)
        {
            Current = RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            RightWheel.setPower(-0.5);
            LeftWheel.setPower(0.5);
            telemetry.addData("Right: ", RightWheel.getCurrentPosition());
            telemetry.addData("Left: ", LeftWheel.getCurrentPosition());
            telemetry.update();
        }

        telemetry.addData("Stop loop", 0);
        telemetry.update();

        LeftOmni.setPower(0);
        RightOmni.setPower(0);
        LeftWheel.setPower(0);
        RightWheel.setPower(0);

 //       while(Current < degrees)
 //       {
            //RightWheel.setPower(-0.5);
            //LeftWheel.setPower(0.5);
 //       }

    }*/

    public void Move(double speed, double distanceInch, DcMotor encoder, DcMotor encoder2)
    {
        MotorSpeed(0, 0);
        newTarget =  (int)(distanceInch);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        encoder.setTargetPosition(newTarget);
        encoder2.setTargetPosition(newTarget);
        encoder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        encoder2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(encoder.isBusy() || encoder2.isBusy())
        {
            double difference = encoder.getCurrentPosition() - encoder2.getCurrentPosition();
            double change = Range.clip(difference, -1.2, 1.2);;
            if(difference >= 0)
            {
                MotorSpeed(speed * change, speed / change);
            } else if (difference < 0) {
                MotorSpeed(speed / change, speed * change);
            }

        }
        MotorSpeed(0, 0);
        //RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void MotorSpeed(double P1speed, double P2speed)
    {
        LeftFront.setPower(P1speed);
        RightFront.setPower(P2speed);
        LeftBack.setPower(P2speed);
        RightBack.setPower(P1speed);
        telemetry.addData("MotorSpeed", "Pair1 " + P1speed);
        telemetry.addData("MotorSpeed", "Pair2 " + P2speed);
        telemetry.update();
    }



}
