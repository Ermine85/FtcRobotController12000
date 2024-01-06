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

import android.view.inputmethod.CorrectionInfo;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;
import java.util.Vector;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.internal.ui.ThemedActivity;


@Autonomous(name="12000", group="Robot")
public class TestAuto12000 extends LinearOpMode {

    public int newTarget = 0;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor RightEncoder = null;
    private DcMotor LeftEncoder = null;
    private CRServo PixelServo = null;

    private Vector<Double> InitialPosition = new Vector<Double>(3);
    private Vector<Double> CurrentPosition = new Vector<Double>(3);
    private double FinalDistance = 0;
    private IMU RobotIMU = null;
    private double StartAngle = 0; //setting starting robot orientation in radians

    //Robot12000 RobotFunctions = new Robot12000(this);
    //New wheel diameter of 12 cm
    private double COUNTS_PER_INCH  = (((((2.0 * Math.PI * 2.0) / 8192.0) * 2.54 * 18.0) / 70.0) / 18.0 * 28.0) ; // 2pi * wheel radios / encoder tpr
    @Override
    public void runOpMode() {

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");
        RightEncoder = hardwareMap.get(DcMotor.class, "right_odom");
        LeftEncoder = hardwareMap.get(DcMotor.class, "left_odom");
        PixelServo = hardwareMap.get(CRServo.class, "pixel_servo");

        RobotIMU = hardwareMap.get(IMU.class, "imu");

        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        StartAngle = RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        StartVector(InitialPosition, 0, 0, 10); // might want to be SetVector
        StartVector(CurrentPosition, 0, 0,10);  // ^
        waitForStart();
        //Red Audience side code
        // Move to used inches in x and y direction with respect to front of robot
        MoveTo(-10, 23, 0, .1,5,0.4);  // Place pixel on left strike mark
        sleep(2000);
        MoveTo(0, 0, 90, 1,5,0.5);
        //sleep(2000);
        //MoveTo(0,15, 0,2,10,0.4);

       // MoveTo(-43,0, 0, 2,3,0.4);
        //MoveTo(10, 40, 45, 1,5, 0.4 );
        //MoveTo(0, 0, 0, 1,5,0.4);  // This MoveTo goes diagonally 18 inches forward and 18 inches to the right

        sleep(1000);



    } // :)

    /* public void Move(double speed, double distanceInch, DcMotor encoder, DcMotor encoder2)
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

    } */
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

    public void MoveTo(double TargetX, double TargetY, double TargetAngle, double PositionTolerance, double AngleTolerance, double Speed)
    {
        TargetAngle = (TargetAngle * (2 * Math.PI) / 360); //Convert target angle to radians
        AngleTolerance = AngleTolerance * (2* Math.PI/ 360); //Convert angle tolerance from degres to radians
        RightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
        LeftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
        RightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //start measuring encoders
        LeftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //start measuring encoders
        double Crx = 0; //set robot x position to 0
        double Cry = 0; //set robot y position to 0
        double Cfx = InitialPosition.get(0); //set current field position x to last known position
        double Cfy = InitialPosition.get(1); //set current field position y to last known position
        double RobotYaw = StartAngle - RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //find current robot field orientation in radians

        SetVector(CurrentPosition, InitialPosition.get(0), InitialPosition.get(1), RobotYaw);

        //Find distance from target

        double ThetaF = RobotYaw;
        double R = Math.sqrt(Math.pow(CurrentPosition.get(0) - TargetX, 2 ) + Math.pow(CurrentPosition.get(1) - TargetY, 2));
        double J = Math.abs(TargetAngle-RobotYaw); //calculating the difference to angle from target
        //double RB = 5;

        while(R > PositionTolerance || J > AngleTolerance ){
            //RB = Math.sqrt(Math.pow(CurrentPosition.get(0) - TargetX, 2) + Math.pow(CurrentPosition.get(1) - TargetY, 2));
            //double AngleDelta = Math.atan((TargetX - CurrentPosition.get(0))/(TargetY - CurrentPosition.get(1)));
            RobotYaw = StartAngle - RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //Calculate robot distance and direction on robot frame of reference
            double D1 = COUNTS_PER_INCH * Math.sqrt(Math.pow(LeftEncoder.getCurrentPosition(), 2) + Math.pow(RightEncoder.getCurrentPosition(), 2));
            double alpha = LeftEncoder.getCurrentPosition();
            double beta = RightEncoder.getCurrentPosition();

            double Theta1 = Math.atan(beta / alpha) - (Math.PI / 4.0);
            telemetry.addData("Raw Robot Theta",Theta1);


            if (alpha == 0 && beta >= 0) {
                Theta1 = Math.PI / 4;
            } else if (alpha == 0 && beta < 0) {
                Theta1 = 5 * Math.PI / 4;
            }

            //Account for issue with arctan since it only returns 0-PI
            if (alpha < 0) {
                Theta1 = Theta1 + Math.PI;
            }
            //Calculate Robot frame of reference X and Y distance moved
            double Drx = Math.sin(Theta1) * D1;
            double Dry = Math.cos(Theta1) * D1;

            //Convert distance and direction from robot frame of reference to field frame of reference
            //Angle of robot movement in field reference
            ThetaF = Theta1 + RobotYaw;  // Remove RobotYaw for now
            //Distance robot has moved in x-direction
            double Dfx = Math.sin(ThetaF) * D1;
            //Distance robot has moved in y-direction
            double Dfy = Math.cos(ThetaF) * D1;

            //track current position
            Crx = Crx + Drx;
            Cry = Cry + Dry;
            Cfx = Cfx + Dfx;
            Cfy = Cfy + Dfy;

            //Calculate distance X and Y from target - negative to flip the coordinate system
            double DeltaX = (TargetX - Cfx);
            double DeltaY = (TargetY - Cfy);

            //calculate distance from target
            R = Math.sqrt(Math.pow(DeltaX, 2) + Math.pow(DeltaY, 2));
            //Calculate direction to target
            if (DeltaY == 0)  DeltaY = 0.001;
            double Ttf = Math.atan(DeltaX / DeltaY);
            if (DeltaY < 0) {
                Ttf = Ttf + Math.PI;
            }
            if (DeltaY > 0){
                //Do nothing
            }

            Ttf = Ttf + Math.PI;

            telemetry.addData("Robot TargetX",TargetX);
            telemetry.addData("Robot TargetY",TargetY);
            telemetry.addLine();
            telemetry.addData("Robot X",Crx);
            telemetry.addData("Robot Y",Cry);
            telemetry.addLine();
            telemetry.addData("Field X",Cfx);
            telemetry.addData("Field Y",Cfy);
            telemetry.addLine();
            telemetry.addData("Delta to target angle",(TargetAngle-RobotYaw)*180/Math.PI);
            telemetry.addData("Field Theta",ThetaF);
            telemetry.addLine();
            telemetry.addData("target direction",Ttf*360/(2*Math.PI));
            telemetry.addData("Target Radius: ", R);
            telemetry.addData("robot imu", RobotYaw*360/(2*Math.PI));
            telemetry.update();

            //RB = Math.sqrt(Math.pow(CurrentPosition.get(0) - TargetX, 2) + Math.pow(CurrentPosition.get(1) - TargetY, 2));

            double RobotAngle =  Ttf - RobotYaw;
            J = Math.abs(TargetAngle-RobotYaw);
            //Motor Speed
            double F=1; //adding in a proportional scaling factor for distance
            if (R<3/1.4) {
                F = (R)/3+.4;
            }
            double U =1; //adding in a proportional scalaing factor for angle
            if (J<Math.PI/4/1.2) {
                U = U/Math.PI/4+0.25;
            }
            double M1 = F*(Math.sin(RobotAngle) + Math.cos(RobotAngle)) + U*(RobotYaw - TargetAngle); //LF
            double M2 = F*(Math.sin(RobotAngle) - Math.cos(RobotAngle)) + U*(RobotYaw - TargetAngle); //RF
            double M3 = F*(-Math.sin(RobotAngle) + Math.cos(RobotAngle)) + U*(RobotYaw - TargetAngle); //LB
            double M4 = F*(-Math.sin(RobotAngle) - Math.cos(RobotAngle)) + U*(RobotYaw - TargetAngle); //RB
            double Mmax;

            Mmax = Math.max(M1, M2);
            Mmax = Math.max(Mmax, M3);
            Mmax = Math.max(Mmax, M4);

            if(Mmax > 1)
            {
                M1 = M1/Mmax;
                M2 = M2/Mmax;
                M3 = M3/Mmax;
                M4 = M4/Mmax;
            }

            LeftFront.setPower(M1 * Speed);
            RightFront.setPower(M2 * Speed);
            LeftBack.setPower(M3 * Speed);
            RightBack.setPower(M4 * Speed);

           // RB = Math.sqrt(Math.pow(CurrentPosition.get(0) - TargetX, 2) + Math.pow(CurrentPosition.get(1) - TargetY, 2));


            //RobotYaw = RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            //double DeltaX = (RightEncoder.getCurrentPosition() * Math.cos(Math.PI/4 + RobotYaw)) - (LeftEncoder.getCurrentPosition() * Math.cos(Math.PI/4 - RobotYaw));
            //double DeltaY = (RightEncoder.getCurrentPosition() * Math.sin(Math.PI/4 + RobotYaw)) + (LeftEncoder.getCurrentPosition() * Math.sin(Math.PI/4 - RobotYaw));

            //SetVector(CurrentPosition, CurrentPosition.get(0) + (DeltaX * COUNTS_PER_INCH), CurrentPosition.get(1) + (DeltaY * COUNTS_PER_INCH), RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            RightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LeftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //ThetaF = (ThetaF * 360) / (2 * Math.PI);
        }
        SetVector(InitialPosition, Cfx, Cfy, RobotYaw + Math.PI);
        LeftFront.setPower(0);
        LeftBack.setPower(0);
        RightBack.setPower(0);
        RightFront.setPower(0);
    }

    public void SetVector(Vector vector, double X, double Y, double angle )
    {
        vector.set(0, X);
        vector.set(1, Y);
        vector.set(2, angle);
    }
    public void StartVector(Vector vector, double X, double Y, double angle)
    {
        vector.add(X);
        vector.add(Y);
        vector.add(angle);
    }

    public void PlacePixel(int time, double speed)
    {
        PixelServo.setPower(speed);
        sleep(time);
        PixelServo.setPower(-speed);
        sleep(time);
        PixelServo.setPower(0);
    }


}
