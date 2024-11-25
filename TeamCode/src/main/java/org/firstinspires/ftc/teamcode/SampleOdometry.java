/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Vector;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "OdometryTest", group = "Concept")
// @Disabled
public class SampleOdometry extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "12000RedBaseModel.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/12000RedBaseModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       "REDdiabolo",
    };
    public int newTarget = 0;
    private DcMotor LeftFrontALE = null;
    private DcMotor LeftBackABE = null;
    private DcMotor RightFront = null;
    private DcMotor RightBackARE = null;
    private DcMotor RightEncoder = null; //Y
    private DcMotor LeftEncoder = null; //Y

    private DcMotor BackEncoder = null; //X
    //private CRServo PixelServo = null;
    //private ColorSensor colorSensor = null;

    private Vector<Double> InitialPosition = new Vector<Double>(3);
    private Vector<Double> CurrentPosition = new Vector<Double>(3);
    private double FinalDistance = 0;
    private IMU RobotIMU = null;

    private double RobotAngle = 0;

    private double TrackLength = 12.5; //Track-Length;
    private double StartAngle = 0; //setting starting robot orientation in radians


    private ElapsedTime   runtime = new ElapsedTime();
    private double COUNTS_PER_INCH  = (Math.PI * 1.25984) / 2000;

    //Other Variables
    private double DeltaX;
    private double DeltaY;
    private double DeltaA;


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    //private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {


        int position;

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        LeftFrontALE = hardwareMap.get(DcMotor.class, "left_front");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        LeftBackABE = hardwareMap.get(DcMotor.class, "left_back");
        RightBackARE = hardwareMap.get(DcMotor.class, "right_back");


        RobotIMU = hardwareMap.get(IMU.class, "imu");

        LeftFrontALE.setDirection(DcMotor.Direction.FORWARD);
        LeftBackABE.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBackARE.setDirection(DcMotor.Direction.REVERSE);

        StartAngle = RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        StartVector(InitialPosition, 0, 0, 0); // might want to be SetVector
        StartVector(CurrentPosition, 0, 0,0);  // ^

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

            // Put navigation code here based on result = 0, 1, or 2


            MoveTo(5, 5, 45, 1,5,0.5);


                // Push telemetry to the Driver Station.
            telemetry.update();

                // Save CPU resources; can resume streaming when needed.

            //visionPortal.stopStreaming();


                // Share the CPU.
            sleep(20000);
        }

        // Save more CPU resources when camera is no longer needed.
        //visionPortal.close();

    }   // end runOpMode()

    // private void FindProp() {
    // }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */

    public void MoveTo(double TargetX, double TargetY, double TargetAngle, double PositionTolerance, double AngleTolerance, double Speed)
    {
        double Start = getRuntime();
        double END = getRuntime() + 3;
        TargetAngle = (TargetAngle * (2 * Math.PI) / 360); //Convert target angle to radians
        AngleTolerance = AngleTolerance * (2* Math.PI/ 360); //Convert angle tolerance from degres to radians

        ResetOdom();

        //Set Position
        double Crx = 0; //set robot x position to 0
        double Cry = 0; //set robot y position to 0

        double Cfx = InitialPosition.get(0); //set current field position x to last known position
        double Cfy = InitialPosition.get(1); //set current field position y to last known position

        //double RobotYaw = StartAngle - RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); //find current robot field orientation in radians

        double RobotYaw = StartAngle - RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        SetVector(CurrentPosition, InitialPosition.get(0), InitialPosition.get(1), RobotYaw);

        //Find distance from target

        double ThetaF = RobotYaw;

        double RadiusToTarget = Math.sqrt(Math.pow(CurrentPosition.get(0) - TargetX, 2 ) + Math.pow(CurrentPosition.get(1) - TargetY, 2));

        double AngleDifference = Math.abs(TargetAngle-RobotYaw); //calculating the difference to angle from target
        //double RB = 5;

        //while((RadiusToTarget > PositionTolerance || AngleDifference > AngleTolerance))
        while(RadiusToTarget > PositionTolerance)
        {
            RobotYaw = StartAngle - RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //RB = Math.sqrt(Math.pow(CurrentPosition.get(0) - TargetX, 2) + Math.pow(CurrentPosition.get(1) - TargetY, 2));
            //double AngleDelta = Math.atan((TargetX - CurrentPosition.get(0))/(TargetY - CurrentPosition.get(1)));

            telemetry.addData("ImuAngle", RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("ROBOTYAW", RobotYaw);
            //Calculate robot distance and direction on robot frame of reference

            GCOV(); //Get-Current-Odom-Vaulues

            //Distance Traveled
            double D1 = COUNTS_PER_INCH * Math.sqrt(Math.pow(DeltaX, 2) + Math.pow(DeltaY, 2));


            if (DeltaX == 0) DeltaX = 0.0001; //Stops from dividing by 0

            double Theta1 = Math.atan(DeltaY/DeltaX);  //- (Math.PI / 4.0);
            telemetry.addData("Raw Robot Theta",Theta1);
            //Display "Angle"

            if (DeltaX == 0.0001 && DeltaY >= 0) {
                Theta1 = Math.PI / 4;
            } else if (DeltaX == 0.0001 && DeltaY < 0) {
                Theta1 = 5 * Math.PI / 4;
            }

            //Account for issue with arctan since it only returns 0-PI
            if (DeltaX < 0) {
                Theta1 = Theta1 + Math.PI;
            }
            //Calculate Robot frame of reference X and Y distance moved



            //Convert distance and direction from robot frame of reference to field frame of reference
            //Angle of robot movement in field reference
            ThetaF = Theta1;// + RobotYaw;  // Remove RobotYaw for now
            //Distance robot has moved in x-direction
            double Dfx = Math.sin(ThetaF) * D1;
            //Distance robot has moved in y-direction
            double Dfy = Math.cos(ThetaF) * D1;

            //track current position
            Crx = Crx + DeltaX;
            Cry = Cry + DeltaY;
            Cfx = Cfx + Dfx;
            Cfy = Cfy + Dfy;

            telemetry.addData("Angle", (RobotAngle * 180)/Math.PI);
            telemetry.addData("RtT", RadiusToTarget);
            telemetry.addData("X", Cfx);
            telemetry.addData("Y", Cfy);
            telemetry.update();

            //sleep(2000);

            //Calculate distance X and Y from target - negative to flip the coordinate system
            double DltX = (TargetX - Cfx);
            double DltY = (TargetY - Cfy);
            //find Radius to the targets
            RadiusToTarget = Math.sqrt(Math.pow(DltX, 2) + Math.pow(DltY, 2));
            //Calculate direction to target
            if (DltY == 0)  DltY = 0.001; //make sure not to divide by 0

            double Ttf = Math.atan(DltX / DltY);

            if (DltX < 0) {
                Ttf = Ttf + Math.PI;
            }
            if (DltY > 0){
                //Do nothing
            }

            Ttf = Ttf + Math.PI;


            telemetry.update();

            //RB = Math.sqrt(Math.pow(CurrentPosition.get(0) - TargetX, 2) + Math.pow(CurrentPosition.get(1) - TargetY, 2));

            RobotAngle =  Ttf - RobotYaw;

            AngleDifference = Math.abs(TargetAngle-RobotYaw);
            //Motor Speed


            double F = 1; //adding in a proportional scaling factor for distance
            if (RadiusToTarget<3/1.4)
            {
                F = (RadiusToTarget)/3+.4;
            }

            double U =1; //adding in a proportional scalaing factor for angle
            if (AngleDifference<Math.PI/4/1.2)
            {
                U = U/Math.PI/4+0.3;
            }
            double M1 = F*(Math.sin(RobotAngle) + Math.cos(RobotAngle)) + 0.5*(RobotYaw - TargetAngle); //LF
            double M2 = F*(Math.sin(RobotAngle) - Math.cos(RobotAngle)) + 0.5*(RobotYaw - TargetAngle); //RF
            double M3 = F*(-Math.sin(RobotAngle) + Math.cos(RobotAngle)) + 0.5*(RobotYaw - TargetAngle); //LB
            double M4 = F*(-Math.sin(RobotAngle) - Math.cos(RobotAngle)) + 0.5*(RobotYaw - TargetAngle); //RB
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

            /*LeftFrontALE.setPower(M1 * Speed);
            RightFront.setPower(M2 * Speed);
            LeftBackABE.setPower(M3 * Speed);
            RightBackARE.setPower(M4 * Speed);*/

            GCOV(); //Get-Current-Odom-Values

            //SetVector(CurrentPosition, CurrentPosition.get(0) + (DeltaX * COUNTS_PER_INCH), CurrentPosition.get(1) + (DeltaX * COUNTS_PER_INCH), RobotYaw);
            ResetOdom();

            //ThetaF = (ThetaF * 360) / (2 * Math.PI);
        }

        SetVector(InitialPosition, Cfx, Cfy, RobotAngle);
        LeftFrontALE.setPower(0);
        LeftBackABE.setPower(0);
        RightBackARE.setPower(0);
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


    public double GetDeltaAngle(double leftencoder, double rightencoder)
    {
        leftencoder = leftencoder * COUNTS_PER_INCH;
        rightencoder = rightencoder * COUNTS_PER_INCH;
        //double DeltaAngle = TrackLength * ((leftencoder + rightencoder/2)/(leftencoder-rightencoder));
        double DeltaAngle = ((leftencoder + rightencoder)/2)/TrackLength;
        return DeltaAngle;
    }

    public void ResetOdom()
    {
        RightBackARE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
        LeftFrontALE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset encoder
        LeftBackABE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //reset
        RightBackARE.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //start measuring encoders
        LeftFrontALE.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //start measuring encoders
        LeftBackABE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void GCOV() //GetCurrentOdomValues
    {
        double LE = LeftFrontALE.getCurrentPosition() * -1;
        double RE = RightBackARE.getCurrentPosition() * 1;
        DeltaX = LeftBackABE.getCurrentPosition() * 1;

        DeltaY = ((LE - RE) / 2);
        //double DltX = BE;

    }
}   // end class
// around the world x100000, 14000000 BPM is the craziest of all crazies
