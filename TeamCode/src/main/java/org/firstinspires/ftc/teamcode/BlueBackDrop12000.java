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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Vector;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Blue-BackDrop", group = "Blue")
// @Disabled
public class BlueBackDrop12000 extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "12000BlueBaseModel.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/12000BlueBaseModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
       "REDdiabolo",
    };
    public int newTarget = 0;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor RightEncoder = null;
    private DcMotor LeftEncoder = null;
    private CRServo PixelServo = null;
    private ColorSensor colorSensor = null;
    private ColorSensor colorSensor2 = null;
    private TouchSensor PixelTouch = null;
    public double StartingNum = 0;
    private double ReturnTime;
    private Vector<Double> InitialPosition = new Vector<Double>(3);
    private Vector<Double> CurrentPosition = new Vector<Double>(3);
    private double FinalDistance = 0;
    private IMU RobotIMU = null;
    private double StartAngle = 0; //setting starting robot orientation in radians


    private ElapsedTime   runtime = new ElapsedTime();
    private double COUNTS_PER_INCH  = (((((2.0 * Math.PI * 2.0) / 8192.0) * 2.54 * 18.0) / 70.0) / 18.0 * 28.0) ; // 2pi * wheel radios / encoder tpr


    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initTfod();
        int position;

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");
        RightEncoder = hardwareMap.get(DcMotor.class, "right_odom");
        LeftEncoder = hardwareMap.get(DcMotor.class, "left_odom");
        PixelServo = hardwareMap.get(CRServo.class, "pixel_servo");
        colorSensor = hardwareMap.get(ColorSensor.class, "color1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "color2");
        PixelTouch = hardwareMap.get(TouchSensor.class, "pixel_touch");

        RobotIMU = hardwareMap.get(IMU.class, "imu");

        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        StartAngle = RobotIMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        StartVector(InitialPosition, 0, 0, 10); // might want to be SetVector
        StartVector(CurrentPosition, 0, 0,10);  // ^

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {



            // Put navigation code here based on result = 0, 1, or 2
            int result = FindProp();
            switch(result){
                case 0: //LEFT
                    MoveTo(-10, 28 ,0, 1, 10, 0.4, false);
                    sleep(500);
                    MoveTo(-10, 20, 0, 1,10,0.4, false);
                    sleep(500);
                    MoveTo(10, 5, -90, 1, 5, 0.4, false);
                    sleep(500);
                    MoveTo(-50, 5, -90, 1,5, 0.4, true);
                    sleep(500);
                    MoveTo(-50, 37, -90,1,5, 0.4, false);
                    sleep(500);
                    MoveTo(-65, 37, -90, 1,5, 0.4, false);
                    PlacePixel();


                    //Close
                    MoveTo(-50, 10, -90, 1, 5, 0.5, false);
                    MoveTo(-65, 10, -90, 1, 5, 0.5, false);
                    //Far
                    /*MoveTo(-50, 60, -90, 1, 5, 0.5, false);
                    MoveTo(-65, 60, -90, 1, 5, 0.5, false);
                    */
                    return;
                case 1: //CENTER
                    MoveTo(6, 5 ,0, 1, 5, 0.4, false);
                    sleep(500);
                    MoveTo(6, 38, 0, 1,5,0.4, false);
                    sleep(500);
                    MoveTo(10, 5, -90, 1,5,0.4, false);
                    sleep(500);
                    MoveTo(-50, 5, -90, 1,5, 0.325, true);
                    sleep(500);
                    MoveTo(-50, 38.5, -90,1,5, 0.4, false);
                    sleep(500);
                    MoveTo(-62, 38.5, -90, 1,5, 0.4, false);
                    PlacePixel();

                    //Close
                    MoveTo(-50, 5, -90, 1, 5, 0.5, false);
                    MoveTo(-65, 5, -90, 1, 5, 0.5, false);

                    //Far
                    /*MoveTo(-50, 60, -90, 1, 5, 0.5, false);
                    MoveTo(-65, 60, -90, 1, 5, 0.5, false);
                    */
                    return;
                case 2: //RIGHT OR NULL
                    MoveTo(12, 29, 40,1,5,0.3, false);
                    sleep(500);
                    //MoveTo(23.75, 28,40,2,10,0.4, false);
                    sleep(500);
                    MoveTo(7, 22, 40, 2, 10,0.4, false );
                    sleep(500);
                    MoveTo(10,5,-90,1,10,0.5, false);
                    sleep(500);
                    MoveTo(-50, 5, -90, 1,5, 0.4, true);
                    sleep(500);
                    MoveTo(-50, 52, -90,1,5, 0.35, false);
                    sleep(500);
                    MoveTo(-62, 53, -90, 1,5, 0.4, false);
                    PlacePixel();
                    //Close
                    MoveTo(-50, 10, -90, 1, 5, 0.5, false);
                    MoveTo(-65, 10, -90, 1, 5, 0.5, false);
                    //Far
                    /*MoveTo(-50, 60, -90, 1, 5, 0.5, false);
                    MoveTo(-65, 60, -90, 1, 5, 0.5, false);
                    */
                    return;

            }



                // Push telemetry to the Driver Station.
                //telemetry.update();

                // Save CPU resources; can resume streaming when needed.

            //visionPortal.stopStreaming();


                // Share the CPU.
                //sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    // private void FindProp() {
    // }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

            // With the following lines commented out, the default TfodProcessor Builder
            // will load the default model for the season. To define a custom model to load, 
            // choose one of the following:
            //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
            //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
            .setModelAssetName(TFOD_MODEL_ASSET)
            //.setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to 
            // set parameters for custom models.
            .setModelLabels(LABELS)
            //.setIsModelTensorFlow2(true)
            //.setIsModelQuantized(true)
            //.setModelInputSize(300)
            //.setModelAspectRatio(16.0 / 9.0)

            .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private int FindProp() {
        tfod.setZoom(1.0);

        double conf = 0.0d;
        double x = 0.0d;
        double y = 0.0d;
        int position = 2;

        runtime.reset();

        while (conf < 0.65 && opModeIsActive() && (runtime.seconds() <= 4.0)) {

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            telemetry.addData("# Objects Detected", currentRecognitions.size());

            // Step through the list of recognitions and display info for each one.
            for (Recognition recognition : currentRecognitions) {
                x = (recognition.getLeft() + recognition.getRight()) / 2;
                y = (recognition.getTop() + recognition.getBottom()) / 2;
                conf = recognition.getConfidence();

                telemetry.addData("", " ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                sleep(500);
            }       // end for() loop
            telemetry.update();
        }       // end while() loop
        if (runtime.seconds() > 3.0) {
            position = 2;
        }
        else {
            if (x >= 300) {
                position = 1;
            }
            else {
                if (x < 300) {
                    position = 0;
                }
                else {
                    position = 2;
                }
            }
        }

        telemetry.addData("Position", position);
        telemetry.update();
        //sleep(5000);
        return(position);
    }   // end method telemetryTfod()
    public void MoveTo(double TargetX, double TargetY, double TargetAngle, double PositionTolerance, double AngleTolerance, double Speed, boolean Color)
    {
        //double Start = getRuntime();
        double END = getRuntime() + 4;
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

        while((R > PositionTolerance || J > AngleTolerance) && (getRuntime() < END)){
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
            /*
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
            telemetry.addData("robot imu", RobotYaw*360/(2*Math.PI)); */
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

            if(((colorSensor.blue() - colorSensor.red() > colorSensor.green()) || colorSensor2.blue() - colorSensor2.red() > colorSensor2.green()) && Color)
            {
                //SetVector(InitialPosition, TargetX, Cfy, RobotYaw + Math.PI);
                Cfx = TargetX;
                //LeftFront.setPower(0);
                //LeftBack.setPower(0);
                //RightBack.setPower(0);
                //RightFront.setPower(0);
                telemetry.addData("Red", "SAW RED");
                telemetry.update();
                //sleep(1000);
                //return;
            }
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

    public void PlacePixelOld(int time, double speed)
    {
        PixelServo.setPower(-speed);
        sleep(time);
        PixelServo.setPower(speed);
        sleep(time);
        PixelServo.setPower(0);
    }
    public void PlacePixel()
    {
        double endTIME = getRuntime() + 2; //Sets auto-end Time
        double startTIME = getRuntime(); //Gets Start Time
        while(getRuntime() < endTIME && PixelTouch.isPressed() == false)// Makes sure its within time and touchsensor is not pressed
        {

            PixelServo.setPower(-0.3);
            telemetry.addData("placer", "GOING");
            telemetry.update();


        }
        double newEnd = getRuntime() + 0.1;
        while(getRuntime() < newEnd)
        {

        }

        PixelServo.setPower(0);
        ReturnTime = getRuntime() - startTIME - 0.1;
        double newTIME = getRuntime() + ReturnTime;

        while(getRuntime() < newTIME)
        {
            PixelServo.setPower(0.2);
        }

        PixelServo.setPower(0);

        while(true)
        {
            telemetry.addData("pixel", PixelServo.getPower());
            telemetry.update();
        }

    }

}   // end class
// around the world x100000, 14000000 BPM is the craziest of all crazies
