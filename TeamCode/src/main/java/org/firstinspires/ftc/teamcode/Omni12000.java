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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.sun.tools.javac.comp.Check;

//import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.List;

//import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;


@TeleOp(name="Omni", group="Linear Opmode")
//@Disabled
public class Omni12000 extends LinearOpMode {

    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    private ColorSensor ColorSensor = null;
    private TouchSensor touch = null, verttouch = null;

    private Robot12000 Functions = null;
    private boolean BackUpDrive = false;

    //private boolean ArmMode = false;
    private double armSpeed = 1;

    private double ServoPos = 0;

    private int ArmTarget = 0;
    private boolean toggleReadyB = true, toggleReadyT = true; //Trigger and Buttons

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private double RobotStartAngle = 0;
    private boolean MoveToTarget = true;
    private double CurrentRobotAngle = 0;
    private Boolean HasCube = false;

    private boolean ClawOpen = false;

    private String IntakeMode = "INTAKE";

    private String CurrentCube = "";
    private String WantedCube = "RED";
    private String AfterCube = "NONE";
    private Boolean ReversDrive = false;
    private Boolean Hold = false;

    //Timers
    private double REJECTTIMER = 0, TRANSFERTIMER = -5, CUBEDELAY = 0;

    private double FieldAngle = 0;



    private Boolean ReturningArm = false, ReturningVert = false;
    //    private String ArmPosition = "NORMAL"; //DOWN, PLANE, NORMAL
    private IMU Imu = null;


    //Motors F = Front B = Back
    //Robot12000 RobotFunctions = new Robot12000(this);

    public Thread slowIntakeThread = null;
    public static double findLargest(double[] array) {
        double max = array[0];
        for (int i = 1; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
            }
        }
        return max;
    }
    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        Imu = hardwareMap.get(IMU.class, "imu");
        telemetry.update();
        Functions = new Robot12000(this);
        Imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP, //Control is UP Expansion Down
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )

        );
        Functions.init();
        Imu.resetYaw();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        ColorSensor = hardwareMap.get(ColorSensor.class, "ColorSensor");
        touch = hardwareMap.get(TouchSensor.class, "SlideTouch");
        verttouch = hardwareMap.get(TouchSensor.class, "vert_touch");

        List<Integer> list = new ArrayList<>();
        List<Integer> GreenList = new ArrayList<>();
        List<Integer> BlueList = new ArrayList<>();

        // Define the task to add red color sensor data to the list

        // run until the end of the match (driver presses STOP)
       while (opModeIsActive()) {
           telemetry.addData("Wanting Cube", WantedCube);
           telemetry.addData("After Cube", AfterCube);
           telemetry.addData("Cube is", CurrentCube);

           CheckColor(false);

           double max; //Used to compare wheel power
           CurrentRobotAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

           CurrentRobotAngle *= -1;

           /*if(CurrentRobotAngle < 0)
           {
               CurrentRobotAngle += Math.PI * 2;
           }*/
           // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.

           double axial   =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value --
           double lateral =  gamepad1.left_stick_x;

           /*(ReversDrive)
           {
               axial *= -1;
               lateral *= -1;
           }*/
           double yaw = -gamepad1.right_stick_x/2;
            /*
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
           }*/


           telemetry.addData("yaw", yaw);
           double Speed = Math.sqrt(Math.pow(axial, 2) + Math.pow(lateral, 2));

           //FieldAngle = Math.atan(lateral/axial);
           if (axial == 0)  axial = 0.001;

           FieldAngle = Math.atan(lateral / axial);

           if (axial < 0) {
               FieldAngle = FieldAngle + Math.PI;
           }
           if(axial < 0 && lateral < 0)
           {
               FieldAngle -= Math.PI * 2;
           }
           if(FieldAngle < 0)
           {
               FieldAngle += Math.PI * 2;
           }
           if (axial > 0){
               //Do nothing
           }

           //FieldAngle = FieldAngle + Math.PI;

           if(gamepad1.x && gamepad1.start)
           {
               BackUpDrive = true;
           }

            telemetry.addLine();

           telemetry.addData("lateral", lateral);
           telemetry.addData("axial", axial);
           telemetry.addLine();
           telemetry.addData("IMU Angle", CurrentRobotAngle*180/Math.PI);
           telemetry.addData("Angle", 360*FieldAngle/(2 * Math.PI));
           telemetry.addLine();


           double RobotAngle = FieldAngle - CurrentRobotAngle;



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
           } else if(BackUpDrive) {
               //Functions.Move(BleftFrontPower, BrightFrontPower, BleftBackPower, BrightBackPower);
           }

           if(gamepad1.a && toggleReadyB)
           {
               if(ReversDrive)
               {
                   ReversDrive = false;
               }else{
                   ReversDrive = true;
               }
           }

           //Flip Drive-Direction


           if(touch.isPressed() && ReturningArm)
           {
               Functions.HorzArm(0);
               ReturningArm = false;
           }

           if(!gamepad1.a  && !gamepad1.x && !gamepad1.back && !gamepad1.b)
           {
               toggleReadyB = true;
           }
           if(gamepad1.right_trigger <= 0.05 && gamepad1.left_trigger <= 0.05)
           {
               toggleReadyT = true;
           }

           /*if(gamepad1.right_trigger > 0.1)
           {
               if(IntakeMode == "INTAKE"){
                   Functions.SetIntake(-gamepad1.right_trigger/3);
               }else Functions.SetIntake(gamepad1.right_trigger/3);

           }else if (gamepad1.left_trigger > 0.1)
           {
               if(IntakeMode == "INTAKE"){
                   Functions.SetIntake(-gamepad1.left_trigger/2);
               }else Functions.SetIntake(gamepad1.left_trigger/2);
           }else {
               Functions.SetIntake(0);
           }*/

           if(gamepad1.right_trigger > 0.1 && toggleReadyT)
           {
               toggleReadyT = false;
               if(AfterCube == "NONE" || AfterCube == "YELLOW")
               {
                   SetAfter(WantedCube);

               }else if(!Hold) {
                   Hold = true;
               }else{
                   AfterCube = "NONE";
                   Functions.SetIntake(0);
                   Hold = false;
               }




           }
           if(gamepad1.left_trigger > 0.1 && toggleReadyT)
           {
               toggleReadyT = false;
               if(AfterCube != "YELLOW")
               {
                   SetAfter("YELLOW");
               }else {
                   AfterCube = "NONE";
                   Functions.SetIntake(0);
               }
           }



           if(gamepad1.back && toggleReadyB)
           {
               toggleReadyB = false;
               if(WantedCube == "BLUE")
               {
                   WantedCube = "RED";
               }else{
                   WantedCube = "BLUE";
               }
           }

           if(getRuntime() >= REJECTTIMER + 0.5 && REJECTTIMER != 0)
           {
               REJECTTIMER = 0;
               Functions.BlockServo(0.725);
           }

           if(gamepad1.x && toggleReadyB)
           {
               if(ClawOpen)
               {
                   Functions.ClawServo(0.75);
                   ClawOpen = false;
               } else {
                   Functions.ClawServo(0.55);
                   ClawOpen = true;
               }
               toggleReadyB = false;
           }

           telemetry.addData("Claw Open", ClawOpen);

           telemetry.addData("Intake Mode", IntakeMode);
           telemetry.addData("Horz-Returning", ReturningArm);
           telemetry.addData("IntakeArm-Returning", Functions.ArmReturning());
           telemetry.addData("Has Cube", HasCube);

           if(gamepad1.right_bumper) //Into Robot
           {
               if (gamepad1.b) { //Intake Arm
                   Functions.IntakeArmP(1);
               }else{ Functions.IntakeArmP(0);}

               if(gamepad1.y){ //Horz Arm
                   Functions.HorzArm(1);
               }else if(!ReturningArm){ Functions.HorzArm(0); }

           }else if(gamepad1.left_bumper) //Out of Robot
           {
               if(gamepad1.b){ //Intake Arm
                   Functions.IntakeArmP(-1);
               }else{ Functions.IntakeArmP(0);}

               if(gamepad1.y){ //Horz Arm
                   Functions.HorzArm(-1);
               }else if (!ReturningArm){ Functions.HorzArm(0); }
           }else {
               if(!ReturningArm && !Functions.ArmReturning()){
                   Functions.HorzArm(0);
                   Functions.IntakeArmP(0);
               }

           }

           if(CurrentCube != "EMPTY" && CurrentCube != AfterCube && AfterCube != "NONE")
           {
               Reject();
           }


           if(gamepad1.start && !gamepad1.x) // Entrance U Pointing Away
           {
               Imu.resetYaw();
           }

           if(gamepad1.dpad_left || gamepad2.dpad_right)
           {
               Functions.Bucket(-0.15);
               //Functions.ClawServo(0.6);
           }else{
               Functions.Bucket(0);
           }


           if(gamepad1.dpad_up){
               Functions.VertArm(1);
           }else if(gamepad1.dpad_down)
           {
               ReturningVert = true;
               Functions.VertArm(-1);
           }else if(!ReturningVert){
               Functions.VertArm(0);
           }

           if(ReturningVert)
           {
               Functions.VertArm(-1);
           }
           if(verttouch.isPressed())
           {
               Functions.VertArm(0);
               ReturningVert = false;
           }

           if(HasCube && !ReturningArm && !Functions.ArmReturning())
           {
               Functions.BlockServo(0.45);
               Transfer(-0.4);

               HasCube = false;
           }


           // Show the elapsed game time and wheel power.
            //telemetry.addData("ArmSpeed", armSpeed);
            telemetry.addData("Back Up", BackUpDrive);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
       }


    }
    void CheckColor(boolean loop)
    {
        double[] empty = {74, 120, 120};
        double[] red = {896, 170, 407};
        double[] blue = {179, 867, 365};
        double[] yellow = {1386, 363, 1724};

        double deltaE = Math.sqrt((Math.pow(empty[0] - ColorSensor.red(),2) + (Math.pow(empty[1] - ColorSensor.blue(),2)) + (Math.pow(empty[2] - ColorSensor.green(),2)) ));
        double deltaR = Math.sqrt((Math.pow(red[0] - ColorSensor.red(),2) + (Math.pow(red[1] - ColorSensor.blue(),2)) + (Math.pow(red[2] - ColorSensor.green(),2)) ));
        double deltaB = Math.sqrt((Math.pow(blue[0] - ColorSensor.red(),2) + (Math.pow(blue[1] - ColorSensor.blue(),2)) + (Math.pow(blue[2] - ColorSensor.green(),2)) ));
        double deltaY = Math.sqrt((Math.pow(yellow[0] - ColorSensor.red(),2) + (Math.pow(yellow[1] - ColorSensor.blue(),2)) + (Math.pow(yellow[2] - ColorSensor.green(),2)) ));

        double emptyConfidence = 1 - (deltaE/3)*((1 / (deltaE + deltaR)) + (1/((deltaE + deltaB))) + (1/(deltaE + deltaY)));
        double redConfidence = 1 - (deltaR/3)*((1 / (deltaR + deltaE)) + (1/((deltaR + deltaB))) + (1/(deltaR + deltaY)));
        double blueConfidence = 1 - (deltaB/3)*((1 / (deltaB + deltaE)) + (1/((deltaB + deltaE))) + (1/(deltaB + deltaY)));
        double yellowConfidence = 1 - (deltaY/3)*((1 / (deltaY + deltaE)) + (1/((deltaY + deltaE))) + (1/(deltaY + deltaE)));
        double[] confidenceValues = {emptyConfidence,redConfidence,blueConfidence,yellowConfidence};
        double largestConfidence = findLargest(confidenceValues);

        if(emptyConfidence == largestConfidence){
            CurrentCube = "EMPTY";
        } else if (redConfidence == largestConfidence) {
            CurrentCube = "RED";
        } else if (blueConfidence == largestConfidence) {
            CurrentCube = "BLUE";
        } else if (yellowConfidence == largestConfidence) {
            CurrentCube = "YELLOW";
        }

        if(AfterCube == CurrentCube)
        {
            Functions.SetIntake(0);
            AfterCube = "NONE";

            if(!Hold)
            {
                ReturnToBucket();
            }

        }
    }

    void ReturnToBucket()
    {
        ReturningArm = true;
        HasCube = true;
        Functions.HorzArm(-1);
        Functions.ReturnArm();
    }

    void Reject()
    {
        Functions.BlockServo(0.45);
        REJECTTIMER = getRuntime();
        Functions.SetIntake(-0.45);

    }
    void Transfer(double speed)
    {
        Functions.SetIntake(speed);
        Functions.BlockServo(0.45);
    }

    void SetAfter(String Color)
    {
        Functions.BlockServo(0.725);
        Functions.SetIntake(-0.5);
        AfterCube = Color;
    }

    void IntakeCube()
    {
        if(AfterCube != "NONE")
        {
            Reject();
        }
    }
}
