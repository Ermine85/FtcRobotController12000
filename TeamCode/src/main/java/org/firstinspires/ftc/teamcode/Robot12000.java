package org.firstinspires.ftc.teamcode;
//Contains Functions For OpMOde and Autonomous
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
public class Robot12000 {
    //Wheel Motors (6 total -- 4 omni -- 2 normal) ~Change to 4 mecanum later~
    //Distance per inch (encoder)
    static final double     COUNTS_PER_INCH  = (537.6 * 1) / (2 * 3.1415); //Counts per motor rev * Gear Reduction / Wheel diameter * pi
    private DcMotor LeftFront = null; //left_front
    private DcMotor RightFront = null; //right_front
    private DcMotor LeftBack = null; //left_back
    private DcMotor RightBack = null; //right_back
    private Servo IntakeLift = null; //intake_lift

    private DcMotor Intake = null;
    private DcMotor Arm = null;
    private Servo Drone = null;
    private CRServo PixelPlacer = null;
    private boolean FlippedDrive = false;
    private IMU RobotIMU = null;
    private boolean HighINTAKE = false;

    //Auto Move Distance
    public int newRightTarget = 0;
    public int newLeftTarget = 0;
    private LinearOpMode OpMode = null;
    public Robot12000(LinearOpMode opMode) { OpMode = opMode; }

    //All Variable/Motor SetUps
    public void init()
    {
        //Variable Setting
        //Wheel-Motors
        LeftFront = OpMode.hardwareMap.get(DcMotor.class, "left_front");
        RightFront = OpMode.hardwareMap.get(DcMotor.class, "right_front");
        LeftBack = OpMode.hardwareMap.get(DcMotor.class, "left_back");
        RightBack = OpMode.hardwareMap.get(DcMotor.class, "right_back");
        //Other Motors
        Arm = OpMode.hardwareMap.get(DcMotor.class, "arm");
        Intake = OpMode.hardwareMap.get(DcMotor.class, "intake");
        //Servos
        IntakeLift = OpMode.hardwareMap.get(Servo.class, "intake_lift");
        PixelPlacer = OpMode.hardwareMap.get(CRServo.class, "pixel_servo");
        Drone = OpMode.hardwareMap.get(Servo.class, "drone");
        //IMU
        RobotIMU = OpMode.hardwareMap.get(IMU.class, "imu");
        //Set Wheel Direction (Headless uses all wheels forward)
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        //Other
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    //Move Function sets all motors to Powers depended on values given when function is called.
    public void Move(double LF, double RF, double LB, double RB)
    {

            RightBack.setPower(RB);
            RightFront.setPower(RF);
            LeftBack.setPower(LB);
            LeftFront.setPower(LF);

    }

    //Arm Sets Power of Arm motor to value given when called.
    public void Arm(double power)
    {
        Arm.setPower(power);
    }
    //Move OpMode

    //Changes Drive (CURRENTLY NOT-FUNCTIONAL)
    public void FlipDrive()
    {
        if(FlippedDrive = false)
        {
            FlippedDrive = true;
        }
        else if (FlippedDrive)
        {
            FlippedDrive = false;
        }
    }

    //MoveArm Moves to a position (not used)
    public void MoveArm(int position, boolean reset, double power)
    {
        //Resets Position on Encoders if reset is set to ture
        if(reset)
        {
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        //Sets it to a position based off starting position (0 will put it back to start)
        Arm.setTargetPosition(position);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(power);

        while(Arm.isBusy()){
            //Stops it from moving on if still moving to the position
        }
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Everything to do with Intake
    public void Intake(double power) //robot activating intake
    {
        //Makes it so if this function is called with a power value of > 0 it raises the hinge
        if(power != 0 && !HighINTAKE){
            IntakeLift.setPosition(0.29); //Low
            Intake.setPower(power);
        } else if (power != 0 && HighINTAKE)
        {
          Intake.setPower(power);
          IntakeLift.setPosition(0.10); //High
        } else
        {
            IntakeLift.setPosition(0.5);//Low
            //Intake.setPower(0.25);
        }
        //Sets Power
        Intake.setPower(power);
    }

    //Drone Servo
    public void Drone(double position)
    {
        Drone.setPosition(position);
    }

    //Top Auto-Pixel Placer movement
    public void PixelPlacer(double power)
    {
        PixelPlacer.setPower(power);
    }

    //Pre-Set Arm Locations (Up, Down, Plane)
    public void ArmTarget(String position)
    {
        switch(position) //switch case uses string value to help read.
        {
            case "DOWN":
                Arm.setTargetPosition(0);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(1);
                while(Arm.isBusy()){}
                Arm.setPower(0);
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return;
            case "PLANE":
                Arm.setTargetPosition(2050);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(1);
                while(Arm.isBusy()){}
                Arm.setPower(0);
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return;
            case "NORMAL":
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }
    //Change Intake hinge high from low to high or high to low.
    public void ChangeIntakeLIFT()
    {
        if(HighINTAKE)
        {
            HighINTAKE = false;
        }else if(!HighINTAKE)
        {
            HighINTAKE = true;
        }
    }
}
