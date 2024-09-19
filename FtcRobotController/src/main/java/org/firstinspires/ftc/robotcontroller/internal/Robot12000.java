package org.firstinspires.ftc.robotcontroller.internal;
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

    //Auto Move Distance
    public int newRightTarget = 0;
    public int newLeftTarget = 0;
    private LinearOpMode OpMode = null;
    public Robot12000(LinearOpMode opMode) { OpMode = opMode; }

    //All Variable/Motor SetUps
    public void init()
    {
        LeftFront = OpMode.hardwareMap.get(DcMotor.class, "left_front");
        RightFront = OpMode.hardwareMap.get(DcMotor.class, "right_front");
        LeftBack = OpMode.hardwareMap.get(DcMotor.class, "left_back");
        RightBack = OpMode.hardwareMap.get(DcMotor.class, "right_back");
        Intake = OpMode.hardwareMap.get(DcMotor.class, "intake");
        IntakeLift = OpMode.hardwareMap.get(Servo.class, "intake_lift");
        PixelPlacer = OpMode.hardwareMap.get(CRServo.class, "pixel_servo");
        Arm = OpMode.hardwareMap.get(DcMotor.class, "arm");
        Drone = OpMode.hardwareMap.get(Servo.class, "drone");
        RobotIMU = OpMode.hardwareMap.get(IMU.class, "imu");
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.FORWARD);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    public void Move(double LF, double RF, double LB, double RB)
    {

            RightBack.setPower(RB);
            RightFront.setPower(RF);
            LeftBack.setPower(LB);
            LeftFront.setPower(LF);

    }

    public void Arm(double power)
    {
        Arm.setPower(power);
    }
    //Move OpMode
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

    public void MoveArm(int position, boolean reset, double power)
    {
        if(reset)
        {
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        Arm.setTargetPosition(position);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(power);

        while(Arm.isBusy()){

        }
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void Intake(double power) //robot activating intake
    {

        if(power != 0){
            IntakeLift.setPosition(0.25);
            Intake.setPower(power);
        } else
        {
            IntakeLift.setPosition(0.5);
            //Intake.setPower(0.25);
        }
        Intake.setPower(power);
    }

    public void Drone(double position)
    {
        Drone.setPosition(position);
    }


    public void PixelPlacer(double power)
    {
        PixelPlacer.setPower(power);
    }

    public void ArmTarget(String position)
    {
        switch(position)
        {
            case "DOWN":
                Arm.setTargetPosition(0);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(1);
                while(Arm.isBusy())
                {

                }
                Arm.setPower(0);
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return;
            case "PLANE":
                Arm.setTargetPosition(500);
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setPower(1);
                while(Arm.isBusy())
                {

                }
                Arm.setPower(0);
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                return;
            case "NORMAL":
                Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}
