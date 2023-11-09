package org.firstinspires.ftc.teamcode;
//Contains Functions For OpMOde and Autonomous
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
public class Robot12000 {
    //Wheel Motors (6 total -- 4 omni -- 2 normal) ~Change to 4 mecanum later~
    //Distance per inch (encoder)
    static final double     COUNTS_PER_INCH  = (537.6 * 1) / (4 * 3.1415); //Counts per motor rev * Gear Reduction / Wheel diameter * pi
    private DcMotor LeftOmni = null; //left_omni
    private DcMotor RightOmni = null; //right_omni
    private DcMotor LeftWheel = null; //left_wheel
    private DcMotor RightWheel = null; //right_wheel
    private DcMotor IntakeMotor = null; //intake_motor

    private DcMotor HingeMotor = null; //hinge_motor

    private IMU RobotIMU = null;

    private Servo CheeseStick = null;

    private boolean GoingForward = true;

    //Auto Move Distance
    public int newRightTarget = 0;
    public int newLeftTarget = 0;
    private LinearOpMode OpMode = null;
    public Robot12000(LinearOpMode opMode) { OpMode = opMode; }

    //All Variable/Motor SetUps
    public void init()
    {
        LeftOmni = OpMode.hardwareMap.get(DcMotor.class, "left_omni");
        RightOmni = OpMode.hardwareMap.get(DcMotor.class, "right_omni");
        LeftWheel = OpMode.hardwareMap.get(DcMotor.class, "left_wheel");
        RightWheel = OpMode.hardwareMap.get(DcMotor.class, "right_wheel");
        IntakeMotor = OpMode.hardwareMap.get(DcMotor.class, "intake_motor");
        HingeMotor = OpMode.hardwareMap.get(DcMotor.class, "hinge_motor");
        RobotIMU = OpMode.hardwareMap.get(IMU.class, "imu");
        CheeseStick = OpMode.hardwareMap.get(Servo.class, "cheese_stick");
        LeftOmni.setDirection(DcMotor.Direction.FORWARD);
        LeftWheel.setDirection(DcMotor.Direction.FORWARD);
        RightOmni.setDirection(DcMotor.Direction.REVERSE);
        RightWheel.setDirection(DcMotor.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        HingeMotor.setDirection(DcMotor.Direction.FORWARD);

    }
    //Move OpMode
    public void Move(double leftPower, double rightPower)
    {
        LeftOmni.setPower(leftPower);
        LeftWheel.setPower(leftPower);
        RightOmni.setPower(rightPower);
        RightWheel.setPower(rightPower);
    }

    public void MoveOmni(double LBP, double LFP, double RBP, double RFP)
    {
        LeftOmni.setPower(LBP);
        LeftWheel.setPower(LFP);
        RightOmni.setPower(RBP);
        RightWheel.setPower(RBP);
    }

    //Intake Wheel SetPower
    public void Intake(double power)
    {
        IntakeMotor.setPower(power);
    }

    //Changes Wheel Types ("RUN_USING_ENCODER" "RUN_WITHOUT_ENCODER" "STOP_AND_RESET_ENCODER")
    public void WheelEncoder()
    {

        //LeftWheel.setMode(type);
        //RightWheel.setMode(type);
    }

    public void HingeMotor(double power)
    {
        HingeMotor.setPower(power);
    }

    //Autonomous Move
    public void Cheese(double position)
    {
        CheeseStick.setPosition(position);
    }

    public void Reverse()
    {
        if (GoingForward == false)
        {
            LeftOmni.setDirection(DcMotor.Direction.FORWARD);
            LeftWheel.setDirection(DcMotor.Direction.FORWARD);
            RightOmni.setDirection(DcMotor.Direction.REVERSE);
            RightWheel.setDirection(DcMotor.Direction.REVERSE);
            GoingForward = true;
        }
        else
        {
            LeftOmni.setDirection(DcMotor.Direction.REVERSE);
            LeftWheel.setDirection(DcMotor.Direction.REVERSE);
            RightOmni.setDirection(DcMotor.Direction.FORWARD);
            RightWheel.setDirection(DcMotor.Direction.FORWARD);
            GoingForward = false;
        }
    }


}
