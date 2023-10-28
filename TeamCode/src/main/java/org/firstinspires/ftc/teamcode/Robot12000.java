package org.firstinspires.ftc.teamcode;
//Contains Functions For OpMOde and Autonomous
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
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
    public void AMove(double speed, double distanceInch)
    {
        //WheelEncoder(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //WheelEncoder(DcMotor.RunMode.RUN_USING_ENCODER);
        RightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        newLeftTarget = LeftWheel.getCurrentPosition() + (int)(distanceInch * COUNTS_PER_INCH);
        newRightTarget = RightWheel.getCurrentPosition() + (int)(distanceInch * COUNTS_PER_INCH);

        LeftWheel.setTargetPosition(newLeftTarget);
        RightWheel.setTargetPosition(newRightTarget);

        RightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (LeftWheel.isBusy() && RightWheel.isBusy())
        {
            if(newLeftTarget > 0.01)
            {
                LeftOmni.setPower(speed);
                RightOmni.setPower(speed);
            }
            else
            {
                LeftOmni.setPower(-speed);
                RightOmni.setPower(-speed);
            }
        }

        RightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftOmni.setPower(0);
        RightOmni.setPower(0);
    }




}
