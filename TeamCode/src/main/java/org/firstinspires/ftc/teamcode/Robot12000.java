package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
public class Robot12000 {
    //Wheel Motors (6 total -- 4 omni -- 2 normal) ~Change to 4 mecanum later~
    private DcMotor LeftOmni = null; //left_omni
    private DcMotor RightOmni = null; //right_omni
    private DcMotor LeftWheel = null; //left_wheel
    private DcMotor RightWheel = null; //right_wheel
    private DcMotor IntakeMotor = null; //intake_motor

    private LinearOpMode OpMode = null;
    public Robot12000(LinearOpMode opMode) { OpMode = opMode; }
    public void init()
    {
        LeftOmni = OpMode.hardwareMap.get(DcMotor.class, "left_omni");
        RightOmni = OpMode.hardwareMap.get(DcMotor.class, "right_omni");
        LeftWheel = OpMode.hardwareMap.get(DcMotor.class, "left_wheel");
        RightWheel = OpMode.hardwareMap.get(DcMotor.class, "right_wheel");
        IntakeMotor = OpMode.hardwareMap.get(DcMotor.class, "intake_motor");

        LeftOmni.setDirection(DcMotor.Direction.REVERSE);
        LeftWheel.setDirection(DcMotor.Direction.REVERSE);
        RightOmni.setDirection(DcMotor.Direction.FORWARD);
        RightWheel.setDirection(DcMotor.Direction.FORWARD);
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

    }
    public void Move(double leftPower, double rightPower)
    {
        LeftOmni.setPower(leftPower);
        LeftWheel.setPower(leftPower);
        RightOmni.setPower(rightPower);
        RightWheel.setPower(rightPower);
    }

    public void Intake(double power)
    {
        IntakeMotor.setPower(power);
    }


}
