package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class Robot {
    private DcMotor leftDriveF = null; //left_drive_front
    private DcMotor leftDriveB = null; //left_drive_back
    private DcMotor rightDriveF = null; //right_drive_front
    private DcMotor rightDriveB = null; //right_drive_back

    private Servo gateServo = null; //gate_servo
    private LinearOpMode CurrentOpMode = null;
    public Robot(LinearOpMode opMode) {
        CurrentOpMode = opMode;
    }
    public void init()
    {
        leftDriveF = CurrentOpMode.hardwareMap.get(DcMotor.class, "left_drive_front");
        leftDriveB = CurrentOpMode.hardwareMap.get(DcMotor.class, "left_drive_back");
        rightDriveF = CurrentOpMode.hardwareMap.get(DcMotor.class, "right_drive_front");
        rightDriveB = CurrentOpMode.hardwareMap.get(DcMotor.class, "right_drive_back");
        gateServo = CurrentOpMode.hardwareMap.get(Servo.class, "gate_servo");

        leftDriveF.setDirection(DcMotor.Direction.FORWARD);
        leftDriveB.setDirection(DcMotor.Direction.FORWARD);
        rightDriveF.setDirection(DcMotor.Direction.REVERSE);
        rightDriveB.setDirection(DcMotor.Direction.REVERSE);
    }

    public void Move(double LFPower, double RFPower, double LBPower, double RBPower)
    {
        leftDriveF.setPower(LFPower);
        leftDriveB.setPower(LBPower);
        rightDriveF.setPower(RFPower);
        rightDriveB.setPower(RBPower);
    }

    public void RotateServo(double position)
    {
        gateServo.setPosition(position);
    }
}
