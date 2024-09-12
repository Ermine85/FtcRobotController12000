package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.inter

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="why", group="Linear Opmode")
//Why would you run this
public class stupidDrive extends LinearOpMode {
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;

    private Double TurnSpeed = 0.5; //Change to change turn speed (Smaller = faster)
    //private Robot12000 Functions = null;

    @Override
    public void runOpMode()
    {

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");

        /*LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD); Option  1*/
        // /*
        LeftFront.setDirection(DcMotor.Direction.REVERSE); //Option 2
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD); // */

        waitForStart();

        while(opModeIsActive())
        {
            //The only one you need :(
            double stickValue = -gamepad1.left_stick_y;

            double Modified = stickValue * TurnSpeed;
            double Natural = stickValue;

            telemetry.addData("Modified", Modified);
            telemetry.addData("Natural", Natural);
            telemetry.update();

            if(stickValue >  0)
            {
                LeftBack.setPower(Natural);
                LeftFront.setPower(Natural);
                RightBack.setPower(Natural);
                RightFront.setPower(Natural);

            } else if(stickValue < 0)
            {
                LeftBack.setPower(Natural);
                LeftFront.setPower(Modified);
                RightBack.setPower(Natural);
                RightFront.setPower(Modified);
            }
            else {
                LeftBack.setPower(0);
                LeftFront.setPower(0);
                RightBack.setPower(0);
                RightFront.setPower(0);
            }


        }

    }

}
