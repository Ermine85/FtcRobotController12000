package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot12000 {
    //Wheel Motors (6 total -- 4 omni -- 2 normal) ~Change to 4 mecanum later~
    private DcMotor FLeftOmni = null; //left_front_omni
    private DcMotor FRightOmni = null; //right_front_omni
    private DcMotor BLeftOmni = null; //left_back_omni
    private DcMotor BRightOmni = null; //right_back_omni
    private DcMotor LeftWheel = null; //left_wheel
    private DcMotor RightWheel = null; //right_wheel
    public LinearOpMode OpMode = null;
    public Robot12000(LinearOpMode opMode) { OpMode = opMode; }
    public void init()
    {
        FLeftOmni = OpMode.hardwareMap.get(DcMotor.class, "left_front_omni");
        FRightOmni = OpMode.hardwareMap.get(DcMotor.class, "right_front_omni");
        BLeftOmni = OpMode.hardwareMap.get(DcMotor.class, "left_back_omni");
        BRightOmni = OpMode.hardwareMap.get(DcMotor.class, "right_back_omni");
        LeftWheel = OpMode.hardwareMap.get(DcMotor.class, "left_wheel");
        RightWheel = OpMode.hardwareMap.get(DcMotor.class, "right_wheel");


    }


}
