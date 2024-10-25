package org.firstinspires.ftc.robotcontroller.internal;
//Contains Functions For OpMOde and Autonomous

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
public class Robot12000 {
    //Wheel Motors (6 total -- 4 omni -- 2 normal) ~Change to 4 mecanum later~
    //Distance per inch (encoder)
    static final double     COUNTS_PER_INCH  = (537.6 * 1) / (2 * 3.1415); //Counts per motor rev * Gear Reduction / Wheel diameter * pi
    private DcMotor LeftFront = null; //left_front
    private DcMotor RightFront = null; //right_front
    private DcMotor LeftBack = null; //left_back
    private DcMotor RightBack = null; //right_back

    private DcMotor LeftArm = null;
    private DcMotor RightArm = null;
    private DcMotor IntakeArm = null;
    private CRServo IntakeExtender = null;

    private CRServo Intake = null;


    //Uncomment when implemented
    //private ColorSensor IntakeColor = null;


    private boolean FlippedDrive = false;
    private IMU RobotIMU = null;

    //Auto Move Distance
    public int newRightTarget = 0;
    public int newLeftTarget = 0;
    private LinearOpMode OpMode = null;
    public Robot12000(LinearOpMode opMode) { OpMode = opMode; }

    //All Variable/Motor SetUps
    public void init() //On Initialization
    {
        //Wheels + Odometry
        LeftFront = OpMode.hardwareMap.get(DcMotor.class, "left_front");
        RightFront = OpMode.hardwareMap.get(DcMotor.class, "right_front");
        LeftBack = OpMode.hardwareMap.get(DcMotor.class, "left_back");
        RightBack = OpMode.hardwareMap.get(DcMotor.class, "right_back");
        //Sliders
        LeftArm = OpMode.hardwareMap.get(DcMotor.class, "arm_left");
        RightArm = OpMode.hardwareMap.get(DcMotor.class, "arm_right");
        //Intake
        IntakeArm = OpMode.hardwareMap.get(DcMotor.class, "intake_arm");
        IntakeExtender = OpMode.hardwareMap.get(CRServo.class, "intake_extender");

        //LeftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Servos
        Intake = OpMode.hardwareMap.get(CRServo.class, "intake");
        //IMU
        RobotIMU = OpMode.hardwareMap.get(IMU.class, "imu");

        //Uncomment when implemented
        //IntakeColor = OpMode.hardwareMap.get(ColorSensor.class, "intake_color");

        //Set Wheels Facing Same Direction
        LeftFront.setDirection(DcMotor.Direction.FORWARD);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.REVERSE);

        LeftArm.setDirection(DcMotor.Direction.REVERSE);

    }

    //Move Function
    public void Move(double LF, double RF, double LB, double RB) //Sets Motors to work using pre-given power values
    {

            RightBack.setPower(RB);
            RightFront.setPower(RF);
            LeftBack.setPower(LB);
            LeftFront.setPower(LF);

    }
    //Arm Function(s)
    public void ArmPower(double power) //Run Without Encoder
    {
        RightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftArm.setPower(power); //Sets both arms to power
        RightArm.setPower(power);
    }

    public void Arm(double power, int location) //Running using encoder counts (should hold position)
    {


        //LeftArm.setTargetPosition(location); //Location
        RightArm.setTargetPosition(location);

        //LeftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Mode
        RightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LeftArm.setPower(power); //Power
        RightArm.setPower(power);

        while(RightArm.isBusy()) //Pauses until they reach location
        {
            //nothing
        }

        LeftArm.setPower(0);
        RightArm.setPower(0);

    }

    //Intake
    void SetIntake(double power) //Basic Intake Function (Not with implemented color sensor)
    {
        Intake.setPower(power);
    }

    void SetIntakeArm(double power)
    {
        IntakeArm.setPower(power);
    }


    void SetIntakeExtender(double power){
        IntakeExtender.setPower(power);
    }

    /* void SpecificIntake(String color, double power) //Intake with implemented color sensor checking
    {
        Intake.setPower(power);

        if(color == "BLUE") //if color is blue
        {

        }

        if(color == "RED")
        {

        }

    }*/ //Intake W/ Color



}
