package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import com.qualcomm.robotcore.hardware.inter


@TeleOp(name="Headless", group="Linear Opmode")
//Why would you run this
public class HeadlessOnly extends LinearOpMode {
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private double CurrentRobotAngle = 0;

    private IMU Imu = null;
    private double RobotStartAngle = 0;

    private Double TurnSpeed = 0.5; //Change to change turn speed (Smaller = faster)
    //private Robot12000 Functions = null;

    @Override
    public void runOpMode()
    {

        LeftFront = hardwareMap.get(DcMotor.class, "left_front");
        RightFront = hardwareMap.get(DcMotor.class, "right_front");
        LeftBack = hardwareMap.get(DcMotor.class, "left_back");
        RightBack = hardwareMap.get(DcMotor.class, "right_back");
        Imu = hardwareMap.get(IMU.class, "Eimu");
        RobotStartAngle = Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        /*LeftFront.setDirection(DcMotor.Direction.REVERSE);
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.FORWARD);
        RightBack.setDirection(DcMotor.Direction.FORWARD); Option  1*/
        // /*
        LeftFront.setDirection(DcMotor.Direction.REVERSE); //Option 2
        LeftBack.setDirection(DcMotor.Direction.REVERSE);
        RightFront.setDirection(DcMotor.Direction.REVERSE);
        RightBack.setDirection(DcMotor.Direction.REVERSE ); // */

        waitForStart();

        while(opModeIsActive())
        {
            double max;
            CurrentRobotAngle = RobotStartAngle - Imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   =  -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value --
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            if(axial == 0 && lateral == 0 )
            {
                yaw = gamepad1.right_stick_x/2;
            }
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
            }

            double FieldAngle = 0;
            telemetry.addData("yaw", yaw);
            double Speed = Math.sqrt(Math.pow(axial, 2) + Math.pow(lateral, 2));

            //FieldAngle = Math.atan(lateral/axial);
            if (axial == 0)  axial = 0.001;
            FieldAngle = Math.atan(lateral / axial);
            if (axial < 0) {
                FieldAngle = FieldAngle + Math.PI;
            }
            if (axial > 0){
                //Do nothing
            }

            FieldAngle = FieldAngle + Math.PI;



            telemetry.addData("Angle", 360*FieldAngle/(2 * Math.PI));

            double RobotAngle = FieldAngle - CurrentRobotAngle + Math.PI;
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

            LeftBack.setPower(leftBackPower);
            RightBack.setPower(rightBackPower);
            LeftFront.setPower(leftFrontPower);
            RightFront.setPower(rightFrontPower);

        }

    }

}
