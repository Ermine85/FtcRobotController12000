package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Metabot", group="Metabot")
@Disabled
public class MetabotCode extends OpMode {
    private static final double GRIPPER_OPEN_TIME = 700;
    private static final double GRIPPER_CLOSED_POS = 1;
    private static final double GRIPPER_OPEN_POS = 0.28; // 0.18

    private static final int ARM_VERTICAL = 110;

    private static final int ARM_BACK = 150;
    private static final int ARM_UP = 2100;
    //private static final int ARM_HALF = 800;
    private static final int ARM_DOWN = 0;
    private static final int ARM_SHAKE = -1;

    private static final double ARM_TOGGLE_TIME = 250;

    private static final float CONE_DISTANCE_THRESHOLD = 0.25f;

    public ElapsedTime timer = new ElapsedTime();

    private DcMotorEx left;
    private DcMotorEx right;

    private DcMotorEx arm;

    private Servo gripper;
    private DigitalChannel gripper_limit;

    private Servo rClaw;
    private Servo lClaw;

    private NormalizedColorSensor coneSensor;

    private BNO055IMU imu;

    private float targetZ = 180;

    private boolean gripperTargetState = true;
    private double gripperTimer = -GRIPPER_OPEN_TIME;
    private int armTarget = 0;
    private double armTimer = -ARM_TOGGLE_TIME;
    private float power = 0.4f;
    private int shakeDirection = 1;

    private boolean grabbingCone = false;
    private boolean shouldLiftCone = false;
    private boolean resetafter = false;

    private enum Cone {
        RED,
        BLUE,
        NONE,
    }

    @Override
    public void init() {
        left = hardwareMap.get(DcMotorEx.class, "left");
        right = hardwareMap.get(DcMotorEx.class, "right");

        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setDirection(Direction.FORWARD);

        arm.setMode(RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(RunMode.RUN_WITHOUT_ENCODER);

        left.setDirection(Direction.REVERSE);
        right.setDirection(Direction.FORWARD);

        rClaw = hardwareMap.get(Servo.class, "rClaw");
        lClaw = hardwareMap.get(Servo.class, "lClaw");

        gripper = hardwareMap.get(Servo.class, "gripper");
        gripper_limit = hardwareMap.get(DigitalChannel.class, "gripper_limit");

        coneSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // move to starting positions
        rClaw.setPosition(90);
        lClaw.setPosition(0);
        //arm.setPower(-0.8);
    }

    // this is just testing!
    @Override
    public void init_loop() {
        //gripper.setPosition(GRIPPER_CLOSED_POS);

        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.addData("cone", getCone());

        telemetry.update();
    }

    @Override
    public void loop() {
        //arm.setPower(0);
        //arm.setMode(RunMode.STOP_AND_RESET_ENCODER);
        // left trigger = gripper hold closed
        // right trigger = flappers hold open
        // a = arm toggle

        double currentTime = timer.milliseconds();

        if (gamepad1.a) {
            if (currentTime - armTimer > ARM_TOGGLE_TIME) {
                if (armTarget == ARM_DOWN) {
                    armTarget = ARM_UP;
                } else {
                    armTarget = ARM_DOWN;
                }

                armTimer = currentTime;
            }
        }

        if (gamepad1.left_trigger > 0.8) {
            /*if (!gripperTargetState) {
                gripperTargetState = true;
            }*/
            gripper.setPosition(GRIPPER_CLOSED_POS);

        } else {
            /*if (gripperTargetState) {
                gripperTargetState = false;
                gripperTimer = timer.milliseconds();
            }*/
            gripper.setPosition(GRIPPER_OPEN_POS);
        }

        // needs to be refactored, ugly and bad rn
        if (gamepad1.b) {
            grabbingCone = true;
        }

        if (!grabbingCone) {
            // do not touch the magic numbers, theyre magic
            if (gamepad1.right_trigger > 0.8) {
                rClaw.setPosition(0.56);  // middle open
                lClaw.setPosition(0.7);
            } else if (gamepad1.right_bumper) {
                rClaw.setPosition(0);  // all the way open
                lClaw.setPosition(1);
            } else {
                rClaw.setPosition(0.84);  // Closed
                lClaw.setPosition(0.4);
            }
        }

        if (grabbingCone) {
            if (getCone() == Cone.NONE) {
                // open - copied from above
                rClaw.setPosition(0.56);
                lClaw.setPosition(0.7);
            } else {
                // closed - copied from above
                rClaw.setPosition(0.84);
                lClaw.setPosition(0.4);

                grabbingCone = false;
            }
        }

        telemetry.update();
        drive(-gamepad1.left_stick_y);

        double armPos = arm.getCurrentPosition();

        if(gamepad1.x)
        {
            armPos = 1785;
            armTarget = ARM_DOWN;
            //armPos = arm.getCurrentPosition();

            resetafter = true;
        }else if(resetafter)
        {
            arm.setMode(RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODERS);
            armPos = arm.getCurrentPosition();
            resetafter = false;
        }

        if (gamepad1.y) {
            armTarget = ARM_SHAKE;
        }

        switch(armTarget) {
            case ARM_DOWN:
                if (armPos > ARM_DOWN) {
                    arm.setPower(-0.5);
                } else {
                    arm.setPower(0);
                }
                break;
            case ARM_UP:
                if (armPos < ARM_UP) {
                    if (armPos > ARM_UP - 200) {
                        arm.setPower(0.6);
                    } else {
                        arm.setPower(1);
                    }
                } else {
                    arm.setPower(0);
                }
                break;
            case ARM_SHAKE:
                if (shakeDirection == 1) {
                    if (armPos < ARM_VERTICAL + 10) {
                        arm.setPower(1);
                    } else {
                        shakeDirection = -1;
                    }
                } else {
                    if (armPos > ARM_VERTICAL - 10) {
                        arm.setPower(-1);
                    } else {
                        shakeDirection = 1;
                    }
                }
        }

        /* git
        switch(armTarget) {
            case ARM_DOWN:
                if (armPos > ARM_DOWN) {
                    if (armPos > ARM_VERTICAL - 15) {
                        arm.setPower(-1);
                    } else if (armPos < ARM_DOWN + 20) {
                        arm.setPower(-0.5);
                    } else {
                        arm.setPower(0.1);
                    }
                } else {
                    arm.setPower(0);
                }
                break;
            case ARM_UP:
                if (armPos < ARM_UP) {
                    if (armPos > ARM_UP - 10) {
                        arm.setPower(0.690);
                    } else {
                        arm.setPower(1);
                    }
                } else if (armPos > ARM_UP) {
                    if (armPos > ARM_VERTICAL - 10) {
                        arm.setPower(-1);
                    } else {
                        arm.setPower(0);
                    }
                } else {
                    arm.setPower(0.5);
                }
                break;
            case ARM_BACK:
                if (armPos < ARM_BACK) {
                    if (armPos > ARM_VERTICAL + 10) {
                        arm.setPower(0);
                    } else {
                        arm.setPower(1);
                    }
                } else {
                    arm.setPower(0);
                }
            case ARM_SHAKE:
                if (shakeDirection == 1) {
                    if (armPos < ARM_VERTICAL + 10) {
                        arm.setPower(1);
                    } else {
                        shakeDirection = -1;
                    }
                } else {
                    if (armPos > ARM_VERTICAL - 10) {
                        arm.setPower(-1);
                    } else {
                        shakeDirection = 1;
                    }
                }
        }
        */

        /*telemetry.addData("gripper limit", gripper_limit.getState());
        telemetry.addData("gripper timer", gripperTimer);
        telemetry.addData("timer", timer.milliseconds());
        telemetry.addData("gripper", gripper.getPosition()); */

        //gripper.setPosition(0);

        /*
        if (gripperTargetState) {
            // limit switch is false when pressed.
            // probably plugged in backwards
            if (gripper_limit.getState()) {
                gripper.setPower(1);
            } else {
                gripper.setPower(0);
            }
        } else {
            if (currentTime - gripperTimer < GRIPPER_OPEN_TIME) {
                gripper.setPower(-1);
            } else {
                gripper.setPower(0);
            }
        }*/

        telemetry.addData("arm", arm.getCurrentPosition());

        telemetry.update();
    }

    private void drive(float power) {
        float currentZ = imu.getAngularOrientation().firstAngle + 180;
        float diff = 0;

        float rawDiff = targetZ - currentZ;

        if (Math.abs(rawDiff) < Math.abs(rawDiff + 360) && Math.abs(rawDiff) < Math.abs(rawDiff - 360)) {
            diff = rawDiff;
        } else if (Math.abs(rawDiff + 360) < Math.abs(rawDiff) && Math.abs(rawDiff + 360) < Math.abs(rawDiff - 360)) {
            diff = rawDiff + 360;
        } else if (Math.abs(rawDiff - 360) < Math.abs(rawDiff) && Math.abs(rawDiff - 360) < Math.abs(rawDiff + 360)) {
            diff = rawDiff - 360;
        }

        double strength = 0.095;
        //double adjustment = diff / 90 * strength;
        double adjustment = 0;

        double leftPower = power * 0.75;
        double rightPower = power * 0.75;

        if (gamepad1.right_stick_x == 0) {
            if (diff > 0.2) {
                leftPower -= strength;
                rightPower += strength;
            } else if (diff < -0.2) {
                leftPower += strength;
                rightPower -= strength;
            }
        } else {
            leftPower += gamepad1.right_stick_x * 0.5;
            rightPower -= gamepad1.right_stick_x * 0.5;

            targetZ = currentZ;
        }


        telemetry.addData("left", leftPower);
        telemetry.addData("right", rightPower);

        telemetry.addData("diff", diff);

        telemetry.addData("t - c", targetZ - currentZ);
        telemetry.addData("t - c + 360", (targetZ - currentZ) + 360);
        telemetry.addData("t - c - 360", (targetZ - currentZ) - 360);

        telemetry.addData("target", targetZ);
        telemetry.addData("angle", imu.getAngularOrientation().firstAngle + 180);

        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    private Cone getCone() {
        float red = coneSensor.getNormalizedColors().red;
        float blue = coneSensor.getNormalizedColors().blue;
        float distance = coneSensor.getNormalizedColors().alpha;
        if (distance > CONE_DISTANCE_THRESHOLD) {
            if (red > blue) {
                return Cone.RED;
            } else {
                return Cone.BLUE;
            }
        } else {
            return Cone.NONE;
        }
    }
}