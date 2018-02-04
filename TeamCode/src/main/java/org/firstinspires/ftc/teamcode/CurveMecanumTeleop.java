package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by naluz on 9/29/2017.
 * Modified by dkudriavtsev on 1/28/2017.
 */

@TeleOp(name="Curve Mecanum Teleop", group="Robot")
public class CurveMecanumTeleop extends OpMode {
    RobotHardware robot = new RobotHardware();
    private ElapsedTime	 runtime = new ElapsedTime(); //timer to incrementally move servos
    private double lastTime;
    private double deltaTime;

    private boolean isSlow = false;

    private boolean grabState = false; //false - rest, true - grab
    private double targetRight = 0.7;
    private double targetLeft = 0.3;
    private double targetTop = robot.TOP_ARM_REST_POSITION;

    private boolean previousRightTrigger = false;
    private boolean previousLeftTrigger = false;

    private boolean s_grabState = false; //false - rest, true - grab
    private double s_targetRight = 0.9;
    private double s_targetLeft = 0.01;
    private boolean previousRightBumper = false;
    private boolean previousLeftBumper = false;

    private boolean previousB = false;

    private boolean previousDUp = false;
    private boolean previousDDown = false;
    private int pulleyPosition = 0;
    public double[] PULLEY_POSITIONS	= { 0, 2, 6, 12, 16};
    private double relicTarget = 0;
    public double PULLEY_COUNTS_PER_INCH	= 1120/3.1415;

    private double RELIC_COUNTS_PER_INCH	   = 560/(3.1415);
    private double RELIC_MAX_POS			   = 35 * RELIC_COUNTS_PER_INCH;
    private double RELIC_MIN_POS			   = 0;
    private double relicArmTarget = 0.0;
    private double relicArmSpeed = 0.2; //per second
    private boolean previousA2 = false;
    private boolean relicGrabState = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    /*
  * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
  */
    @Override
    public void init_loop() {
        //RESET PULLEY ENCODER FOR TESTING, REMOVE LATER
        //robot.pulleyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pulleyMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.pulleyMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
		/*
		STORE DELTA TIME
		*/
        deltaTime = runtime.seconds() - lastTime;
        lastTime = runtime.seconds();

		/*
		MOVEMENT
		*/
		/*
		double strafeRight = gamepad1.left_stick_x;
		double forward = -gamepad1.left_stick_y;
		double turnRight = gamepad1.right_stick_x;
		*/

        double strafeRight = java.lang.Math.pow(gamepad1.left_stick_x, 3);
        double forward = -java.lang.Math.pow(gamepad1.left_stick_y, 3);
        double turnRight = java.lang.Math.pow(gamepad1.right_stick_x, 3);

        telemetry.addData("input", "strafe " + strafeRight + " forward " + forward + " turn " + turnRight);
        double factor = isSlow ? 0.4 : 0.8;
        robot.rightFrontMotor.setPower(Math.max(Math.min(-strafeRight + forward - turnRight, 1.0), -1.0) * factor);
        robot.leftFrontMotor.setPower(Math.max(Math.min(strafeRight + forward + turnRight, 1.0), -1.0) * factor);
        robot.rightBackMotor.setPower(Math.max(Math.min(strafeRight + forward - turnRight, 1.0), -1.0) * factor);
        robot.leftBackMotor.setPower(Math.max(Math.min(-strafeRight + forward + turnRight, 1.0), -1.0) * factor);

        boolean b = gamepad1.a;
        if (b && !previousB) {
            isSlow = !isSlow;
        }
        telemetry.addData("Mode", isSlow ? "SLOW" : "FAST");
        previousB = b;

		/*
		BIG ARMS
		*/
        boolean rightTrigger = (gamepad1.right_trigger > 0.1);
        boolean leftTrigger = (gamepad1.left_trigger > 0.1);

        if (rightTrigger && !previousRightTrigger) {
            grabState = !grabState;
            targetRight = grabState ? robot.RIGHT_ARM_GRAB_POSITION : robot.RIGHT_ARM_REST_POSITION;
            targetLeft = grabState ? robot.LEFT_ARM_GRAB_POSITION : robot.LEFT_ARM_REST_POSITION;
            targetTop = grabState ? robot.TOP_ARM_GRAB_POSITION : robot.TOP_ARM_REST_POSITION;
        } else if (leftTrigger && !previousLeftTrigger) { //incrementally let go
            targetRight += 0.05;
            targetLeft -= 0.05;
            targetTop -= 0.05;
        }

        robot.rightArm.setPosition(targetRight);
        robot.leftArm.setPosition(targetLeft);
        robot.topArm.setPosition(targetTop);

        previousRightTrigger = rightTrigger;
        previousLeftTrigger = leftTrigger;

		/*
		SMALL ARMS
		*/
        boolean rightBumper = gamepad1.right_bumper;
        boolean leftBumper = gamepad1.left_bumper;

        if (rightBumper && !previousRightBumper) {
            s_grabState = !s_grabState;
            s_targetRight = s_grabState ? 0.1 : 0.9;
            s_targetLeft = s_grabState ? 0.9 : 0.01;
        } else if (leftBumper && !previousLeftBumper) { //incrementally let go
            s_targetRight += 0.05;
            s_targetLeft -= 0.05;
        }
        telemetry.addData("s_targets", s_targetLeft + " " + s_targetRight);
        robot.smallRightArm.setPosition(s_targetRight);
        robot.smallLeftArm.setPosition(s_targetLeft);

        previousRightBumper = rightBumper;
        previousLeftBumper = leftBumper;

		/*
		TOP ARM
		*/

        if (gamepad1.y) {
            targetTop = robot.TOP_ARM_REST_POSITION;
        }

		/*
		PULLEY
		*/
        boolean dUp = gamepad1.dpad_up;
        boolean dDown = gamepad1.dpad_down;
		/*if (dUp) {
			robot.pulleyMotor.setPower(1.0);
		} else if (dDown) {
			robot.pulleyMotor.setPower(-1.0);
		} else {
			robot.pulleyMotor.setPower(0.0);
		}*/

        //if (gamepad1.right_bumper && !previousRightBumper) {
        if (dUp && !previousDUp) {
            pulleyPosition++;
            if (pulleyPosition >= PULLEY_POSITIONS.length) {
                pulleyPosition = PULLEY_POSITIONS.length -1;
            }
        }
        //} else if (gamepad1.left_bumper && !previousLeftBumper) {
        else if (dDown && !previousDDown) {
            pulleyPosition--;
            if (pulleyPosition < 0) {
                pulleyPosition = 0;
            }
        }

        int pulleyTarget = (int)(PULLEY_POSITIONS[pulleyPosition] * PULLEY_COUNTS_PER_INCH);
        robot.pulleyMotor.setTargetPosition(pulleyTarget);
        if (Math.abs(robot.pulleyMotor.getCurrentPosition() - pulleyTarget) < 5) {
            robot.pulleyMotor.setPower(0.0);
        } else if (robot.pulleyMotor.getCurrentPosition() > pulleyTarget) {
            robot.pulleyMotor.setPower(-1.0);
        } else {
            robot.pulleyMotor.setPower(1.0);
        }

        previousDUp = dUp;
        previousDDown = dDown;

		/*
		RELIC ARM
		*/
        telemetry.addData("relic motor position", robot.relicMotor.getCurrentPosition() + " min: " + RELIC_MIN_POS + " max: " + RELIC_MAX_POS);
        telemetry.addData("relic input", gamepad2.left_bumper + " " + gamepad2.right_bumper);
        if (gamepad2.right_trigger > 0.1 && robot.relicMotor.getCurrentPosition() < RELIC_MAX_POS) {
            robot.relicMotor.setPower(1.0);
        } else if (gamepad2.left_trigger > 0.1 && robot.relicMotor.getCurrentPosition() > RELIC_MIN_POS) {
            robot.relicMotor.setPower(-1.0);
        } else if (gamepad2.right_bumper && robot.relicMotor.getCurrentPosition() > RELIC_MIN_POS) {
            robot.relicMotor.setPower(0.4);
        } else if (gamepad2.left_bumper && robot.relicMotor.getCurrentPosition() > RELIC_MIN_POS) {
            robot.relicMotor.setPower(-0.4);
        } else {
            robot.relicMotor.setPower(0.0);
        }

		/*if (gamepad2.right_bumper) {
			robot.relicJoint1.setPosition(-0.2);
		} else if (gamepad2.left_bumper) {
			robot.relicJoint1.setPosition(0.2);
		} else {
			robot.relicJoint1.setPosition(0.4);
		}*/

        //RELIC CLAW RUBBER BAND
		/*
		boolean a2 = gamepad2.a;
		boolean b2 = gamepad2.b;

		if (a2) {
			relicArmTarget -= relicArmSpeed * deltaTime;
		} else if (b2) {
			relicArmTarget += relicArmSpeed * deltaTime;
		}
		if (gamepad2.x) {
			relicArmTarget = 0.7;
			robot.relicJoint2.setPosition(0.7);
		} else if (gamepad2.y) {
			relicArmTarget = 0.2;
			robot.relicJoint2.setPosition(0.2);
		}
		relicArmTarget = Range.clip(relicArmTarget, 0, 1);
		robot.relicJoint1.setPosition(relicArmTarget);
		telemetry.addData("relic joint 1", robot.relicJoint1.getPosition());
		telemetry.addData("relic target", relicArmTarget);

		if (gamepad2.left_bumper) {
			robot.relicJoint2.setPosition(0.0);
		} else if (gamepad2.right_bumper) {
			robot.relicJoint2.setPosition(1.0);
		} else {
			robot.relicJoint2.setPosition(0.5);
		}*/

		/*if (gamepad2.b) {
			robot.relicJoint2.setPosition(1.0);
		} else if (gamepad2.a) {
			robot.relicJoint2.setPosition(0.5);
		}
		if (gamepad2.left_bumper) { //let go
			robot.relicJoint1.setPosition(0.5);
		} else if (gamepad2.right_bumper) { //grab
			robot.relicJoint1.setPosition(0.0);
		}*/

        //RUBBER BAND SINGLE MOVEMENT
		/*double y1 = gamepad2.left_stick_y;
		double y2 = gamepad2.right_stick_y;
		boolean a2 = gamepad2.a;

		if (a2 && !previousA2) {
			relicGrabState = !relicGrabState;
		}
		telemetry.addData("relic grab state", relicGrabState);
		robot.relicJoint2.setPosition(relicGrabState ? 0.0 : 1.0);

		if (gamepad2.b) {
			robot.relicJoint1.setPosition(0.7);
		} else if (gamepad2.x) {
			robot.relicJoint1.setPosition(0.95);
		} else if (Math.abs(y2) > 0.1) {
			robot.relicJoint1.setPosition(robot.relicJoint1.getPosition() + (0.8 * deltaTime * y2));
		} else if (Math.abs(y1) > 0.1) {
			robot.relicJoint1.setPosition(robot.relicJoint1.getPosition() + (0.3 * deltaTime * y1));
		}

		previousA2 = a2;*/

        //RUBBER BAND CONTINUOUS MOVEMENT
        double y1 = -gamepad2.left_stick_y;
        y1 = Math.abs(y1) > 0.1 ? y1/2.0 : 0.0;
        robot.relicJoint1.setPosition (0.5 + y1);
        if (gamepad2.y) {
            robot.relicJoint2.setPosition(1.0);
        } else if (gamepad2.a) {
            robot.relicJoint2.setPosition(0.0);
        } else {
            robot.relicJoint2.setPosition(0.5);
        }
		/*
		SET SERVOS
		*/
        robot.jewelJoint1.setPosition(0.7);
        robot.jewelJoint2.setPosition(0.5);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}