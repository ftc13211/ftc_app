package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;
import com.qualcomm.robotcore.hardware.LegacyModulePortDevice;
import com.qualcomm.robotcore.hardware.LegacyModule;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import android.graphics.Color;

public abstract class BaseAutonomous extends LinearOpMode {
    protected abstract boolean colorIsRed();
    protected RelicRecoveryVuMark pictograph;

    /* Declare OpMode members. */
    RobotHardware		 robot   = new RobotHardware();   // robot's hardware
    BNO055IMU imu;					// Additional Gyro device
    protected ElapsedTime	 runtime = new ElapsedTime(); //timer for actions

    static final int		LIFT_POSITION		   = 1120;

    private double RELIC_COUNTS_PER_INCH	   = 560/(3.1415);

    static final double	 COUNTS_PER_MOTOR_REV	= 1120 ;
    static final double	 DRIVE_GEAR_REDUCTION	= 0.67 ;	 // This is < 1.0 if geared UP
    static final double	 WHEEL_DIAMETER_INCHES   = 4.0 ;	 // For figuring circumference
    static final double	 COUNTS_PER_INCH		 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double	 DRIVE_SPEED			 = 0.2;	 // Nominal speed for better accuracy.
    static final double	 STRAFE_SPEED			= 0.2;
    static final double	 TURN_SPEED			  = 0.2;	 // Nominal half speed for better accuracy.
    static final double	 STRAFE_THRESHOLD		= 1.5;
    static final double	 HEADING_THRESHOLD	   = 1 ;	  // As tight as we can make it with an integer gyro
    static final double	 P_TURN_COEFF			= 0.08;	 // Larger is more responsive, but also less stable
    static final double	 P_DRIVE_COEFF		   = 0.08;	 // Larger is more responsive, but also less stable
    static final double	 P_STRAFE_COEFF		  = 0.08;

    ModernRoboticsI2cRangeSensor rangeSensor;

    //JEWEL DETECTOR VARIABLES
    final double scaleFactor = 255; //amplify color values for better detection
    final double BLUE = 240;
    final double THRESHOLD = 65;

    @Override
    public void runOpMode() {
        telemetry.addData("started", "autonomous");
        telemetry.update();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit		   = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit		   = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled	  = true;
        parameters.loggingTag		  = "IMU";

        telemetry.addData("started", "initializing gyro");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("started", "init gyro");
        telemetry.update();

        robot.init(hardwareMap);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData("Calibration Status", imu.isGyroCalibrated());	//
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!isStopRequested() && !imu.isGyroCalibrated())  {
            sleep(50);
            idle();
        }

        telemetry.addData("CalibrationStatus", imu.isGyroCalibrated());	//
        telemetry.update();

		/* ACTIVATE VUFORIA*/
        VuforiaDetector vuforiaDetector = new VuforiaDetector();
        vuforiaDetector.init(hardwareMap);

        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading: " + getGyroAngle());
            telemetry.update();
        }
        //RESET motors
        robot.pulleyMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.relicMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        robot.rightArm.setPosition(robot.RIGHT_ARM_REST_POSITION);
        robot.leftArm.setPosition(robot.LEFT_ARM_REST_POSITION);
        robot.topArm.setPosition(robot.TOP_ARM_GRAB_POSITION);

		/*HIT JEWEL*/
        //get jewel sensors and servos
        Servo jewelJoint1 = robot.jewelJoint1;
        Servo jewelJoint2 = robot.jewelJoint2;
        LynxI2cColorRangeSensor jewelSensor = robot.jewelSensor;

        //bring jewel arm out of the frame
        jewelJoint1.setPosition(0.1);
        //set second joint to neutral position
        jewelJoint2.setPosition(0.45);
        sleep(1500);
        int hitDirection = 0; //0 = none, 1 = back, 2 = forward
        for (int i = 0; i < 3; i++) { //try to read the jewel 3 times
            float hsvValues[] = {100f, 100f, 100f}; //begin with green which is neutral
            Color.RGBToHSV(
                    (int)(jewelSensor.red() * scaleFactor),
                    (int)(jewelSensor.green() * scaleFactor),
                    (int)(jewelSensor.blue() * scaleFactor),
                    hsvValues);
            //color sensor sees red
            telemetry.addData("hue", hsvValues[0]);
            if (hsvValues[0] < THRESHOLD || hsvValues[0] > 359 - THRESHOLD) {
                telemetry.addData("Detected", "Red");
                if (colorIsRed()) {
                    telemetry.addData("hitting", "blue jewel");
                    hitDirection = 1;
                    //jewelJoint2.setPosition(0.2); //swing to hit blue right jewel
                } else {
                    telemetry.addData("hitting", " red jewel");
                    hitDirection = 2;
                }
                telemetry.update();
                break;
            } else if (Math.abs(BLUE - hsvValues[0]) < THRESHOLD) { //color sensors see blue
                telemetry.addData("Detected", "Blue");
                telemetry.update();
                if (colorIsRed()) {
                    telemetry.addData("hitting", "blue jewel");
                    hitDirection = 2;
                } else {
                    telemetry.addData("hitting", "red jewel");
                    hitDirection = 1;
                }
                telemetry.update();
                break;
            } else {
                telemetry.addData("Detected", "Nothing");
                telemetry.update();
                sleep(50); //try again after time
            }
        }
        if (hitDirection > 0) {
            jewelJoint1.setPosition(0.15);
            sleep (500);
            jewelJoint2.setPosition (hitDirection == 1 ? 0.2 : 0.65);
            sleep (1000);
        }
        //retract arm
        jewelJoint1.setPosition(0.8);
        jewelJoint2.setPosition(1.0);


        //grab glyph and raise it out of the way
        robot.rightArm.setPosition(robot.RIGHT_ARM_GRAB_POSITION);
        robot.leftArm.setPosition(robot.LEFT_ARM_GRAB_POSITION);
        robot.topArm.setPosition(robot.TOP_ARM_REST_POSITION);

        sleep(1000);
		 /* GRAB GLYPH */

        //move baby arms out of the way
        robot.smallRightArm.setPosition(0.9);
        robot.smallLeftArm.setPosition(0.01);

        // DETECT PICTOGRAPH
        //detect pictograph configuration
        pictograph = vuforiaDetector.detect(telemetry, 3.0);

        // LIFT PULLEY
        robot.pulleyMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftPulley(LIFT_POSITION, 4.0); //low enough to hit jewel out of way
    }

    protected double getGyroAngle () {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle; //header
    }
    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed	  Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle	  Absolute Angle (in Degrees) relative to last gyro reset.
     *				   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *				   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int	 newLeftFrontTarget;
        int	 newLeftBackTarget;
        int	 newRightFrontTarget;
        int	 newRightBackTarget;
        int	 moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + moveCounts;
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + moveCounts;

            telemetry.update();

            //newLeftTarget = (int)COUNTS_PER_MOTOR_REV;
            //newRightTarget = (int)COUNTS_PER_MOTOR_REV;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontMotor.setPower(speed);
            robot.leftBackMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);
            robot.rightBackMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.leftFrontMotor.setPower(leftSpeed);
                robot.leftBackMotor.setPower(leftSpeed);
                robot.rightFrontMotor.setPower(rightSpeed);
                robot.rightBackMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Actual",  "%7d:%7d,%7d,%7d",	  robot.leftFrontMotor.getCurrentPosition(),
                        robot.leftBackMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition(),
                        robot.rightBackMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private double getDistance(
            ModernRoboticsI2cRangeSensor sensor, double timeoutS) {
        double value = sensor.getDistance(DistanceUnit.CM);
        while (value > 255.0) {
            if (runtime.seconds() < timeoutS) {
                telemetry.addData("Noise Timeout", "%5.2f", value);
                telemetry.update();
                return value;
            }
            value = sensor.getDistance(DistanceUnit.CM);
        }
        telemetry.addData("Distance", "%5.2f", value);
        return value;
    }
    //strafe to the right while maintaining heading
    public void gyroStrafe (double speed,
                            double distance,
                            double angle,
                            boolean useRight,
                            double timeoutS) {
        double  max;
        double  error;
        double  steer;
        double  backSpeed;
        double  frontSpeed;

        ModernRoboticsI2cRangeSensor rangeSensor = useRight ?
                hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range right"):
                hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range left");
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            // start motion
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontMotor.setPower(-speed);
            robot.leftBackMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);
            robot.rightBackMotor.setPower(-speed);

            // double finalError = Math.abs(rangeSensor.getDistance(DistanceUnit.CM) - distance);

            //go to the right instead of left if blue team
            if (!useRight) {
                speed = -speed;
            }

            runtime.reset();

            double value = getDistance(rangeSensor, timeoutS);
            // rangeSensor.getDistance(DistanceUnit.CM);

            //wait for rangesensor input to be calm
            //if moving towards wall, move in opposite direction
            //if (rangeSensor.getDistance(DistanceUnit.CM) > distance) {
            if (value > distance) {
                speed = -speed;
            }

            // keep looping while we are still active, and BOTH motors are running.
            while (
                    opModeIsActive() &&
                            runtime.seconds() < timeoutS &&
                            Math.abs(getDistance(rangeSensor, timeoutS) - distance) > STRAFE_THRESHOLD) {

                // double finalError = Math.abs(getDistance(rangeSensor, timeoutS) - distance);
                // finalError = Math.abs(rangeSensor.getDistance(DistanceUnit.CM) - distance);

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_STRAFE_COEFF);

                backSpeed = speed - steer;
                frontSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(backSpeed), Math.abs(frontSpeed));
                if (max > 1.0)
                {
                    backSpeed /= max;
                    frontSpeed /= max;
                }

                robot.leftFrontMotor.setPower(-frontSpeed);
                robot.leftBackMotor.setPower(backSpeed);
                robot.rightFrontMotor.setPower(frontSpeed);
                robot.rightBackMotor.setPower(-backSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Speed",   "%5.2f:%5.2f",  backSpeed, frontSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
        }
    }

    //strafe to the left
    public void gyroStrafeEncoder ( double speed,
                                    double distance,
                                    double angle) {
        int	 newLeftFrontTarget;
        int	 newLeftBackTarget;
        int	 newRightFrontTarget;
        int	 newRightBackTarget;
        int	 moveCounts;
        double  max;
        double  error;
        double  steer;
        double  backSpeed;
        double  frontSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() - moveCounts;
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() - moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            //no negatives since run to position
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontMotor.setPower(speed);
            robot.leftBackMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);
            robot.rightBackMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy())) {
                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_STRAFE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                backSpeed = speed - steer;
                frontSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(backSpeed), Math.abs(frontSpeed));
                if (max > 1.0)
                {
                    backSpeed /= max;
                    frontSpeed /= max;
                }

                robot.leftFrontMotor.setPower(frontSpeed);
                robot.leftBackMotor.setPower(backSpeed);
                robot.rightFrontMotor.setPower(frontSpeed);
                robot.rightBackMotor.setPower(backSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Speed",   "%5.2f:%5.2f",  backSpeed, frontSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.rightBackMotor.setPower(0);
            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void extendRelicEncoder(double speed, double distance, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newTarget = (int)(distance * RELIC_COUNTS_PER_INCH);
            robot.relicMotor.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.relicMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.relicMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    runtime.seconds() < timeoutS && robot.relicMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", newTarget);
                telemetry.addData("Path2",  "Running at %7d",
                        robot.relicMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.relicMotor.setPower(0.0);

            // Turn off RUN_TO_POSITION
            robot.relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle	  Absolute Angle (in Degrees) relative to last gyro reset.
     *				   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *				   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed	  Desired speed of turn.
     * @param angle	  Absolute Angle (in Degrees) relative to last gyro reset.
     *				   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *				   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed	 Desired speed of turn.
     * @param angle	 Absolute Angle (in Degrees) relative to last gyro reset.
     *				  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *				  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff	Proportional Gain coefficient
     * @return
     */
    protected boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFrontMotor.setPower(leftSpeed);
        robot.leftBackMotor.setPower(leftSpeed);
        robot.rightFrontMotor.setPower(rightSpeed);
        robot.rightBackMotor.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Heading", getGyroAngle());
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *		  +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntegratedZValue();
        robotError = targetAngle - getGyroAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftFrontMotor.setTargetPosition(newLeftFrontTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightFrontMotor.setTargetPosition(newRightFrontTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            setMotorRunMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            setMotorPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.leftBackMotor.isBusy()&& robot.rightFrontMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            setMotorPower(0.0);

            // Turn off RUN_TO_POSITION
            setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    //set all motor speeds
    private void setMotorPower (double speed) {
        robot.leftFrontMotor.setPower(Math.abs(speed));
        robot.leftBackMotor.setPower(Math.abs(speed));
        robot.rightFrontMotor.setPower(Math.abs(speed));
        robot.rightBackMotor.setPower(Math.abs(speed));
    }

    private void setMotorRunMode (DcMotor.RunMode runMode) {
        robot.leftFrontMotor.setMode(runMode);
        robot.leftBackMotor.setMode(runMode);
        robot.rightFrontMotor.setMode(runMode);
        robot.rightBackMotor.setMode(runMode);
    }

    protected void liftPulley (int pulleyTarget, double timeoutS) {
        //int pulleyTarget = isUp ? robot.pulleyMotor.getCurrentPosition() + LIFT_POSITION : robot.pulleyMotor.getCurrentPosition() - LIFT_POSITION;
        robot.pulleyMotor.setTargetPosition(pulleyTarget);
        robot.pulleyMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        robot.pulleyMotor.setPower(1.0);
        telemetry.addData("pulley values", "target: " + pulleyTarget + " actual: " + robot.pulleyMotor.getCurrentPosition() + " busy: " + robot.pulleyMotor.isBusy());
        while (robot.pulleyMotor.isBusy() && runtime.seconds() < timeoutS) {
            telemetry.addData("pulley values", "target: " + pulleyTarget + " actual: " + robot.pulleyMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.pulleyMotor.setPower(0.0);
        robot.pulleyMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}