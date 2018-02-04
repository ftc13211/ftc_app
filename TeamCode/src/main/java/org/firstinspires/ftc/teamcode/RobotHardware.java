package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Created by naluz on 9/29/2017.
 */

public class RobotHardware {
    //define member constants used by both teleop and autonomous
    public double RIGHT_ARM_REST_POSITION   = 0.7;
    public double LEFT_ARM_REST_POSITION	= 0.3;
    public double RIGHT_ARM_GRAB_POSITION   = 0.5;
    public double LEFT_ARM_GRAB_POSITION	= 0.5;
    public double TOP_ARM_GRAB_POSITION	 = 0.1;
    public double TOP_ARM_REST_POSITION	 = 0.7;

    public double RELIC_GRAB_POSITION	   = 0.1;
    public double RELIC_REST_POSITION	   = 3.0;

    //define member motors
    public DcMotor led			 = null;

    public DcMotor leftFrontMotor  = null;
    public DcMotor leftBackMotor   = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor  = null;

    public DcMotor pulleyMotor	 = null;

    public DcMotor relicMotor	   = null;
    public Servo relicJoint1		= null;
    public Servo relicJoint2		= null;

    //glyph arm servos
    public Servo rightArm		  = null;
    public Servo smallRightArm	  = null;
    public Servo topArm			 = null;
    public Servo leftArm		   = null;
    public Servo smallLeftArm	   = null;
    public Servo jewelJoint1		  = null;
    public Servo jewelJoint2		= null;
    public LynxI2cColorRangeSensor jewelSensor = null;

    HardwareMap hwMap   = null;

    public RobotHardware () {

    }

    public void init (HardwareMap ahwMap) {
        //save reference to hardware map
        hwMap = ahwMap;

        led = hwMap.get (DcMotor.class, "led");

        leftFrontMotor  = hwMap.get(DcMotor.class, "lf motor");
        leftBackMotor   = hwMap.get(DcMotor.class, "lb motor");
        rightFrontMotor = hwMap.get(DcMotor.class, "rf motor");
        rightBackMotor  = hwMap.get(DcMotor.class, "rb motor");

        pulleyMotor	 = hwMap.get(DcMotor.class, "pulley motor");

        rightArm		= hwMap.get(Servo.class, "right arm");
        smallRightArm   = hwMap.get(Servo.class, "small right arm");
        leftArm		 = hwMap.get(Servo.class, "left arm");
        smallLeftArm	= hwMap.get(Servo.class, "small left arm");
        topArm		  = hwMap.get(Servo.class, "top arm");

        jewelJoint1		= hwMap.get(Servo.class, "jewel joint 1");
        jewelJoint2	 = hwMap.get(Servo.class, "jewel joint 2");
        jewelSensor	 = hwMap.get(LynxI2cColorRangeSensor.class, "jewel sensor");

        relicMotor	  = hwMap.get(DcMotor.class, "relic motor");
        relicMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        relicJoint1	 = hwMap.get(Servo.class, "relic joint 1");
        relicJoint2	 = hwMap.get(Servo.class, "relic joint 2");

        //set motor directions
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        //set all motors to zero power
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
}
