package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Left Range", group = "Range")
public class RangeBlueLeft extends BaseAutonomous {

    // todo: write your code here
    protected boolean colorIsRed() {
        return false;
    }

    public void runOpMode () {
        super.runOpMode();
        double cryptoboxDirection = 180.0;
        double relicDirection = -47;
        encoderDrive (DRIVE_SPEED, -24.0, -24.0, 5.0);
        //GRAB RELIC
		/*gyroStrafe (STRAFE_SPEED, 24, 0.0, true);
		gyroTurn (0.4, relicDirection); //faster turn
		robot.relicJoint1.setPosition(0.87);
		robot.relicJoint2.setPosition(1.0); //open
		sleep (1500);
		extendRelicEncoder (0.3, 5.0, 3.0);
		robot.relicJoint1.setPosition(1.0);
		sleep (1000);
		robot.relicJoint2.setPosition(0.0);
		sleep (1000);
		//gyroStrafeEncoder (STRAFE_SPEED, 3.0, relicAngle);
		extendRelicEncoder (0.3, 0.0, 3.0);
		double relicArmTarget = 0.8;
		int i = 0;
		while (i < 100) {
			i++;
			relicArmTarget -= 0.006;
			robot.relicJoint1.setPosition(relicArmTarget);
			sleep (6);
		}
		encoderDrive (DRIVE_SPEED, -3.0, -3.0, 4.0); //make space for turning*/
        //PLACE GLYPH
        gyroTurn (0.4, cryptoboxDirection);
        //encoderDrive (DRIVE_SPEED, -2.0, -2.0, 4.0); //make space for strafe after relic
        double[] strafeDistances = {7.3, 19, 27};
        double strafeDistance;
        switch (pictograph) {
            case LEFT:
                strafeDistance = strafeDistances[0];
                break;
            case CENTER:
                strafeDistance = strafeDistances[1];
                break;
            case RIGHT:
                strafeDistance = strafeDistances[2];
                break;
            default:
                strafeDistance = strafeDistances[0];
                break;
        }
        //gyroStrafe(STRAFE_SPEED, strafeDistance, cryptoboxDirection, false, 5.0);
        gyroStrafeEncoder(STRAFE_SPEED, -strafeDistance, cryptoboxDirection);
        gyroDrive(DRIVE_SPEED, 6.0, cryptoboxDirection);
        robot.rightArm.setPosition(0.9);
        robot.leftArm.setPosition(0.1);
        robot.topArm.setPosition(0.1); //move it out of the way for strafing to grab relic
        liftPulley(0, 5.0);
        //POSITION TO RELIC
        double relicAngle = -30.8;
        gyroDrive(DRIVE_SPEED, -5.0, cryptoboxDirection);
        //gyroStrafe (STRAFE_SPEED, 35, 180.0, false, 5.0);
        gyroStrafeEncoder (STRAFE_SPEED, strafeDistance + 2.5, 180.0);
        gyroTurn (0.4, relicAngle); //turn while extend claw
        gyroTurn (TURN_SPEED, relicAngle); //correction
        gyroDrive (DRIVE_SPEED, -4.0, relicAngle);
        double joint1TimeOut = 2.0;
        double joint2TimeOut = 1.78;
        runtime.reset();
        robot.relicJoint1.setPosition(0.1);
        robot.relicJoint2.setPosition(1.0);
        //extendRelicEncoder (0.6, 3.0, 2.0);
        while (runtime.seconds() < joint1TimeOut && !isStopRequested()) {
            if (runtime.seconds() > joint2TimeOut) {
                robot.relicJoint2.setPosition(0.5);
            }
        }
        robot.relicJoint1.setPosition(0.5);
        robot.relicJoint2.setPosition(0.5);
    }

	/*public void runOpMode() {
		super.runOpMode();
		double relicAngle = -47;
		//gyroStrafe (STRAFE_SPEED, 34, 0.0);
		gyroStrafe (STRAFE_SPEED, 24, 180.0);
		//gyroDrive (DRIVE_SPEED, 8.0, 0.0);
		//gyroDrive (DRIVE_SPEED, 2.0, 0.0);
		gyroTurn (TURN_SPEED, relicAngle);
		robot.relicJoint1.setPosition(0.87);
		robot.relicJoint2.setPosition(1.0); //open
		sleep (1500);
		extendRelicEncoder (0.3, 4.0, 3.0);
		robot.relicJoint1.setPosition(1.0);
		sleep (1000);
		robot.relicJoint2.setPosition(0.0);
		sleep (1000);
		//gyroStrafeEncoder (STRAFE_SPEED, 3.0, relicAngle);
		extendRelicEncoder (0.3, 0.0, 3.0);
		double relicArmTarget = 0.8;
		int i = 0;
		while (i < 100) {
			i++;
			relicArmTarget -= 0.006;
			robot.relicJoint1.setPosition(relicArmTarget);
			sleep (15);
		}
	}*/
}