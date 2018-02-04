package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Red Right Range", group = "Range")
public class RangeRedRight extends BaseAutonomous{

    // todo: write your code here
    protected boolean colorIsRed() {
        return true;
    }

    public void runOpMode () {
        super.runOpMode();
        encoderDrive (DRIVE_SPEED, 26.0, 28.0, 5.0);
        //GRAB RELIC
		/*double relicDirection = 52.0;
		gyroStrafe (STRAFE_SPEED, 37.5, 0.0, true);
		gyroTurn (0.4, relicDirection);
		robot.relicJoint1.setPosition(0.87);
		robot.relicJoint2.setPosition(1.0); //open
		sleep (1500);
		extendRelicEncoder (0.5, 10.0, 3.0);
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
		gyroTurn(0.4, 0.0);*/
        //PLACE GLYPH
		/*double[] strafeDistances = {50, 69, 87};
		double strafeDistance;
		switch (pictograph) {
			case LEFT:
				strafeDistance = strafeDistances[2];
				break;
			case CENTER:
				strafeDistance = strafeDistances[1];
				break;
			case RIGHT:
				strafeDistance = strafeDistances[0];
				break;
			default:
				strafeDistance = strafeDistances[0];
				break;
		}
		double cryptoboxDirection = 0.0;
		gyroStrafe(STRAFE_SPEED, strafeDistance, cryptoboxDirection, true, 5.0);
		gyroDrive(DRIVE_SPEED, 7.0, cryptoboxDirection);
		robot.rightArm.setPosition(0.9);
		robot.leftArm.setPosition(0.1);
		robot.topArm.setPosition(0.1); //move it out of the way for strafing to grab relic
		liftPulley(0, 5.0);
		gyroDrive(DRIVE_SPEED, -3.0, cryptoboxDirection);
		gyroDrive(DRIVE_SPEED, 3.0, cryptoboxDirection);
		gyroDrive(DRIVE_SPEED, -2.5, cryptoboxDirection);*/
        double[] strafeDistances = {7.3, 19, 26};
        double strafeDistance;
        switch (pictograph) {
            case LEFT:
                strafeDistance = strafeDistances[2];
                break;
            case CENTER:
                strafeDistance = strafeDistances[1];
                break;
            case RIGHT:
                strafeDistance = strafeDistances[0];
                break;
            default:
                strafeDistance = strafeDistances[0];
                break;
        }
        double cryptoboxDirection = 0.0;
        gyroStrafeEncoder(STRAFE_SPEED, strafeDistance, cryptoboxDirection);
        gyroDrive(DRIVE_SPEED, 7.0, cryptoboxDirection);
        robot.rightArm.setPosition(0.95);
        robot.leftArm.setPosition(0.05);
        robot.topArm.setPosition(0.1); //move it out of the way for strafing to grab relic
        liftPulley(0, 5.0);
        gyroDrive(DRIVE_SPEED, -3.0, cryptoboxDirection);
        gyroDrive(DRIVE_SPEED, 3.0, cryptoboxDirection);
        gyroDrive(DRIVE_SPEED, -2.5, cryptoboxDirection);

        //POSITION FOR RELIC
        double relicAngle = 52.0;
        gyroStrafeEncoder (STRAFE_SPEED, -strafeDistance - 6.5, 0.0);
        telemetry.addData("status", "turning");
        telemetry.update();
        gyroTurn (TURN_SPEED, relicAngle);
        gyroDrive (DRIVE_SPEED, 8.0, relicAngle);
        gyroDrive (DRIVE_SPEED, -2.5, relicAngle); //make sure not touching glyph
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
		double relicAngle = 52.0;
		//gyroStrafe (STRAFE_SPEED, 34, 0.0);
		gyroStrafe (STRAFE_SPEED, 37.5, 0.0);
		//gyroDrive (DRIVE_SPEED, 8.0, 0.0);
		gyroDrive (DRIVE_SPEED, 7.0, 0.0);
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