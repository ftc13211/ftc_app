package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Left Encoders", group="Encoders")

public class EncoderRedLeft extends BaseAutonomous {

    // todo: write your code here
    protected boolean colorIsRed() {
        return true;
    }

    public void runOpMode() {
        super.runOpMode();
        encoderDrive(DRIVE_SPEED, 24.0, 24.0, 5.0);
        double[] distances = {9.0, 16.0, 23.0}; //{30, 37, 44}; //based on blue team in inches
        double moveDistance;
        switch(pictograph) {
            case LEFT:
                moveDistance = distances[2];
                break;
            case CENTER:
                moveDistance = distances[1];
                break;
            case RIGHT:
                moveDistance = distances[0];
                break;
            default:
                moveDistance = distances[0];
                break;
        }

        double cryptoboxDirection = -90.0;

        gyroDrive (DRIVE_SPEED, moveDistance, 0.0);
        gyroTurn (TURN_SPEED, cryptoboxDirection);
        gyroTurn (TURN_SPEED, cryptoboxDirection);

        gyroDrive(DRIVE_SPEED, 8.0, cryptoboxDirection);
        robot.rightArm.setPosition(0.9);
        robot.leftArm.setPosition(0.1);
        robot.topArm.setPosition(0.1); //move it out of the way for strafing to grab relic
        liftPulley(0, 5.0);
        gyroDrive (DRIVE_SPEED, 2.0, cryptoboxDirection);
        gyroDrive(DRIVE_SPEED, -7.0, cryptoboxDirection);
        gyroTurn(TURN_SPEED, (cryptoboxDirection + 180) % 180);
    }
}
