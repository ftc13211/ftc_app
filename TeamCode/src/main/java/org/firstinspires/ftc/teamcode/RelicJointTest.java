package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Relic Test", group = "Test")

public class RelicJointTest extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    protected ElapsedTime	 runtime = new ElapsedTime(); //timer for actions
    // todo: write your code here
    public void runOpMode () {
        robot.init(hardwareMap);
        double joint1TimeOut = 2.0;
        //double joint2TimeOut = 1.78;
        robot.relicJoint1.setPosition(0.1);
        //robot.relicJoint2.setPosition(1.0);
        telemetry.addData("relic joint 1", "running");
        telemetry.update();
        runtime.reset();
        while (runtime.seconds() < joint1TimeOut) {
			/*if (runtime.seconds() > joint2TimeOut) {
				robot.relicJoint2.setPosition(0.5);
			}*/
        }
        robot.relicJoint1.setPosition(0.5);
    }
}