/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.graphics.Matrix;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Color Sensor Test", group="Test")
public class JewelSensorTest extends LinearOpMode {

    RobotHardware		 robot   = new RobotHardware();   // Use a Pushbot's hardware
    Servo jewelJoint1;
    Servo jewelJoint2;
    LynxI2cColorRangeSensor jewelSensor;

    final double scaleFactor = 255; //amplify color values for better detection

    final double BLUE = 240;
    final double THRESHOLD = 65;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        jewelJoint1 = robot.jewelJoint1;
        jewelJoint2 = robot.jewelJoint2;
        jewelSensor = robot.jewelSensor;

        //set second joint to neutral position
        jewelJoint2.setPosition(0.45);

        //send arm all the way down
        jewelJoint1.setPosition(0.1);
        sleep (3000);

        waitForStart();
        while(opModeIsActive()) {
            float hsvValues[] = {100f, 100f, 100f}; //begin with green which is neutral
            Color.RGBToHSV(
                    (int)(jewelSensor.red() * scaleFactor),
                    (int)(jewelSensor.green() * scaleFactor),
                    (int)(jewelSensor.blue() * scaleFactor),
                    hsvValues);
            telemetry.addData("hue", hsvValues[0]);
            if (hsvValues[0] < THRESHOLD || hsvValues[0] > 359 - THRESHOLD) {
                telemetry.addData("Detected", "Red");
                telemetry.update();
            } else if (Math.abs(BLUE - hsvValues[0]) < THRESHOLD) {
                telemetry.addData("Detected", "Blue");
                telemetry.update();
            } else {
                telemetry.addData("Detected", "Nothing");
                telemetry.update();
            }
        }
    }
}