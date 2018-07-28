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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.CompassSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@Autonomous(name = "Blue Back Glyph Mode", group = "Robot Opmode")
public class BlueBackWithGlyphAutonomousOpMode extends BaseAutonomousOpMode {
    @Override
    public void runOpMode() {

        InitRobot();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Starting", "now");
        telemetry.update();

        LoadGlyphPosition();

        DropArmKnockLiftArmReposition(AllianceColor.Blue, 1350);

        Rotate((int)-getAngle(), .05);
        if (GetGlyphPosition() == RelicRecoveryVuMark.LEFT) {
            SlideRobot(-385); // change this number to match left position
        } else if (GetGlyphPosition() == RelicRecoveryVuMark.RIGHT) {
            SlideRobot(-2205); // change this number to match right position
        } else {
            SlideRobot(-1265); // change this number to match center position
        }

        ChangeWheelPowerLevel(0.25);

        // push it in
        MoveClaw(-400);

        sleep(200);

        MoveRobot(600);  // change this number if we don't push it in far enough

        MoveRobot(-525);

        // Move to the center and rotate so we face the pile
        if (GetGlyphPosition() == RelicRecoveryVuMark.CENTER) {
            SlideRobot(-700); // change this number to match right position
        }

        if (GetGlyphPosition() == RelicRecoveryVuMark.LEFT) {
            SlideRobot(-1295); // change this number to match left position
        }

        Rotate(-135, .25);
/*
        ChangeWheelPowerLevel(0.40);

        MoveRobot(1100);

        MoveClaw(395);

        MoveRobot(-1100);
*/
        telemetry.addData("Status", "Finished");
        telemetry.update();
    }
}
// Done!