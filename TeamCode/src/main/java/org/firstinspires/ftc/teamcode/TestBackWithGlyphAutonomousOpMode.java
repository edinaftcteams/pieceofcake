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

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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


@Autonomous(name = "Test Back Glyph Mode", group = "Robot Opmode")
@Disabled
public class TestBackWithGlyphAutonomousOpMode extends BaseAutonomousOpMode {
    @Override
    public void runOpMode() {
        RelicRecoveryVuMark column = RelicRecoveryVuMark.CENTER;
        int SlidePosition = -1265;
        AllianceColor color = AllianceColor.Blue;

        InitRobot();

        do {
            if (gamepad1.x) {
                column = RelicRecoveryVuMark.LEFT;
                if (color == AllianceColor.Blue) {
                    SlidePosition = -360;
                }
                else if (color == AllianceColor.Red) {
                    SlidePosition = 2350;
                }
            }
            else if (gamepad1.y) {
                column = RelicRecoveryVuMark.CENTER;
                if (color == AllianceColor.Blue) {
                    SlidePosition = -1265;
                }
                else if (color == AllianceColor.Red) {
                    SlidePosition = 1450;
                }
            }
            else if (gamepad1.b) {
                column = RelicRecoveryVuMark.RIGHT;
                if (color == AllianceColor.Blue) {
                    SlidePosition = -2180;
                }
                else if (color == AllianceColor.Red) {
                    SlidePosition = 425;
                }
            }
            else if (gamepad1.a) {
                break;
            }
            //Red
            if (gamepad1.dpad_right) {
                color = AllianceColor.Red;
                if (column == RelicRecoveryVuMark.LEFT) {
                    SlidePosition = 2350;
                }
                else if (column == RelicRecoveryVuMark.RIGHT) {
                    SlidePosition = 425;
                }
                else if (column == RelicRecoveryVuMark.CENTER) {
                    SlidePosition = 1450;
                }
            }
            //Blue
            if (gamepad1.dpad_left) {
                color = AllianceColor.Blue;
                if (column == RelicRecoveryVuMark.LEFT) {
                    SlidePosition = -360;
                }
                else if (column == RelicRecoveryVuMark.RIGHT) {
                    SlidePosition = -2180;
                }
                else if (column == RelicRecoveryVuMark.CENTER) {
                    SlidePosition = -1265;
                }
            }
            if (gamepad1.right_bumper) {
                SlidePosition = SlidePosition + 25;
            }
            if (gamepad1.left_bumper) {
                SlidePosition = SlidePosition - 25;
            }
            telemetry.addData("Column", "%s", column);
            telemetry.addData("Color", "%s", color);
            telemetry.addData("New Slide Position", "%s", SlidePosition);
            telemetry.addData("Press A to get out", "");
            telemetry.update();
            sleep(150);
        } while (true);

        telemetry.addData("Ready to go", "");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Starting", "now");
        telemetry.update();

        DropArmKnockLiftArmReposition(color, 1350);

        Rotate((int)-getAngle(), .05);

        SlideRobot(SlidePosition);

        ChangeWheelPowerLevel(0.25);

        // push it in
        MoveClaw(-400);

        sleep(200);

        MoveRobot(600);  // change this number if we don't push it in far enough

        MoveRobot(-525);

        if (color == AllianceColor.Blue) {
            // Move to the center and rotate so we face the pile
            if (column == RelicRecoveryVuMark.CENTER) {
                SlideRobot(-700); // change this number to match right position
            }

            if (column == RelicRecoveryVuMark.LEFT) {
                SlideRobot(-1495); // change this number to match left position
            }


            Rotate(-125, .25);
        } else {
            // Move to the center and rotate so we face the pile
            if (column == RelicRecoveryVuMark.CENTER) {
                SlideRobot(700); // change this number to match right position
            }

            if (column == RelicRecoveryVuMark.RIGHT) {
                SlideRobot(1595); // change this number to match left position
            }


            Rotate(125, .25);

        }
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