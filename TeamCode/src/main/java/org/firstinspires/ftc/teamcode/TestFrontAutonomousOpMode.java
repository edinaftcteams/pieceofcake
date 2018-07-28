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

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Test Front Glyph Mode", group="Robot Opmode")
@Disabled
public class TestFrontAutonomousOpMode extends BaseAutonomousOpMode
{
    @Override
    public void runOpMode() {
        RelicRecoveryVuMark column = RelicRecoveryVuMark.CENTER;
        int forwardRobot = 850; // red
        AllianceColor color = AllianceColor.Red;
        InitRobot();


        do {
            if (gamepad1.x){
                column = RelicRecoveryVuMark.LEFT;
                if (color == AllianceColor.Red){
                    forwardRobot = 1300;
                }
                else {
                    forwardRobot = 355;
                }
            }

            if (gamepad1.y){
                column = RelicRecoveryVuMark.CENTER;
                if (color == AllianceColor.Red){
                    forwardRobot = 825;
                }
                else {
                    forwardRobot = 900;
                }

            }

            if (gamepad1.b){
                column = RelicRecoveryVuMark.RIGHT;
                if (color == AllianceColor.Red){
                    forwardRobot = 380;
                }
                else {
                    forwardRobot = 1300;
                }

            }

            if (gamepad1.a){
                break;
            }

            if (gamepad1.dpad_left){
                color = AllianceColor.Blue;
                if (column == RelicRecoveryVuMark.LEFT){
                    forwardRobot = 355;
                }
                else if (column == RelicRecoveryVuMark.CENTER){
                    forwardRobot = 825;
                }
                else if (column == RelicRecoveryVuMark.RIGHT){
                    forwardRobot = 1300;
                }
            }

            if (gamepad1.dpad_right){
                color = AllianceColor.Red;
                if (column == RelicRecoveryVuMark.LEFT){
                    forwardRobot = 1300;
                }
                if (column == RelicRecoveryVuMark.CENTER){
                    forwardRobot = 850;
                }
                if (column == RelicRecoveryVuMark.RIGHT){
                    forwardRobot = 380;
                }
            }

            if (gamepad1.left_bumper){
                forwardRobot = forwardRobot - 25;
            }

            if (gamepad1.right_bumper){
                forwardRobot = forwardRobot + 25;
            }

            telemetry.addData("Column", "%s", column);
            telemetry.addData("Color", "%s", color);
            telemetry.addData("Forward Postiton", "%d", forwardRobot);
            telemetry.addData("Press the A button to exit", "");
            telemetry.update();
            sleep(150);

        } while (true);

        telemetry.addData("Ready to go", "");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Starting", "now");
        telemetry.update();

        // 450 + 650 + 850
        // 1100 + 1500
        // 1100 + 2100
        // the 2000 is the distance to travel after knocking the jewel.
        // change it to the match the distance we need to place the right glyph
        DropArmKnockLiftArmReposition(color, 1050);

        Rotate((int)-getAngle(), 0.05);
        MoveRobot(forwardRobot);

        ChangeWheelPowerLevel(0.25);

        if (color == AllianceColor.Red){
            // turn right
            Rotate(-81, 0.15);
        } else {
            // turn left
            Rotate(81, 0.15);
        }

        // drop glyph by opening claws
        MoveClaw(-400);

        sleep(200);

        // push it in
        MoveRobot(700);  // change this number if we don't push it in far enough

        // backup
        MoveRobot(-500); // change this number based on the above number

        // turn 180
        Rotate(-170, 0.25);

        ChangeWheelPowerLevel(0.40);
/*
        // move towards pile
        MoveRobot(1500); // change this number if don't move far enough into the pile

        // close claw
        MoveClaw(405);

        // backup to safe zone
        MoveRobot(-1500); // change this number if we don't move back far enough to the safe zone
*/
        telemetry.addData("Status","Finished");
        telemetry.update();
    }
}
// Done!