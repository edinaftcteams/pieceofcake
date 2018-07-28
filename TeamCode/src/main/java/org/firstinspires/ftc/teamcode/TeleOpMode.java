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

import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorConstantPowerCommand;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.R.attr.left;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpMode Mode", group="Robot Opmode")
public class TeleOpMode extends OpMode {

    private PieceOfCakeRobot robot = new PieceOfCakeRobot();
    private float PreviousXlimit = 0;
    private float PreviousYlimit = 0;

    @Override
    public void init(){
        double currentPosition = 0.5;

        //Reset motor encoders
        robot.init(hardwareMap);

        robot.GetClawR().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetClawR().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.GetClawL().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetClawL().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.GetLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLift().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.GetLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLeftFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.GetLeftFront().setDirection(DcMotor.Direction.REVERSE);
        robot.GetLeftBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLeftBack().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.GetLeftBack().setDirection(DcMotor.Direction.REVERSE);

        robot.GetRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetRightFront().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.GetRightBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetRightBack().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.GetLeftServo().setPosition(0);
        robot.GetRightServo().setPosition(0);
    }

    @Override
    public void loop(){
        ProcessClaw();
        LiftControl();
        DriveControl();
        SlideControl();
        PowerChange();

        telemetry.addData("Left Front Position", "%d", robot.GetLeftFront().getCurrentPosition());
        telemetry.addData("Right Front Position", "%d", robot.GetRightFront().getCurrentPosition());
        telemetry.addData("Left Back Position", "%d", robot.GetLeftBack().getCurrentPosition());
        telemetry.addData("Right Back Position", "%d", robot.GetRightBack().getCurrentPosition());
        telemetry.addData("Left Clamp Position", "%d", robot.GetClawL().getCurrentPosition());
        telemetry.addData("Right Clamp Position", "%d", robot.GetClawR().getCurrentPosition());
        telemetry.addData("Slide Position", "%d", robot.GetSlide().getCurrentPosition());

        telemetry.addData("PowerPercentage", "%f", robot.GetPowerPercentage());

        telemetry.addData("Angle", "%f", Math.toDegrees(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x)));
        telemetry.addData("Distance", "%f", Math.sqrt((gamepad1.left_stick_y * gamepad1.left_stick_y) +
                (gamepad1.left_stick_x * gamepad1.left_stick_x)));

        // this should always be the last line so any telemetry that wes done in other
        // methods is displayed
        telemetry.update();
    }

    // Code written by Narendra
    private void ProcessClaw() {
        double currentPower = 0.0;

        if (gamepad1.left_bumper){
            currentPower = 1.0;
        }

        if (gamepad1.right_bumper) {
            currentPower = -1.0;
        }

        robot.GetClawL().setPower(currentPower);
        robot.GetClawR().setPower(currentPower);
    }

    private void PowerChange() {
        if (gamepad1.x) {
            robot.SetPowerPercentage(0.90);
            robot.SetLastPowerSetting(0.90);
        }
        else if (gamepad1.y) {
            robot.SetPowerPercentage(1.0);
            robot.SetLastPowerSetting(1.0);
        }
        else if (gamepad1.a) {
            robot.SetPowerPercentage(0.8);
            robot.SetLastPowerSetting(0.8);
        }
        else if (gamepad1.b) {
            robot.SetPowerPercentage(0.95);
            robot.SetLastPowerSetting(0.95);
        }
    }

    private void LiftControl() {
        // Sets initial power.
        double LiftPower = 0.0;

        // We can disable the boundaries of maximum lift height and tilt checks
        if (gamepad1.left_trigger > 0) {
            LiftPower = -1.0;
        }

        if ((gamepad2.right_trigger > 0) || (gamepad1.right_trigger > 0)) {
            LiftPower = 1.0;
        }

        if ((LiftPower != 0) || (robot.GetLift().getMode() == DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
            robot.GetLift().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.GetLift().setPower(LiftPower);
        }
    }

    // This code moves the slide motor.
    private void SlideControl() {
        double SlidePower;

        SlidePower = -gamepad1.right_stick_x * robot.GetPowerPercentage();
        SlidePower = Range.clip(SlidePower, -1, 1);

        robot.GetSlide().setPower(SlidePower);
    }

    /*
        Powers robot for TeleOp arcade drive and clips power values.
     */
    private void DriveControl(){
        float xValue = gamepad1.left_stick_x;
        float yValue = gamepad1.left_stick_y;

        double distance = Math.sqrt((yValue * yValue) + (xValue * xValue));
        double angle = Math.toDegrees(Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x));

        //slow down turns

        robot.SetPowerPercentage(robot.GetLastPowerSetting());

        if (distance > 0.8) {
            if (Math.abs(angle) < 45) {
                robot.SetPowerPercentage(0.5);
            } else if (Math.abs(angle) > 135) {
                robot.SetPowerPercentage(0.5);
            }
        }

        //calculate the power needed for each motor
        double leftPower = (yValue - xValue) * robot.GetPowerPercentage();
        double rightPower =(yValue + xValue) * robot.GetPowerPercentage();

        //clip the power values so that it only goes from -1 to 1
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        //set the power of the motors with the gamepad values
        robot.GetLeftFront().setPower(leftPower);
        robot.GetLeftBack().setPower(leftPower);
        robot.GetRightFront().setPower(rightPower);
        robot.GetRightBack().setPower(rightPower);
    }

}
