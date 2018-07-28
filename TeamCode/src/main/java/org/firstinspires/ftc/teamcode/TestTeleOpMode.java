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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TestTeleOpMode Mode", group="Robot Opmode")
@Disabled
public class TestTeleOpMode extends OpMode {

    private PieceOfCakeRobot robot = new PieceOfCakeRobot();

    @Override
    public void init(){
        //Reset motor encoders
        robot.init(hardwareMap);


        robot.GetLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLeftFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetLeftFront().setDirection(DcMotor.Direction.REVERSE);
        robot.GetLeftFront().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.GetLeftBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLeftBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetLeftBack().setDirection(DcMotor.Direction.REVERSE);
        robot.GetLeftBack().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.GetRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetRightFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetRightFront().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.GetRightBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetRightBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetRightBack().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.GetLeftServo().setPosition(1);
        robot.GetRightServo().setPosition(1);
    }

    @Override
    public void loop(){
        handleButtons();

        telemetry.addData("Left Front Position", "%d", robot.GetLeftFront().getCurrentPosition());
        telemetry.addData("Right Front Position", "%d", robot.GetRightFront().getCurrentPosition());
        telemetry.addData("Left Back Position", "%d", robot.GetLeftBack().getCurrentPosition());
        telemetry.addData("Right Back Position", "%d", robot.GetRightBack().getCurrentPosition());
        telemetry.addData("Left Clamp Position", "%d", robot.GetClawL().getCurrentPosition());
        telemetry.addData("Right Clamp Position", "%d", robot.GetClawR().getCurrentPosition());
        telemetry.addData("Slide Position", "%d", robot.GetSlide().getCurrentPosition());

        telemetry.addData("PowerPercentage", "%f", robot.GetPowerPercentage());

        // this should always be the last line so any telemetry that wes done in other
        // methods is displayed
        telemetry.update();
    }

    // Code written by Narendra
    private void handleButtons(){
        int leftDistance = 0;
        int rightDistance = 0;
        double currentWheelPower = 0.25;

        if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
            if (gamepad1.a) {
                leftDistance = 1000;
                rightDistance = 1000;
            }

            if (gamepad1.b) {
                leftDistance = 1000;
                rightDistance = -1000;
            }

            if (gamepad1.y) {
                leftDistance = -1000;
                rightDistance = 1000;
            }

            if (gamepad1.x) {
                leftDistance = -1000;
                rightDistance = -1000;
            }

            robot.GetRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.GetLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.GetRightBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.GetLeftBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.GetLeftFront().setTargetPosition(robot.GetLeftFront().getCurrentPosition() + -leftDistance);
            robot.GetRightFront().setTargetPosition(robot.GetRightFront().getCurrentPosition() + -rightDistance);

            robot.GetLeftFront().setPower(currentWheelPower);
            robot.GetRightFront().setPower(currentWheelPower);
            robot.GetLeftBack().setPower(currentWheelPower);
            robot.GetRightBack().setPower(currentWheelPower);

            if (leftDistance < 0)
                robot.GetLeftBack().setPower(currentWheelPower);
            else
                robot.GetLeftBack().setPower(-currentWheelPower);

            if (rightDistance < 0)
                robot.GetRightBack().setPower(currentWheelPower);
            else
                robot.GetRightBack().setPower(-currentWheelPower);

            while ((robot.GetLeftFront().isBusy() &&
                    robot.GetRightFront().isBusy())) {

                telemetry.addData("Left Front Position", "%d", robot.GetLeftFront().getCurrentPosition());
                telemetry.addData("Left Back Position", "%d", robot.GetLeftBack().getCurrentPosition());
                telemetry.addData("Right Front Position", "%d", robot.GetRightFront().getCurrentPosition());
                telemetry.addData("Right Back Position", "%d", robot.GetRightBack().getCurrentPosition());
                telemetry.addData("Slide Position", "%d", robot.GetSlide().getCurrentPosition());
                telemetry.update();
            }

            robot.GetLeftFront().setPower(0);
            robot.GetRightFront().setPower(0);
            robot.GetLeftBack().setPower(0);
            robot.GetRightBack().setPower(0);

            robot.GetRightFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.GetLeftFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.GetLeftBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.GetRightBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
