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

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;
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
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

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

abstract class BaseAutonomousOpMode extends LinearOpMode
{
    protected PieceOfCakeRobot robot = new PieceOfCakeRobot();
    private RelicRecoveryVuMark relicRecoveryVuMark = RelicRecoveryVuMark.CENTER;

    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    private double currentWheelPower = 0.10;
    protected Boolean centerDefault = false;

    private VuforiaLocalizer vuforia;

    public enum AllianceColor
    {
        Blue,
        Red
    }

    public abstract void runOpMode();

    protected void InitRobot() {
        robot.init(hardwareMap);

        // We are initializing the motors.
        robot.GetRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetRightBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.GetSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetSlide().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.GetLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLeftFront().setDirection(DcMotor.Direction.REVERSE);
        robot.GetLeftBack().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetLeftBack().setDirection(DcMotor.Direction.REVERSE);

        robot.GetClawL().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.GetClawR().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.GetLeftServo().setPosition(0);
        robot.GetRightServo().setPosition(0);

        // Setting the Power of the motors to 0.15.
        ChangeWheelPowerLevel(0.15);

        InitGyro();
        InitCamera();

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        idle();
    }

    private void InitGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        resetAngle();
    }

    private void InitCamera() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ASBTwVj/////AAAAGWrL5O30VElzgeaoDGIa2shP45ENRY3zwoEBXHkCDTtRmQYmDiFRtuuULnBl5g+fcXpsEiKBitgTN620Up1AKg+r0MpJepnoPfjvoo94oX2JgDNF2lS4AULsXyiEUR6Zq/ObTtIY1+/en1Qj2c28RUsp6+B4VaznIMKtwIGlhFQTpx4xn22I1SPxDpyCfzSC8+d6NDlBoUa8krwX5D+spdWWHZg+69JaFMVCQWCOktKHTyQVQrAicJkrDIQCq2onLAIehZzk+wXySRqix+O5Eftg9tDtjPKErgavsrbWg3/O+PkXYMfKXspNu/laVPJfXM95f3bgt7H6QIcHZiro/A6zsTeubBqNsUeVCMLTAxR6";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    }

    protected void LoadGlyphPosition() {
        int counter = 0;
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate();
        while (counter < 20)
        {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            telemetry.addData("VuMark", "%s visible", relicRecoveryVuMark);
            telemetry.update();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN)
            {
                relicRecoveryVuMark = vuMark;
                break;
            }

            counter ++;
            sleep(100);
            idle();
        }

        if (counter >= 20)
            centerDefault = true;

        relicTrackables.deactivate();
    }

    protected RelicRecoveryVuMark GetGlyphPosition() {
        return relicRecoveryVuMark;
    }

    protected void ChangeWheelPowerLevel(double wheelPowerLevel){
        currentWheelPower = wheelPowerLevel;
    }

    protected float LowPass(float colorAverage, float colorSample) {
        // (0 - .99) Lower value results in stronger smoothing.
        final float FILTER_COEFFICIENT = .2F;
        // Used to filter out values that are way out of range. Tune for expected range.
        final float THRESHOLD = 9F;

        // Optional code to remove outliers.
        /*
        if (Math.abs(colorSample - colorAverage) > THRESHOLD) {
            colorSample = colorAverage;
        }
        */
        colorAverage = ((1.0F - FILTER_COEFFICIENT) * (FILTER_COEFFICIENT + colorSample));
        return  colorAverage;
    }


    protected boolean isRedInFront2(ColorSensor colorSensor) {
        float redAverage = 0F;
        float blueAverage = 0F;

        for (int x = 0; x < 25; x++) {
            redAverage = LowPass(redAverage, colorSensor.red());
            blueAverage = LowPass(blueAverage, colorSensor.blue());
            idle();

            telemetry.addData("average red", redAverage);
            telemetry.addData("average blue", blueAverage);
            telemetry.addData("x", x);
            telemetry.update();
        }

        if (redAverage > blueAverage) {
            return true;
        } else {
            return false;
        }
    }

    protected boolean isRedInFront(ColorSensor colorSensor) {
        int FrontRed = 0;
        int BackRed = 0;

        for (int x = 0; x<11; x++) {

            sleep(50);

            telemetry.addData("front red", colorSensor.red());
            telemetry.addData("front blue", colorSensor.blue());
            telemetry.addData("total counts ", "%d %d", FrontRed, BackRed);
            telemetry.update();

            if (colorSensor.red() > colorSensor.blue()){
                FrontRed = FrontRed + 1;
            }
            else {
                BackRed = BackRed + 1;
            }
        }

        if (FrontRed > BackRed) {
            return true;
        }
        else {
            return false;
        }
    }

    protected void DropArmKnockLiftArmReposition(AllianceColor allianceColor, int distanceToMoveAfterWards) {
        if (allianceColor == AllianceColor.Blue) {
            // drop arm
            robot.GetLeftServo().setPosition(1.0);
            sleep(1500);
            // check color
            if (isRedInFront2(robot.GetLeftColorSensor())){
                // move forward
                MoveRobot(300); // if you change this number change the other 400's
                // raise arm
                robot.GetLeftServo().setPosition(0.0);
                //sleep(1500);
                MoveRobot(distanceToMoveAfterWards - 300);
            } else {
                // turn
                TurnSlightLeft();
                // raise arm
                robot.GetLeftServo().setPosition(0.0);
                //sleep(1500);
                // move forward
                MoveToPosition(0,0);
                MoveRobot(distanceToMoveAfterWards);
            }
        } else if (allianceColor == AllianceColor.Red){
            // drop arm
            robot.GetRightServo().setPosition(1.0);
            sleep(1500);
            // check color
            if (isRedInFront2(robot.GetRightColorSensor()) == false){
                // move forward
                MoveRobot(300); // if you change this number change the other 400's
                // raise arm
                robot.GetRightServo().setPosition(0.0);
                //sleep(1500);
                MoveRobot(distanceToMoveAfterWards - 300);
            } else {
                // turn
                TurnSlightRight();
                // raise arm
                robot.GetRightServo().setPosition(0.0);
                //sleep(1000);
                // move forward
                MoveToPosition(0,0);
                MoveRobot(distanceToMoveAfterWards);
            }
        }
    }

    protected void MoveClaw(int distance) {
        int currentPower = -1;

        robot.GetClawL().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.GetClawR().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.GetClawL().setPower(currentPower);
        robot.GetClawR().setPower(currentPower);

        robot.GetClawL().setTargetPosition(robot.GetClawL().getCurrentPosition() + distance);
        robot.GetClawR().setTargetPosition(robot.GetClawR().getCurrentPosition() + distance);

        while (opModeIsActive() && (robot.GetClawR().isBusy() || robot.GetClawL().isBusy())) {
        }

        robot.GetClawL().setPower(0);
        robot.GetClawR().setPower(0);
        robot.GetClawL().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.GetClawR().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    protected void MoveRobot(int leftDistance, int rightDistance){
        robot.GetRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.GetLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.GetRightBack().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.GetLeftBack().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.GetLeftFront().setTargetPosition(robot.GetLeftFront().getCurrentPosition() + -leftDistance);
        robot.GetRightFront().setTargetPosition(robot.GetRightFront().getCurrentPosition() + -rightDistance);

        robot.GetLeftBack().setTargetPosition(robot.GetLeftFront().getCurrentPosition() + -leftDistance);
        robot.GetRightBack().setTargetPosition(robot.GetRightFront().getCurrentPosition() + -rightDistance);

        robot.GetLeftFront().setPower(currentWheelPower);
        robot.GetRightFront().setPower(currentWheelPower);
        robot.GetLeftBack().setPower(currentWheelPower);
        robot.GetRightBack().setPower(currentWheelPower);

        while (opModeIsActive() && (robot.GetLeftFront().isBusy() &&
                robot.GetRightFront().isBusy())) {

            getAngle();

            telemetry.addData("1 imu heading", "%f %f %f", lastAngles.firstAngle, lastAngles.secondAngle, lastAngles.thirdAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", checkDirection());
            telemetry.addData("VuMark", "%s visible", relicRecoveryVuMark);
            telemetry.addData("Center Defualt", centerDefault);
            telemetry.addData("Left Front Position", "%d", robot.GetLeftFront().getCurrentPosition());
            telemetry.addData("Left Back Position", "%d", robot.GetLeftBack().getCurrentPosition());
            telemetry.addData("Right Front Position", "%d", robot.GetRightFront().getCurrentPosition());
            telemetry.addData("Right Back Position", "%d", robot.GetRightBack().getCurrentPosition());
            telemetry.addData("Slide Position", "%d", robot.GetSlide().getCurrentPosition());
            telemetry.addData("Position", imu.getPosition().toString());
            telemetry.update();

        }

        robot.GetLeftBack().setPower(0);
        robot.GetRightBack().setPower(0);
        robot.GetLeftFront().setPower(0);
        robot.GetRightFront().setPower(0);

        robot.GetRightFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetLeftFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetLeftBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetRightBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    protected void Rotate(int degrees, double power)
    {
        double  leftPower, rightPower;
        double angle;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

        // set power to rotate.
        robot.GetLeftFront().setPower(leftPower);
        robot.GetLeftBack().setPower(leftPower);
        robot.GetRightFront().setPower(rightPower);
        robot.GetRightBack().setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && (angle = getAngle()) == 0) {
                telemetry.addData("Zero Angle ", angle);
                telemetry.update();
            }

            while (opModeIsActive() && (angle = getAngle()) > degrees) {
                telemetry.addData("Right Angle ", angle);
                telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && (angle = getAngle()) < degrees) {
                telemetry.addData("Left Angle ", angle);
                telemetry.update();
            }

        // turn the motors off.
        robot.GetLeftFront().setPower(0);
        robot.GetLeftBack().setPower(0);
        robot.GetRightFront().setPower(0);
        robot.GetRightBack().setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    protected void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    protected double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    protected void TurnSlightRight() {
        MoveToPosition(-115, 115);
    }

    protected void TurnSlightLeft() { MoveToPosition(115, -115); }

    protected void MoveRobot(int distance) {
        MoveRobot(distance, distance);
    }

    protected void SlideRobot(int distance) {
        robot.GetSlide().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.GetSlide().setTargetPosition(robot.GetSlide().getCurrentPosition() + distance);

        robot.GetSlide().setPower(0.35);

        // waiting for the turn to finish
        while (opModeIsActive() && robot.GetSlide().isBusy()) {
            getAngle();
            telemetry.addData("1 imu heading", lastAngles.firstAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", checkDirection());
            telemetry.addData("VuMark", "%s visible", relicRecoveryVuMark);
            telemetry.addData("Center Defualt", centerDefault);
            telemetry.addData("Left Position", "%d", robot.GetLeftFront().getCurrentPosition());
            telemetry.addData("Right Position", "%d", robot.GetRightFront().getCurrentPosition());
            telemetry.addData("Slide Position", "%d", robot.GetSlide().getCurrentPosition());
            telemetry.update();
        }

        robot.GetSlide().setPower(0);

        robot.GetSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    protected void MoveToPosition(int leftPostion, int rightPosition) {

        robot.GetLeftFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.GetLeftBack().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.GetRightFront().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.GetRightBack().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.GetLeftFront().setTargetPosition(leftPostion);
        robot.GetLeftBack().setTargetPosition(leftPostion);
        robot.GetRightFront().setTargetPosition(rightPosition);
        robot.GetRightBack().setTargetPosition(rightPosition);

        robot.GetRightFront().setPower(currentWheelPower);
        robot.GetRightBack().setPower(currentWheelPower);
        robot.GetLeftFront().setPower(currentWheelPower);
        robot.GetLeftBack().setPower(currentWheelPower);

        // waiting for the turn to finish
        while (opModeIsActive() && (robot.GetLeftFront().isBusy() && robot.GetRightFront().isBusy())) {
            telemetry.addData("Left Front Position", "%d", robot.GetLeftFront().getCurrentPosition());
            telemetry.addData("Left Back Position", "%d", robot.GetLeftBack().getCurrentPosition());
            telemetry.addData("Right Front Position", "%d", robot.GetRightFront().getCurrentPosition());
            telemetry.addData("Right Back Position", "%d", robot.GetRightBack().getCurrentPosition());
            telemetry.addData("Slide Position", "%d", robot.GetSlide().getCurrentPosition());
            telemetry.update();

        }

        robot.GetLeftFront().setPower(0);
        robot.GetLeftBack().setPower(0);
        robot.GetRightFront().setPower(0);
        robot.GetRightBack().setPower(0);

        robot.GetLeftFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetLeftBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetRightBack().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.GetRightFront().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
// Done!