package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

abstract class BaseAutoOpMode extends LinearOpMode {

    protected PieceOfCake robot = new PieceOfCake();
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;
    protected Boolean centerDefault = false;

    private VuforiaLocalizer vuforia;
    private double currentWheelPower = 1;

    protected void InitRobot() {
        robot.init(hardwareMap);

     robot.getBackR().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     robot.getFrontR().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     robot.getFrontL().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     robot.getBackR().setDirection(DcMotor.Direction.REVERSE);
     robot.getBackL().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     robot.getFrontR().setDirection(DcMotor.Direction.REVERSE);

        InitGyro();
        InitCamera();
     }

    protected void MoveRobot(int leftDistance, int rightDistance){
        robot.getFrontR().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getFrontL().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBackL().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getBackR().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getFrontL().setTargetPosition(robot.getFrontL().getCurrentPosition() + -leftDistance);
        robot.getFrontR().setTargetPosition(robot.getFrontR().getCurrentPosition() + -rightDistance);

        robot.getBackL().setTargetPosition(robot.getFrontL().getCurrentPosition() + -leftDistance);
        robot.getBackR().setTargetPosition(robot.getFrontR().getCurrentPosition() + -rightDistance);

        robot.getFrontR().setPower(currentWheelPower);
        robot.getFrontL().setPower(currentWheelPower);
        robot.getBackL().setPower(currentWheelPower);
        robot.getBackR().setPower(currentWheelPower);

        while (opModeIsActive() && (robot.getFrontL().isBusy() &&
                robot.getFrontR().isBusy())) {

            getAngle();

            telemetry.addData("1 imu heading", "%f %f %f", lastAngles.firstAngle, lastAngles.secondAngle, lastAngles.thirdAngle);
            telemetry.addData("2 global heading", globalAngle);
            telemetry.addData("3 correction", checkDirection());
//            telemetry.addData("VuMark", "%s visible", relicRecoveryVuMark);
            telemetry.addData("Center Defualt", centerDefault);
            telemetry.addData("Front Left Position", "%d", robot.getFrontL().getCurrentPosition());
            telemetry.addData("Front Right Position", "%d", robot.getFrontR().getCurrentPosition());
            telemetry.addData("Back Left Position", "%d", robot.getBackL().getCurrentPosition());
            telemetry.addData("Back Right Position", "%d", robot.getBackR().getCurrentPosition());
            telemetry.addData("Position", imu.getPosition().toString());
            telemetry.update();

        }

        robot.getFrontL().setPower(0);
        robot.getFrontR().setPower(0);
        robot.getBackL().setPower(0);
        robot.getBackR().setPower(0);

        robot.getFrontL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getFrontR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackL().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.getBackR().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        robot.getFrontL().setPower(leftPower);
        robot.getFrontR().setPower(leftPower);
        robot.getBackL().setPower(rightPower);
        robot.getBackR().setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && (angle = getAngle()) == 0) {
                telemetry.addData("Zero Angle ", angle);
                telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && (angle = getAngle()) < degrees) {
                telemetry.addData("Left Angle ", angle);
                telemetry.update();
            }

        // turn the motors off.
        robot.getFrontL().setPower(0);
        robot.getFrontR().setPower(0);
        robot.getBackL().setPower(0);
        robot.getBackR().setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    protected void resetAngle ()
    {
        
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private void InitCamera () {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ASBTwVj/////AAAAGWrL5O30VElzgeaoDGIa2shP45ENRY3zwoEBXHkCDTtRmQYmDiFRtuuULnBl5g+fcXpsEiKBitgTN620Up1AKg+r0MpJepnoPfjvoo94oX2JgDNF2lS4AULsXyiEUR6Zq/ObTtIY1+/en1Qj2c28RUsp6+B4VaznIMKtwIGlhFQTpx4xn22I1SPxDpyCfzSC8+d6NDlBoUa8krwX5D+spdWWHZg+69JaFMVCQWCOktKHTyQVQrAicJkrDIQCq2onLAIehZzk+wXySRqix+O5Eftg9tDtjPKErgavsrbWg3/O+PkXYMfKXspNu/laVPJfXM95f3bgt7H6QIcHZiro/A6zsTeubBqNsUeVCMLTAxR6";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
    }

    private void InitGyro () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        resetAngle();
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
}
