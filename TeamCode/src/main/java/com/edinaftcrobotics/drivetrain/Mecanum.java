package com.edinaftcrobotics.drivetrain;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mecanum {
    private DcMotor _frontLeft;
    private DcMotor _frontRight;
    private DcMotor _backLeft;
    private DcMotor _backRight;
    private Telemetry _telemetry;

    private double _currentPower = 1.0;

    public Mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, boolean isTeleop, Telemetry telemetry)
    {
        _frontLeft = fl;
        _frontRight = fr;
        _backLeft = bl;
        _backRight = br;
        _telemetry = telemetry;

        if (isTeleop) {
            _backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            _frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    public void SlideLeft(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors();
        SetDistance(-distance, distance, distance, -distance);

        int currentPosition =  Math.abs(_backRight.getCurrentPosition());
        int error = Math.abs((int)(distance * 0.95));
        Move(power, power, power, power);

        while ((currentPosition < error)) {
            currentPosition =  Math.abs(_backRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void SlideLeft2(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors2();

        int currentPosition =  Math.abs(_backRight.getCurrentPosition());
        int error = Math.abs((int)(distance * 0.95));
        Move(-power, power, power, -power);

        while ((currentPosition < error)) {
            currentPosition =  Math.abs(_backRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void Move(double left, double right){
        _frontLeft.setPower(left);
        _frontRight.setPower(right);
        _backLeft.setPower(left);
        _backRight.setPower(right);
    }

    public void Move(double fl, double fr, double bl, double br) {
        _frontLeft.setPower(fl);
        _frontRight.setPower(fr);
        _backLeft.setPower(bl);
        _backRight.setPower(br);
    }

    public void SlideRight(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors();
        SetDistance(distance, -distance, -distance, distance);

        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        int error = Math.abs((int)(distance * 0.95));
        Move(power, power, power, power);

        while ((currentPosition < error)) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void SlideRight2(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors2();

        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        int error = Math.abs((int)(distance * 0.95));
        Move(power, -power, -power, power);

        while (currentPosition < error) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void SlideRight3(double power, int distance, LinearOpMode opMode, BNO055IMU imu) {
        Move(power, -power, -power, power);
    }

    public void MoveForward(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors();
        SetDistance(distance, distance, distance, distance);

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error)) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void MoveForward2(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors2();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        double currentPower = CalculateRampPower(power, distance, currentPosition);
        Move(currentPower, currentPower, currentPower, currentPower);

        while (currentPosition < error) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            currentPower = CalculateRampPower(power, distance, currentPosition);
            Move(currentPower, currentPower, currentPower, currentPower);
            opMode.idle();
        }

        Stop();
    }

    public void MoveBackwards(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors();
        SetDistance(-distance, -distance, -distance, -distance);

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error)) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void MoveBackwards2(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors2();

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        double currentPower = CalculateRampPower(power, distance, currentPosition);

        Move(-currentPower, -currentPower, -currentPower, -currentPower);

        while (currentPosition < error) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            CalculateRampPower(power, distance, currentPosition);
            Move(-currentPower, -currentPower, -currentPower, -currentPower);
            opMode.idle();
        }

        Stop();
    }

    public void TurnRight(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors();
        SetDistance(distance, distance, -distance, -distance);

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error)) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void TurnLeft(double power, int distance, LinearOpMode opMode) {
        StopAndResetMotors();
        SetDistance(-distance, -distance, distance, distance);

        int error = Math.abs((int)(distance * 0.95));
        int currentPosition =  Math.abs(_frontRight.getCurrentPosition());
        Move(power, power, power, power);

        while (_frontLeft.isBusy() && _frontRight.isBusy() && _backLeft.isBusy() && _backRight.isBusy() && (currentPosition < error)) {
            currentPosition =  Math.abs(_frontRight.getCurrentPosition());
            opMode.idle();
        }

        Stop();
    }

    public void Stop() {
        _frontLeft.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(0);
    }

    //
    // This method will calculate a power based on the current position and our maximum distance
    // We want to ramp up the speed to a flat maxPower and then ramp down to zero.
    //
    private double CalculateRampPower(double maxPower, int distance, double currentDistance) {
        if (currentDistance <= (distance * .10)) {
            return .6 * maxPower;
        } else if (currentDistance <= (distance * .20)) {
            return  .85 * maxPower;
        } else if (currentDistance <= (distance * .70)) {
            return maxPower;
        } else if (currentDistance <= (distance * .80)) {
            return .85 * maxPower;
        } else {
            return .6 * maxPower;
        }
    }

    public void Drive(double leftStickX, double leftStickY, double rightStickY) {
        final double x = Math.pow(-leftStickX, 3.0);
        final double y = Math.pow(leftStickY, 3.0);

        final double rotation = Math.pow(-rightStickY, 3.0);
        final double direction = Math.atan2(x, y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double fl = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double fr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double bl = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double br = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        _frontLeft.setPower(-fl * _currentPower);
        _frontRight.setPower(-fr * _currentPower);
        _backLeft.setPower(-bl * _currentPower);
        _backRight.setPower(-br * _currentPower);
    }

    public void SetCurrentPower(double power){
        _currentPower = power;
    }

    public void StopAndResetMotors() {
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void StopAndResetMotors2() {
        _frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void SetDistance(int lf, int lb, int rf, int rb) {
        _frontLeft.setTargetPosition(lf);
        _frontRight.setTargetPosition(rf);
        _backLeft.setTargetPosition(lb);
        _backRight.setTargetPosition(rb);
    }
}
