package com.edinaftcrobotics.drivetrain;

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

    private void SetDistance(int lf, int lb, int rf, int rb) {
        _frontLeft.setTargetPosition(lf);
        _frontRight.setTargetPosition(rf);
        _backLeft.setTargetPosition(lb);
        _backRight.setTargetPosition(rb);
    }
}
