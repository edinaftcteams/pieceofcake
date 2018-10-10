package com.edinaftcrobotics.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Mecanum {
    private DcMotor _frontLeft;
    private DcMotor _frontRight;
    private DcMotor _backLeft;
    private DcMotor _backRight;

    public Mecanum(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br)
    {
        _frontLeft = fl;
        _frontRight = fr;
        _backLeft = bl;
        _backRight = br;
    }

    public void SlideLeft(double power){
        _frontLeft.setPower(-power);
        _frontRight.setPower(power);
        _backLeft.setPower(power);
        _backRight.setPower(-power);
    }

    public void SlideRight(double power){
        _frontLeft.setPower(power);
        _frontRight.setPower(-power);
        _backLeft.setPower(-power);
        _backRight.setPower(power);
    }

    public void MoveNE(double power) {
        _frontLeft.setPower(power);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(power);
    }

    public void MoveSE(double power) {
        _frontLeft.setPower(0);
        _frontRight.setPower(-power);
        _backLeft.setPower(-power);
        _backRight.setPower(0);
    }

    public void MoveNW(double power) {
        _frontLeft.setPower(0);
        _frontRight.setPower(power);
        _backLeft.setPower(power);
        _backRight.setPower(0);
    }

    public void MoveSW(double power) {
        _frontLeft.setPower(-power);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(-power);
    }

    public void MoveForward(double power) {
        _frontLeft.setPower(power);
        _frontRight.setPower(power);
        _backLeft.setPower(power);
        _backRight.setPower(power);
    }

    public void MoveBackwards(double power) {
        _frontLeft.setPower(-power);
        _frontRight.setPower(-power);
        _backLeft.setPower(-power);
        _backRight.setPower(-power);
    }

    public void TurnRight(double power) {
        _frontLeft.setPower(power);
        _frontRight.setPower(0);
        _backLeft.setPower(power);
        _backRight.setPower(0);
    }

    public void TurnLeft(double power) {
        _frontLeft.setPower(power);
        _frontRight.setPower(0);
        _backLeft.setPower(0);
        _backRight.setPower(power);
    }

    public void TurnRightOnRearAxis(double power) {
        _frontLeft.setPower(power);
        _frontRight.setPower(-power);
        _backLeft.setPower(0);
        _backRight.setPower(0);
    }

    public void TurnLeftOnRearAxis(double power){
        _frontLeft.setPower(-power);
        _frontRight.setPower(power);
        _backLeft.setPower(0);
        _backRight.setPower(0);
    }

    public void TurnRightOnFrontAxis(double power) {
        _frontLeft.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(-power);
        _backRight.setPower(power);
    }

    public void TurnLeftOnFrontAxis(double power){
        _frontLeft.setPower(0);
        _frontRight.setPower(0);
        _backLeft.setPower(power);
        _backRight.setPower(-power);
    }
}
