package org.firstinspires.ftc.teamcode;
//Imports
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Set;

/**
* Created by MeechMeechman on 9/23/17.
*/
//Defining motors
public class PieceOfCakeRobot {
    private DcMotor ClawL = null;
    private DcMotor ClawR = null;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor Lift = null;
    private DcMotor Slide = null;
    private DcMotor Tilt = null;
    private ColorSensor LeftColorSensor = null;
    private ColorSensor RightColorSensor = null;
    private HardwareMap hwMap = null;
    private double PowerPercentage = 1.0;
    private double LastPowerSetting = 1.0;
    private Servo LeftServo = null;
    private Servo RightServo = null;

    // Constructor
    public PieceOfCakeRobot() {

    }

    //Hardware naming
    public void init(HardwareMap ahwMap) {
        DcMotor dcMotor = null;
        ColorSensor colorSensor = null;

        hwMap = ahwMap;

        dcMotor = hwMap.dcMotor.get("clawleft"); //0
        SetClawL(dcMotor);

        dcMotor = hwMap.dcMotor.get("clawright"); //1
        SetClawR(dcMotor);

        dcMotor = hwMap.dcMotor.get("leftfront"); //0
        SetLeftFront(dcMotor);

        dcMotor = hwMap.dcMotor.get("leftback"); //0?
        SetLeftBack(dcMotor);

        dcMotor = hwMap.dcMotor.get("rightfront"); //1
        SetRightFront(dcMotor);

        dcMotor = hwMap.dcMotor.get("rightback"); //1?
        SetRightBack(dcMotor);

        dcMotor = hwMap.dcMotor.get("lift"); //2
        SetLift(dcMotor);

        dcMotor = hwMap.dcMotor.get("slide");
        SetSlide(dcMotor);

        colorSensor = hwMap.colorSensor.get("leftcolorsensor");
        SetLeftColorSensor(colorSensor);

        colorSensor = hwMap.colorSensor.get("rightcolorsensor");
        SetRightColorSensor(colorSensor);

        SetLeftServo(hwMap.servo.get("leftservo"));

        SetRightServo(hwMap.servo.get("rightservo"));
        GetRightServo().setDirection(Servo.Direction.REVERSE);
    }

        // sets the .get for all variables
    public DcMotor GetClawL() {
        return ClawL;
    }

    public DcMotor GetClawR() {
        return ClawR;
    }

    public DcMotor GetLeftFront() {
        return LeftFront;
    }

    public DcMotor GetLeftBack() {return LeftBack; }

    public DcMotor GetRightFront() {
        return RightFront;
    }

    public DcMotor GetRightBack() {
        return RightBack;
    }

    public DcMotor GetLift() {
        return Lift;
    }

    public DcMotor GetSlide() {
        return Slide;
    }

    public ColorSensor GetLeftColorSensor() { return LeftColorSensor;}

    public ColorSensor GetRightColorSensor() { return RightColorSensor;}

    public Double GetPowerPercentage() {
        return PowerPercentage;
    }

    public double GetLastPowerSetting() {
        return LastPowerSetting;
    }

    public Servo GetLeftServo() { return LeftServo; }

    public Servo GetRightServo() { return RightServo; }

    private void SetClawR(DcMotor motor) {
        ClawR = motor;
    }

    private void SetClawL(DcMotor motor) {
        ClawL = motor;
    }

    private void SetLeftFront(DcMotor dcMotor) {
        LeftFront = dcMotor;
    }

    private void SetLeftBack(DcMotor dcMotor) {
        LeftBack = dcMotor;
    }

    private void SetRightFront(DcMotor dcMotor) {
        RightFront = dcMotor;
    }

    private void SetRightBack(DcMotor dcMotor) {
        RightBack = dcMotor;
    }

    private void SetLift(DcMotor dcMotor) {
        Lift = dcMotor;
    }

    private void SetSlide(DcMotor dcMotor) {
        Slide = dcMotor;
    }

    private void SetLeftColorSensor(ColorSensor colorSensor) { LeftColorSensor = colorSensor; }

    private void SetRightColorSensor(ColorSensor colorSensor) { RightColorSensor = colorSensor; }

    private void SetLeftServo(Servo leftServo) { LeftServo = leftServo; }

    private void SetRightServo(Servo rightServo) { RightServo = rightServo; }

    public void SetPowerPercentage(Double powerPercentage) {
        PowerPercentage = powerPercentage;
    }

    public void SetLastPowerSetting(Double lastPowerSetting) {
        LastPowerSetting = lastPowerSetting;
    }
}

