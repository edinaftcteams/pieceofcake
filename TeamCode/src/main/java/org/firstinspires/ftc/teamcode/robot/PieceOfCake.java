package org.firstinspires.ftc.teamcode.robot;
//imports
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class PieceOfCake {
    private DcMotor FrontR = null;
    private DcMotor FrontL = null;
    private DcMotor BackR = null;
    private DcMotor BackL = null;
    private DcMotor Lift = null;
    private DcMotor Slide = null;
    private DcMotor FrontFlip = null;
    private DcMotor Intake = null;
    private Servo TopFlip = null;
    private CRServo LockServo = null;
    private HardwareMap hwMap = null;

    //constructor
    public PieceOfCake() {

    }
    public void init(HardwareMap ahwMap) {
        DcMotor dcMotor = null;
        Servo servo = null;

        hwMap = ahwMap;

        dcMotor = hwMap.dcMotor.get("fl");
        SetFrontL(dcMotor);

        dcMotor = hwMap.dcMotor.get("fr");
        SetFrontR(dcMotor);

        dcMotor = hwMap.dcMotor.get("bl");
        SetBackL(dcMotor);

        dcMotor = hwMap.dcMotor.get("br");
        SetBackR(dcMotor);

        dcMotor = hwMap.dcMotor.get("lift");
        SetLift(dcMotor);

        dcMotor = hwMap.dcMotor.get("slide");
        SetSlide(dcMotor);

        dcMotor = hwMap.dcMotor.get("flip");
        SetFrontFlip(dcMotor);

        dcMotor = hwMap.dcMotor.get("intake");
        SetIntake(dcMotor);

        servo = hwMap.servo.get("topflip");
        SetTopFlip(servo);

      //  SetLockServo(hwMap.crservo.get("lockservo"));
    }


    public DcMotor getFrontL() {
        return FrontL;
    }

    public DcMotor getFrontR() {
        return FrontR;
    }

    public DcMotor getBackL() {
        return BackL;
    }

    public DcMotor getBackR() {
        return BackR;
    }

    public DcMotor getLift() { return Lift; }

    public DcMotor getSlide() { return Slide; }

    public DcMotor getFrontFlip() { return FrontFlip; }

    public DcMotor getIntake() { return Intake; }

    public Servo getTopFlip() { return TopFlip; }

    //public CRServo getLockServo() { return LockServo; }


    private void SetFrontL(DcMotor dcMotor) {FrontL = dcMotor; }
    private void SetFrontR(DcMotor dcMotor) {FrontR = dcMotor; }
    private void SetBackL(DcMotor dcMotor) {BackL = dcMotor; }
    private void SetBackR(DcMotor dcMotor) {BackR = dcMotor; }
    private void SetLift(DcMotor dcMotor) {Lift = dcMotor; }
    private void SetSlide(DcMotor dcMotor) {Slide = dcMotor; }
    private void SetFrontFlip(DcMotor dcMotor) {FrontFlip = dcMotor; }
    private void SetIntake(DcMotor dcMotor) {Intake = dcMotor; }
    private void SetTopFlip(Servo servo) {TopFlip = servo; }
    //private void SetLockServo(CRServo servo) { LockServo = servo; }

    public void setMotorPower(double fl, double fr, double bl, double br){
        getFrontL().setPower(fl);
        getFrontR().setPower(fr);
        getBackL().setPower(bl);
        getBackR().setPower(br);
    }
}


