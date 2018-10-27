package org.firstinspires.ftc.teamcode.robot;
//imports
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
    private DcMotor Flip = null;
    private DcMotor Intake = null;
    private Servo LeftFlip = null;
    private Servo RightFlip = null;


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
        SetFlip(dcMotor);

        dcMotor = hwMap.dcMotor.get("intake");
        SetIntake(dcMotor);

        servo = hwMap.servo.get("lflip");
        SetLeftFlip(servo);

        servo = hwMap.servo.get("rflip");
        SetRightFlip(servo);

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

    public DcMotor getFlip() { return Flip; }

    public DcMotor getIntake() { return Intake; }

    public Servo getLeftFlip() { return LeftFlip; }

    public Servo getRightFlip() { return RightFlip; }


    private void SetFrontL(DcMotor dcMotor) {FrontL = dcMotor; }
    private void SetFrontR(DcMotor dcMotor) {FrontR = dcMotor; }
    private void SetBackL(DcMotor dcMotor) {BackL = dcMotor; }
    private void SetBackR(DcMotor dcMotor) {BackR = dcMotor; }
    private void SetLift(DcMotor dcMotor) {Lift = dcMotor; }
    private void SetSlide(DcMotor dcMotor) {Slide = dcMotor; }
    private void SetFlip(DcMotor dcMotor) {Flip = dcMotor; }
    private void SetIntake(DcMotor dcMotor) {Intake = dcMotor; }
    private void SetLeftFlip(Servo servo) {LeftFlip = servo; }
    private void SetRightFlip(Servo servo) {RightFlip = servo; }

    public void setMotorPower(double fl, double fr, double bl, double br){
        getFrontL().setPower(fl);
        getFrontR().setPower(fr);
        getBackL().setPower(bl);
        getBackR().setPower(br);
    }
}


