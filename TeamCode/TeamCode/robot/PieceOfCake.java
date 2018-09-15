package org.firstinspires.ftc.teamcode;
//imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class PieceOfCake {
    private DcMotor FrontR = null;
    private DcMotor FrontL = null;
    private DcMotor BackR = null;
    private DcMotor BackL = null;


    private HardwareMap hwMap = null;


    //constructor
    public PieceOfCake() {

    }
    public void init(HardwareMap ahwMap) {
        DcMotor dcMotor = null;

        hwMap = ahwMap;

        dcMotor = hwMap.dcMotor.get("frontleftmotor");
        SetFrontL(dcMotor);

        dcMotor = hwMap.dcMotor.get("frontrightmotor");
        SetFrontR(dcMotor);

        dcMotor = hwMap.dcMotor.get("backleftmotor");
        SetBackL(dcMotor);

        dcMotor = hwMap.dcMotor.get("backrightmotor");
        SetBackR(dcMotor);



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

    private void SetFrontL(DcMotor dcMotor) {FrontL = dcMotor; }
    private void SetFrontR(DcMotor dcMotor) {FrontR = dcMotor; }
    private void SetBackL(DcMotor dcMotor) {BackL = dcMotor; }
    private void SetBackR(DcMotor dcMotor) {BackR = dcMotor; }
}


