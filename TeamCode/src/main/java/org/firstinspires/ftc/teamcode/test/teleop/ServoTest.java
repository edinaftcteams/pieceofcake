package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test: Servo", group = "Teleop Test")
public class ServoTest extends LinearOpMode {
    public void runOpMode() {
        CRServo ts = hardwareMap.crservo.get("ts");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a){
                ts.setPower(1);
            } else if (gamepad1.b){
                ts.setPower(-1);
            }
            else {
                ts.setPower(0);
            }
        }

    }
}
