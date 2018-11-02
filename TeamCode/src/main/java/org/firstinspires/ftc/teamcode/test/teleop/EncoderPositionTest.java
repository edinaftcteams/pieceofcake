package org.firstinspires.ftc.teamcode.test.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name = "Test: Encoder Position", group = "Test")
public class EncoderPositionTest extends LinearOpMode {
    public void runOpMode() {
        PieceOfCake robot = new PieceOfCake();

        robot.init(hardwareMap);

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Flip Location", "%d", robot.getFrontFlip().getCurrentPosition());
            telemetry.addData("Slide Location", "%d", robot.getSlide().getCurrentPosition());
            telemetry.update();
        }
    }
}
