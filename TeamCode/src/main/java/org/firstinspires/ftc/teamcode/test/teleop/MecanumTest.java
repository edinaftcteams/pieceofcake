package org.firstinspires.ftc.teamcode.test.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name = "Test: Mecanum", group = "Teleop Test")
@Disabled
public class MecanumTest extends LinearOpMode {
    private Mecanum _mecanum;

    public MecanumTest() {
    }

    public void runOpMode() {
        PieceOfCake robot = new PieceOfCake();

        robot.init(hardwareMap);

        _mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR());

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                _mecanum.SlideLeft(.2);
            } else if (gamepad1.dpad_right) {
                _mecanum.SlideRight(.2);
            } else if (gamepad1.dpad_up) {
                _mecanum.MoveForward(.2);
            } else if (gamepad1.dpad_down) {
                _mecanum.MoveBackwards(.2);
            } else if (gamepad1.x) {
                _mecanum.MoveNW(.2); // SE
            } else if (gamepad1.y) {
                _mecanum.MoveNE(.2); // SW
            } else if (gamepad1.a) {
                _mecanum.MoveSW(.2); // NW
            } else if (gamepad1.b) {
                _mecanum.MoveSE(.2); // NE
            } else if (gamepad1.right_bumper) {
                _mecanum.TurnRight(.2);
            } else if (gamepad1.left_bumper) {
                _mecanum.TurnLeft(.2);
            } else {
                _mecanum.Stop();
            }
        }
    }
}
