package org.firstinspires.ftc.teamcode.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.vision.camera.BackPhoneCamera;
import com.edinaftcrobotics.vision.camera.Camera;
import com.edinaftcrobotics.vision.camera.WebCamCamera;
import com.edinaftcrobotics.vision.tracker.roverruckus.PictureTracker;
import com.edinaftcrobotics.vision.utils.Triple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class RobotOpMode extends OpMode {
    private PieceOfCake robot = new PieceOfCake();
    private Mecanum mecanum = null;
    private int MiddleFlip = 0;
    private int BottomFlip = 0;
    private int TopFlip = 0;


    @Override
    public void init(){
        robot.init(hardwareMap);

        robot.getFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR());
    }

    @Override
    public void loop() {
        mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Left front: ", "%d", robot.getFrontL().getCurrentPosition());
        telemetry.addData("Right front: ", "%d", robot.getFrontR().getCurrentPosition());
        telemetry.addData("Left back: ", "%d", robot.getBackL().getCurrentPosition());
        telemetry.addData("Right back: ", "%d", robot.getBackR().getCurrentPosition());

        ProcessArm();
        ProcessIntake();
        ProcessFlip();
        ProcessLift();

    }
    private void ProcessArm() {
        if (gamepad1.left_trigger > 0) {
            robot.getSlide().setPower(1);
        }
        if (gamepad1.right_trigger > 0) {
            robot.getSlide().setPower(-1);
        }
    }
    private void ProcessIntake() {
        if (gamepad1.left_bumper == true) {
            robot.getIntake().setPower(1);
        }
        if (gamepad1.right_bumper == true) {
            robot.getIntake().setPower(-1);
        }
    }
    private void ProcessFlip() {
        if (gamepad1.x || gamepad1.b) {
            robot.getFlip().setTargetPosition(MiddleFlip);
        }
        if (gamepad1.y == true) {
            robot.getFlip().setTargetPosition(TopFlip);
        }
        if (gamepad1.a == true) {
            robot.getFlip().setTargetPosition(BottomFlip);
        }
    }
    private void ProcessLift() {
        if (gamepad2.left_stick_y > 0) {
            robot.getLift().setPower(1);
        }
        if (gamepad1.left_stick_y < -0) {
            robot.getLift().setPower(-1);
        }
    }
}