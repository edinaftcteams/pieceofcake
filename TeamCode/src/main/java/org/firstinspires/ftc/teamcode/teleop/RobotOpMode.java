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
    private int DropArmPerDegree = (int)(8640 / 360);
    private int BackFlip = 0;
    private int VerticalFlip = (int)(DropArmPerDegree * 22);
    private int FlatFlip = (int)(DropArmPerDegree * 102.5);
    private int BottomFlip = (int)(DropArmPerDegree * 143.5);
    private boolean topFlipped = false;

    @Override
    public void init(){
        robot.init(hardwareMap);

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getIntake().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.getTopFlip().setPosition(1);
        //robot.getLockServo().setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.getLockServo().setPower(1);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR());
    }

    @Override
    public void loop() {
        mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Front Flip", "%d", robot.getFrontFlip().getCurrentPosition());
        telemetry.addData("Left front: ", "%d", robot.getFrontL().getCurrentPosition());
        telemetry.addData("Right front: ", "%d", robot.getFrontR().getCurrentPosition());
        telemetry.addData("Left back: ", "%d", robot.getBackL().getCurrentPosition());
        telemetry.addData("Right back: ", "%d", robot.getBackR().getCurrentPosition());

        ProcessArm();
        ProcessIntake();
        //ProcessFrontFlip();
        ProcessLift();
        ProcessTopFlip();

        telemetry.update();
    }
    private void ProcessArm() {
        if (gamepad1.left_trigger > 0) {
            robot.getSlide().setPower(.5);
        } else if (gamepad1.right_trigger > 0) {
            robot.getSlide().setPower(-.3);
        } else {
            robot.getSlide().setPower(0);
        }
    }
    private void ProcessIntake() {
        if (gamepad1.left_bumper) {
            robot.getIntake().setPower(1);
        } else if (gamepad1.right_bumper) {
            robot.getIntake().setPower(-1);
        } else {
            robot.getIntake().setPower(0);
        }
    }

    private void ProcessFrontFlip() {
        if (gamepad1.x) {
            // all the way back to move the mineral into the lifter
            robot.getFrontFlip().setTargetPosition(BackFlip);
            robot.getFrontFlip().setPower(.2);
            topFlipped = true;
        } else if (!gamepad1.x && topFlipped) {
            // after letting go of the x button, we want to call move to middle only once
            robot.getFrontFlip().setTargetPosition(VerticalFlip);
            robot.getFrontFlip().setPower(.2);
            topFlipped = false;
        } else if (gamepad1.b) {
            robot.getFrontFlip().setTargetPosition(FlatFlip);
            topFlipped = false;
        } else if (gamepad1.a) {
            // move down to grab a mineral
            robot.getFrontFlip().setTargetPosition(BottomFlip);
            robot.getFrontFlip().setPower(.2);
            topFlipped = false;
        }
    }

    private void ProcessLift() {
        robot.getLift().setPower(-gamepad2.left_stick_y);
    }

    private void ProcessTopFlip() {
        if (gamepad2.right_stick_y > 0) {
            robot.getTopFlip().setPosition(0);
        } else {
            robot.getTopFlip().setPosition(1);
        }
    }
}