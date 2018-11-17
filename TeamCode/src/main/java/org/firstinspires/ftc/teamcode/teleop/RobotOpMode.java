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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="Teleop", group="Teleop")
public class RobotOpMode extends OpMode {
    private PieceOfCake robot = new PieceOfCake();
    private boolean intakeYPressed = false;
    private boolean intakeAPressed = false;
    private boolean flipBPressed = false;
    private boolean bumpersPressed = false;
    private boolean intakeInToggledOn = false;
    private boolean intakeOutToggledOn = false;
    private boolean listenToGamePad1 = false;
    private boolean listenToGamePad2 = false;
    private Mecanum mecanum = null;

    @Override
    public void init(){
        robot.init(hardwareMap);

        robot.getTopFlip().setPosition(1);

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true);
    }

    @Override
    public void loop() {
        mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Lift", "%d", robot.getLift().getCurrentPosition());
        telemetry.addData("Front Flip", "%d", robot.getFrontFlip().getCurrentPosition());
        telemetry.addData("Lf, rf, lb, rb: ", "%d %d %d %d", robot.getFrontL().getCurrentPosition(),
                robot.getFrontR().getCurrentPosition(), robot.getBackL().getCurrentPosition(),
                robot.getBackR().getCurrentPosition());
        telemetry.addData("y:", "%s", gamepad1.y);
        telemetry.addData("a:", "%s", gamepad1.a);
        telemetry.addData("intakeYPressed", "%s", intakeYPressed);
        telemetry.addData("intakeAPressed", "%s", intakeAPressed);
        telemetry.addData("intakeInToggledOn", "%s", intakeInToggledOn);
        telemetry.addData("intakeOutToggledOn", "%s", intakeOutToggledOn);

        ProcessSlide();
        ProcessIntake();
        ProcessFrontFlip();
        ProcessLift();
        ProcessTopFlip();

        telemetry.update();
    }
    private void ProcessSlide() {
        if ((gamepad1.left_trigger > 0) || (gamepad2.left_trigger > 0)) {
            robot.getSlide().setPower(.7);
        } else if ((gamepad1.right_trigger > 0) || (gamepad2.right_trigger > 0)) {
            robot.getSlide().setPower(-.5);
        } else {
            robot.getSlide().setPower(0);
        }
    }

    private void ProcessIntake() {
        boolean yValue = false;
        boolean aValue = false;

        if (gamepad1.a || gamepad1.y){
            listenToGamePad1 = true;
        } else if (gamepad2.a || gamepad2.y) {
            listenToGamePad2 = true;
        }

        if (listenToGamePad1) {
            yValue = gamepad1.y;
            aValue = gamepad1.a;
        } else if (listenToGamePad2) {
            yValue = gamepad2.y;
            aValue = gamepad2.a;
        }

        if (yValue && !intakeYPressed) {
            intakeYPressed = true;
            intakeAPressed = false;
        } else if (!yValue && intakeYPressed) {
            if (!intakeInToggledOn) {
                robot.getIntake().setPower(1);
                intakeInToggledOn = true;
                intakeOutToggledOn = false;
            } else {
                robot.getIntake().setPower(0);
                intakeInToggledOn = false;
                intakeOutToggledOn = false;
            }

            listenToGamePad1 = false;
            listenToGamePad2 = false;
            intakeYPressed = false;
            intakeAPressed = false;
        } else if (aValue && !intakeAPressed) {
            intakeAPressed = true;
            intakeYPressed = false;
        } else if (!aValue && intakeAPressed) {
            if (!intakeOutToggledOn) {
                robot.getIntake().setPower(-1);
                intakeInToggledOn = false;
                intakeOutToggledOn = true;
            } else {
                robot.getIntake().setPower(0);
                intakeInToggledOn = false;
                intakeOutToggledOn = false;
            }

            listenToGamePad1 = false;
            listenToGamePad2 = false;
            intakeAPressed = false;
            intakeYPressed = false;
        }
    }

    private void ProcessFrontFlip() {
        if ((gamepad1.left_bumper && gamepad1.right_bumper) || (gamepad2.left_bumper && gamepad2.right_bumper)) {
            bumpersPressed = true;
            robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.getFrontFlip().setPower(0);
        }

        if (bumpersPressed) {
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                robot.getFrontFlip().setPower(.7);
            } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                robot.getFrontFlip().setPower(-.7);
            } else {
                robot.getFrontFlip().setPower(0);
            }
        } else {
            if (gamepad1.x || gamepad2.x) {
                robot.getFrontFlip().setPower(.7);
                robot.getFrontFlip().setTargetPosition(2778);
                flipBPressed = false;
            } else if (gamepad1.b || gamepad2.b) {
                robot.getFrontFlip().setPower(.7);
                robot.getFrontFlip().setTargetPosition(0);
                flipBPressed = true;
            } else if ((!gamepad1.b || !gamepad2.b) && flipBPressed) {
                robot.getFrontFlip().setPower(.5);
                robot.getFrontFlip().setTargetPosition(900);
                flipBPressed = false;
            }

            if (!robot.getFrontFlip().isBusy()) {
                robot.getFrontFlip().setPower(0);
            }
        }
    }

    private void ProcessLift() {
        if (gamepad2.left_stick_y < 0) {
            robot.getLift().setPower(1);
            //robot.getLockServo().setPower(-1);
        } else if (gamepad2.left_stick_y > 0) {
            robot.getLift().setPower(-1);
            //robot.getLockServo().setPower(1);
        } else {
            robot.getLift().setPower(0);
            //robot.getLockServo().setPower(0);
        }
    }

    private void ProcessTopFlip() {
        if (gamepad2.right_stick_y != 0) {
            robot.getTopFlip().setPosition(0);
        } else {
            robot.getTopFlip().setPosition(1);
        }
    }
}