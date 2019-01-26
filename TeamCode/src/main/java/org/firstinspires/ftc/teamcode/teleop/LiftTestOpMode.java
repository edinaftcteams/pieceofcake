package org.firstinspires.ftc.teamcode.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="LiftTestOpMode", group="Teleop")
public class LiftTestOpMode extends OpMode {
    private PieceOfCake robot = new PieceOfCake();

    @Override
    public void init(){
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Back Lift", "%d", robot.getBackLift().getCurrentPosition());
        telemetry.addData("Front Lift", "%d", robot.getFrontLift().getCurrentPosition());
        telemetry.addData("Front Flip", "%d", robot.getFrontFlip().getCurrentPosition());
        telemetry.addData("Lf, rf, lb, rb: ", "%d %d %d %d", robot.getFrontL().getCurrentPosition(),
                robot.getFrontR().getCurrentPosition(), robot.getBackL().getCurrentPosition(),
                robot.getBackR().getCurrentPosition());
        telemetry.addData("y:", "%s", gamepad1.y);
        telemetry.addData("a:", "%s", gamepad1.a);

        ProcessLift();
        ProcessIntake();

        telemetry.update();
    }
    private void ProcessLift() {
        robot.getBackLift().setPower(-gamepad2.left_stick_y); // TODO - change these by either flipping the negative or adding/removing a negative
        robot.getFrontLift().setPower(gamepad2.left_stick_y);
    }

    private void ProcessIntake() {
        if (gamepad2.a) {
            robot.getIntake().setPower(1);
        } else if (gamepad2.y) {
            robot.getIntake().setPower(-1);
        } else {
            robot.getIntake().setPower(0);
        }
    }
}
