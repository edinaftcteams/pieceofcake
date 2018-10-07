package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class RobotOpMode extends OpMode {

    private PieceOfCake robot = new PieceOfCake();
    @Override
    public void init(){
        robot.init(hardwareMap);

        robot.getBackL().setDirection(DcMotorSimple.Direction.REVERSE);
        robot.getFrontL().setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        ProcessMecanumV1();

        telemetry.addData("Left front: ", "%d", robot.getFrontL().getCurrentPosition());
        telemetry.addData("Right front: ", "%d", robot.getFrontR().getCurrentPosition());
        telemetry.addData("Left back: ", "%d", robot.getBackL().getCurrentPosition());
        telemetry.addData("Right back: ", "%d", robot.getBackR().getCurrentPosition());

        telemetry.update();
    }

    private void ProcessMecanumV1(){
        final double x = Math.pow(-gamepad1.left_stick_x, 1.0);
        final double y = Math.pow(gamepad1.left_stick_y, 1.0);

        final double rotation = Math.pow(gamepad1.right_stick_x, 3.0);
        final double direction = Math.atan2(x, y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double fl = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double fr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double bl = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double br = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        robot.setMotorPower(fl, fr, bl, br);
    }
}