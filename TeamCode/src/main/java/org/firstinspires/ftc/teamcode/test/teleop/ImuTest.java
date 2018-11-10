package org.firstinspires.ftc.teamcode.test.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.navigation.RevImu;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name = "Test: IMU", group = "Teleop Test")
@Disabled
public class ImuTest extends LinearOpMode {
    public void runOpMode() {
        PieceOfCake robot = new PieceOfCake();
        RevImu revImu = null;
        boolean isTurning = false;
        boolean turningLeft = false;
        robot.init(hardwareMap);
        Mecanum mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true);

        try {
            revImu = new RevImu(hardwareMap, telemetry);
        } catch (Exception ex) {

        }

        revImu.start();
        waitForStart();

        while (this.opModeIsActive())
        {
            idle();
            double turnPower = .2;
            double currentAngle = Math.abs(revImu.getAngle());
            double power = (85 - currentAngle )/ 90;

            if (isTurning) {
                if (currentAngle > 80) {
                    mecanum.Stop();
                    isTurning = false;
                }
            } else {
                if (gamepad1.x) {
                    revImu.resetAngle();
                    isTurning = true;
                    turningLeft = true;
                    mecanum.TurnLeft(turnPower);
                }

                if (gamepad1.b) {
                    revImu.resetAngle();
                    isTurning = true;
                    turningLeft = false;
                    mecanum.TurnRight(turnPower);
                }
            }
            telemetry.addData("Current Angle", "%f", currentAngle);
            telemetry.addData("Current Power", "%f", power);
            telemetry.addData("Turn Power", "%f", turnPower);
            telemetry.update();
        }

        revImu.stop();
    }
}
