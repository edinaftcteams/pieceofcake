package org.firstinspires.ftc.teamcode.test;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.navigation.RevImu;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name = "Test: IMU", group = "Test")
public class ImuTest extends LinearOpMode {
    public void runOpMode() {
        PieceOfCake robot = new PieceOfCake();
        RevImu revImu = null;
        boolean isTurning = false;
        robot.init(hardwareMap);
        Mecanum mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR());

        try {
            revImu = new RevImu(hardwareMap, telemetry);
        } catch (Exception ex) {

        }

        revImu.start();
        waitForStart();

        while (this.opModeIsActive())
        {
            if (isTurning) {
                if (Math.abs(revImu.getAngle()) > 90) {

                }
                isTurning = false;
            } else {
                if (gamepad1.x) {
                    revImu.resetAngle();
                    isTurning = true;
                    mecanum.TurnLeft(.2);
                }

                if (gamepad1.b) {
                    revImu.resetAngle();
                    isTurning = true;
                    mecanum.TurnRight(.2);
                }
            }
            telemetry.addData("Current Angle", "%f", revImu.getAngle());
            telemetry.update();
        }

        revImu.stop();
    }
}
