package org.firstinspires.ftc.teamcode.test.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.navigation.TurnOMatic;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

import java.util.Timer;
import java.util.TimerTask;

@TeleOp(name = "Test: IMU4", group = "Teleop Test")
@Disabled
public class ImuTest5 extends LinearOpMode {
    BNO055IMU imu = null;
    Mecanum mecanum = null;

    public void runOpMode() {
        PieceOfCake robot = new PieceOfCake();

        robot.init(hardwareMap);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true, telemetry);

        SetupIMU();

        waitForStart();

        //mecanum.TurnLeft(.5, 1200, this);

        while (opModeIsActive()) {
            TurnOMatic turner = new TurnOMatic(imu, mecanum, telemetry, 90);
            turner.Turn(.1, 3000);
            //mecanum.Move(-.0, 0, -.3, .3);

            sleep(5000);

            TurnOMatic turner2 = new TurnOMatic(imu, mecanum, telemetry, 45);
            turner.Turn(.1, 3000);

        }
    }

    private void SetupIMU() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
    }
}
