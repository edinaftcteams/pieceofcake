package org.firstinspires.ftc.teamcode.test.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.vision.camera.Camera;
import com.edinaftcrobotics.vision.camera.WebCamCamera;
import com.edinaftcrobotics.vision.tracker.roverruckus.GoldMineralTracker;
import com.edinaftcrobotics.vision.tracker.roverruckus.PictureTracker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="Test: Follow Gold Mineral", group ="Teleop Test")
public class FollowMineralTest extends LinearOpMode {
    static {
        System.loadLibrary("opencv_java3");
    }

    Mecanum _mecanum = null;

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime stopwatch = new ElapsedTime();
        Camera camera = new WebCamCamera(hardwareMap);
        PieceOfCake robot = new PieceOfCake();

        robot.init(hardwareMap);
        camera.activate();
        GoldMineralTracker mineralTracker = new GoldMineralTracker(camera);

        _mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR());

        waitForStart();
        while (opModeIsActive()) {
            idle();
            if (mineralTracker.getGoldMineralLocation()) {
                telemetry.addData("Location: ", mineralTracker.getXPosition());
                telemetry.addData("Aligned: ", mineralTracker.aligned());
                    if (mineralTracker.getXPosition() > 400) {
                        _mecanum.SlideRight(0.2);
                    }
                    else if (mineralTracker.getXPosition() < 400 && mineralTracker.getXPosition() > 300) {
                        //_mecanum.MoveForward(0.2);
                        _mecanum.Stop();
                    }
                    else if (mineralTracker.getXPosition() < 300) {
                        _mecanum.SlideLeft(0.2);
                    }
                    else {
                        _mecanum.Stop();
                    }
            } else {
                telemetry.addData("Object Not Found", "");
                _mecanum.Stop();
            }

            telemetry.update();
        }

        camera.deactivate();
    }
}
