package org.firstinspires.ftc.teamcode.test.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.vision.camera.BackPhoneCamera;
import com.edinaftcrobotics.vision.camera.Camera;
import com.edinaftcrobotics.vision.camera.WebCamCamera;
import com.edinaftcrobotics.vision.tracker.roverruckus.PictureTracker;
import com.edinaftcrobotics.vision.utils.Triple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

@TeleOp(name="Test: Follow Picture", group ="Teleop Test")
@Disabled
public class FollowPictureTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PieceOfCake robot = new PieceOfCake();
        double targetPoint = 0;
        robot.init(hardwareMap);

        Mecanum drivetrain = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true);

        Camera camera = new BackPhoneCamera();
        PictureTracker pictureTracker = null;

        camera.activate();
        pictureTracker = new PictureTracker(camera, 110, 200, 0);
        pictureTracker.startTracking();

        waitForStart();
        while (opModeIsActive()) {
            Triple trackableObject = pictureTracker.getTrackableObject(telemetry);

            if (trackableObject != null) {
                telemetry.addData("Visible Target", trackableObject.PictureName);
                telemetry.addData("Pos (in) ", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        trackableObject.Point.x, trackableObject.Point.y, trackableObject.Point.z);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f}", trackableObject.Orientation.firstAngle,
                        trackableObject.Orientation.secondAngle, trackableObject.Orientation.thirdAngle);
                if ((trackableObject.PictureName == "Red-Footprint") || (trackableObject.PictureName == "Blue-Rover")) {
                    targetPoint = Math.abs(trackableObject.Point.y);
                } else {
                    targetPoint = Math.abs(trackableObject.Point.x);
                }

                if (targetPoint < 58) {
                    telemetry.addData("Moving robot", "closer");
                    drivetrain.SlideLeft(.8);
                } else {
                    telemetry.addData("Moving robot", "stopped");
                    drivetrain.SlideLeft(0);
                }
            } else {
                drivetrain.SlideLeft(0);
                telemetry.addData("Picture", "not found");
            }

            telemetry.update();
        }
    }
}
