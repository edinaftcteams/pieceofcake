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

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
public class RobotOpMode extends OpMode {
    private PieceOfCake robot = new PieceOfCake();
    private Mecanum mecanum = null;
    private Camera camera = null;
    private PictureTracker pictureTracker = null;

    @Override
    public void init(){
        robot.init(hardwareMap);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR());

        camera = new BackPhoneCamera();
        camera.activate();
        pictureTracker = new PictureTracker(camera, 110, 200, 0);
        pictureTracker.startTracking();
    }

    @Override
    public void loop() {
        mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Left front: ", "%d", robot.getFrontL().getCurrentPosition());
        telemetry.addData("Right front: ", "%d", robot.getFrontR().getCurrentPosition());
        telemetry.addData("Left back: ", "%d", robot.getBackL().getCurrentPosition());
        telemetry.addData("Right back: ", "%d", robot.getBackR().getCurrentPosition());

        Triple trackableObject = pictureTracker.getTrackableObject(telemetry);

        if (trackableObject != null) {
            telemetry.addData("Visible Target", trackableObject.PictureName);
            telemetry.addData("Pos (in) ", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    trackableObject.Point.x, trackableObject.Point.y, trackableObject.Point.z);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f}", trackableObject.Orientation.firstAngle,
                    trackableObject.Orientation.secondAngle, trackableObject.Orientation.thirdAngle);
        } else {
            telemetry.addData("Picture", "not found");
        }

        telemetry.update();
    }
}