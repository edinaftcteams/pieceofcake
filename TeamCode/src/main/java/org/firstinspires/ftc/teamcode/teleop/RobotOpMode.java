package org.firstinspires.ftc.teamcode.teleop;

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
    Camera camera = null;
    PictureTracker pictureTracker = null;

    @Override
    public void init(){
        robot.init(hardwareMap);

        robot.getBackL().setDirection(DcMotorSimple.Direction.REVERSE);
        robot.getFrontL().setDirection(DcMotorSimple.Direction.REVERSE);
        camera = new BackPhoneCamera();
        camera.activate();
        pictureTracker = new PictureTracker(camera, 110, 200, 0);
        pictureTracker.startTracking();
    }

    @Override
    public void loop() {
        ProcessMecanumV1();

        telemetry.addData("Left front: ", "%d", robot.getFrontL().getCurrentPosition());
        telemetry.addData("Right front: ", "%d", robot.getFrontR().getCurrentPosition());
        telemetry.addData("Left back: ", "%d", robot.getBackL().getCurrentPosition());
        telemetry.addData("Right back: ", "%d", robot.getBackR().getCurrentPosition());

        Triple trackableObject = pictureTracker.getTrackableObject();

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

    private void ProcessMecanumV1(){
        final double x = Math.pow(-gamepad1.left_stick_x, 3.0);
        final double y = Math.pow(gamepad1.left_stick_y, 3.0);

        final double rotation = Math.pow(-gamepad1.right_stick_x, 3.0);
        final double direction = Math.atan2(x, y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y)) * 1.4;

        telemetry.addData("Speed, direction, rotation, sin, cos", "%f %f %f %f %f", speed, direction, rotation, Math.sin(direction + Math.PI / 4.0),
                Math.cos(direction + Math.PI / 4.0));

        final double fl = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double fr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double bl = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double br = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        telemetry.addData("Power", "%f %f %f %f", fl, fr, bl, br);

        robot.setMotorPower(fl, fr, bl, br);
    }
}