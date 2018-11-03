package org.firstinspires.ftc.teamcode.autonomous;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;

@Autonomous(name="Blue Gold", group="Autonomous")
public class BlueGoldAutoOpMode extends BaseAutoOpMode {

    public void runOpMode()throws InterruptedException{
        AutonomousStates currentState = AutonomousStates.START;
        int slideRightPosition = DrivePerInch * 8;
        int slideLeftPosition = DrivePerInch * 8;
        int knockForwardPosition = DrivePerInch * 8;
        int driveForwardPosition = DrivePerInch * 12;
        boolean yPressed = false;
        boolean aPressed = false;
        boolean dPadLeftPressed = false;
        boolean dPadRightPressed = false;
        boolean dPadUpPressed = false;
        boolean dPadDownPressed = false;
        boolean bumperLeftPressed = false;
        boolean bumperRightPressed = false;
        double latchPower = .3;
        InitRobot();

        while (!gamepad1.x) {
            if (gamepad1.y && !yPressed) {
                driveForwardPosition += (int) (DrivePerInch * .5);
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            if (gamepad1.a && !aPressed) {
                driveForwardPosition -= (int) (DrivePerInch * .5);
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }

            if (gamepad1.dpad_left && !dPadLeftPressed) {
                slideRightPosition -= (int) (DrivePerInch * .5);
                dPadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dPadLeftPressed = false;
            }

            if (gamepad1.dpad_right && !dPadRightPressed) {
                slideRightPosition += (int) (DrivePerInch * .5);
                dPadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dPadRightPressed = false;
            }

            if (gamepad1.dpad_up && !dPadUpPressed) {
                knockForwardPosition += (int) (DrivePerInch * .5);
                dPadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dPadUpPressed = false;
            }

            if (gamepad1.dpad_down && !dPadDownPressed) {
                knockForwardPosition -= (int) (DrivePerInch * .5);
                dPadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dPadDownPressed = false;
            }

            if (gamepad1.left_bumper && !bumperLeftPressed) {
                slideLeftPosition -= (int) (DrivePerInch * .5);
                bumperLeftPressed = true;
            } else if (!gamepad1.left_bumper) {
                bumperLeftPressed = false;
            }

            if (gamepad1.right_bumper && !bumperRightPressed) {
                slideLeftPosition += (int) (DrivePerInch * .5);
                bumperRightPressed = true;
            } else if (!gamepad1.right_bumper) {
                bumperRightPressed = false;
            }

            telemetry.addData("DPad L/R controls slide right position, currently", "%d", slideRightPosition);
            telemetry.addData("DPad U/D controls knock forward position, currently", "%d", knockForwardPosition);
            telemetry.addData("Bumper L/R controls slide left position, currently", "%d", slideLeftPosition);
            telemetry.addData("Buttons Y/A controls drive forward position, currently", "%d", driveForwardPosition);
            telemetry.addData("Press X", "to lock in settings");
            telemetry.update();
        }

        robot.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!gamepad1.x) {
            if (gamepad1.left_bumper && !bumperLeftPressed) {
                latchPower -= .1;
                bumperLeftPressed = true;
            } else if (!gamepad1.left_bumper) {
                bumperLeftPressed = false;
            }

            if (gamepad1.right_bumper && !bumperRightPressed) {
                latchPower += .1;
                bumperRightPressed = true;
            } else if (!gamepad1.right_bumper) {
                bumperRightPressed = false;
            }

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("Orientation Z, Y, X", "%f %f %f", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
            telemetry.addData("Bumper L/R to control latch power, currently", "%f", latchPower);
            telemetry.addData("Latch Height, currently", "%f", robot.getLift().getCurrentPosition());
            telemetry.addData("Press X", "to lock in settings");
            telemetry.update();
            currentState = Latch(latchPower);
        }

        ActivateTFCamera();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    this.wait();
                    LocateTFMineral();
                    if (mineralLocation == MineralLocation.LEFT) {
                        telemetry.addData("Mineral Location", "Left");
                    } else if (mineralLocation == MineralLocation.MIDDLE) {
                        telemetry.addData("Mineral Location", "Middle");
                    } else if (mineralLocation == MineralLocation.RIGHT) {
                        telemetry.addData("Mineral Location", "Right");
                    } else {
                        telemetry.addData("Mineral Location", "Lost");
                    }
                    telemetry.update();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        LocateTFMineral();

        DeactivateTFCamera();

        while (opModeIsActive() && (currentState != AutonomousStates.MINERAL_PUSHED)) {
            switch (currentState) {
                case LATCHED:
                    currentState = Drop();
                    break;
                case DROPPED:
                    currentState = MoveForward(driveForwardPosition);
                    break;
                case MOVED_FORWARD:
                    currentState = DropMarker();
                    break;
                case DROPPED_MARKER:
                    currentState = DriveToMineral(slideLeftPosition, slideRightPosition);
                    break;
                case AT_MINERAL:
                    currentState = PushMineral(knockForwardPosition);
                    break;
            }
        }
    }
}
