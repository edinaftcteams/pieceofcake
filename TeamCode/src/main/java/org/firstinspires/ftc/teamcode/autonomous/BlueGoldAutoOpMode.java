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
        int slideRightPosition = 0;
        int slideLeftPosition = 0;
        int knockForwardPosition = 0;
        int driveForwardPosition = 0;
        boolean yPressed = false;
        boolean aPressed = false;
        boolean dPadLeftPressed = false;
        boolean dPadRightPressed = false;
        boolean dPadUpPressed = false;
        boolean dPadDownPressed = false;
        boolean bumperLeftPressed = false;
        boolean bumperRightPressed = false;
        double latchPower = 0;
        InitRobot();

        while (!gamepad1.x) {
            if (gamepad1.y && !yPressed) {
                driveForwardPosition += 50;
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            if (gamepad1.a && !aPressed) {
                driveForwardPosition -= 50;
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }

            if (gamepad1.dpad_left && !dPadLeftPressed) {
                slideRightPosition -= 50;
                dPadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dPadLeftPressed = false;
            }

            if (gamepad1.dpad_right && !dPadRightPressed) {
                slideRightPosition += 50;
                dPadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dPadRightPressed = false;
            }

            if (gamepad1.dpad_up && !dPadUpPressed) {
                knockForwardPosition += 50;
                dPadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dPadUpPressed = false;
            }

            if (gamepad1.dpad_down && !dPadDownPressed) {
                knockForwardPosition -= 50;
                dPadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dPadDownPressed = false;
            }

            if (gamepad1.left_bumper && !bumperLeftPressed) {
                slideLeftPosition -= 50;
                bumperLeftPressed = true;
            } else if (!gamepad1.left_bumper) {
                bumperLeftPressed = false;
            }

            if (gamepad1.right_bumper && !bumperRightPressed) {
                slideLeftPosition += 50;
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

        ActivateCamera();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    this.wait();
                    LocateMineral();
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

        DeactivateCamera();

        while (opModeIsActive() && (currentState != AutonomousStates.DROPPED_MARKER)) {
            switch (currentState) {
                case LATCHED:
                    currentState = Drop();
                    break;
                case DROPPED:
                    currentState = DriveToMineral(driveForwardPosition, slideLeftPosition, slideRightPosition);
                    break;
                case AT_MINERAL:
                    currentState = PushMineral(knockForwardPosition);
                    break;
                case MINERAL_PUSHED:
                    currentState = MoveToMiddle(slideLeftPosition, slideRightPosition);
                    break;
                case AT_MIDDLE:
                    currentState = DropMarker();
                    break;
            }
        }
    }
}
