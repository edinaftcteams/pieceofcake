package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;

@Autonomous(name="Depot And Crater", group="Autonomous")
//@Disabled
public class DepotAndCraterAutoOpMode extends BaseAutoOpMode {
    private int distanceFromLeftMineral = DrivePerInch * 21;

    public void runOpMode() {
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();
        InitGyro();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentState = Latch();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    LocateTFMineral();
                    telemetry.addData("Mineral Location", mineralLocation);
                    telemetry.addData("Last Recognition", LastRecognition);
                    telemetry.addData("Angle", GetImuAngle());
                    telemetry.addData("Latch Power: ", robot.getBackLift().getPower());
                    telemetry.addData("Flip Position", robot.getFrontFlip().getCurrentPosition());
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        while (opModeIsActive() && (currentState != AutonomousStates.ARM_EXTENDED)) {
            switch (currentState) {
                case LATCHED:
                    currentState = Drop();
                    break;
                case DROPPED:
                    currentState = MoveLeftOffLatch();
                    break;
                case MOVED_OFF_LATCH:
                    currentState = MoveForwardAndSlideBackToCenter(driveForwardPosition);
                    break;
                case MOVED_BACK_TO_CENTER:
                    currentState = DropMarker();
                    break;
                case DROPPED_MARKER:
                    currentState = DriveToMineral(slideLeftPosition, slideRightPosition);
                    break;
                case AT_MINERAL:
                    currentState = PushMineral((int)(DrivePerInch * PushMineralDistance));
                    break;
                case MINERAL_PUSHED:
                    currentState = BackAwayFromMIneral((int)(DrivePerInch * BackAwayFromMineralDistance));
                    break;
                case BACKED_AWAY_FROM_MINERAL:
                    currentState = MoveToLeftWall(distanceFromLeftMineral, slideLeftPosition + distanceFromLeftMineral,
                            slideLeftPosition + slideRightPosition + distanceFromLeftMineral);
                    robot.getFrontFlip().setTargetPosition(800);
                    robot.getFrontFlip().setPower(.7);

                    while (robot.getFrontFlip().isBusy()) {
                        idle();
                    }

                    break;
                case AT_LEFT_WALL:
                    currentState = TurnLeftTowardsCrater2();
                    break;
                case TURNED_TOWARDS_CRATER:
                    currentState = ExtendArm();
                    break;
            }

            telemetry.addData("Angle", GetImuAngle());
            telemetry.update();
        }

        ShutdownTFOD();
    }
}
