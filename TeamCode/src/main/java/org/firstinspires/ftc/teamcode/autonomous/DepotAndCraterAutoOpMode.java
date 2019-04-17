package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;

@Autonomous(name="Depot And Crater", group="Autonomous")
@Disabled
public class DepotAndCraterAutoOpMode extends BaseAutoOpMode {
    private int distanceFromLeftMineral = DrivePerInch * 21;

    public void runOpMode() {
        //
        // get the robot setup and ready to run
        //
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getBackLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getBackLift().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // hang the robot
        currentState = Latch();

        InitGyro();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    // look for the mineral and tell us what the camera sees
                    LocateTFMineral();
                    telemetry.addData("Mineral Location", mineralLocation);
                    telemetry.addData("Last Recognition", LastRecognition);
                    telemetry.addData("Angle", GetImuAngle());
                    telemetry.addData("Latch Power: ", robot.getBackLift().getPower());
                    telemetry.addData("Flip Position", robot.getFrontFlip().getCurrentPosition());
                    telemetry.addData("Lift Position", robot.getBackLift().getCurrentPosition());
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }

        //
        // Our state machine for what we do when we are landing from the depot side.
        //
        while (opModeIsActive() && (currentState != AutonomousStates.INTAKE_ON)) {
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
                    if (mineralLocation == MineralLocation.RIGHT) {
                        currentState = AutonomousStates.MINERAL_RIGHT;
                    } else {
                        currentState = AutonomousStates.MINERALS_CENTER_LEFT;
                    }
                    break;
                case MINERAL_RIGHT:
                    currentState = DriveToRightMineral(slideRightPosition);
                    break;
                case MINERALS_CENTER_LEFT:
                    currentState = DropMarkerForLeftOrCenterMineral();
                    break;
                case DROPPED_MARKER:
                    currentState = DriveToLeftOrCenterMineral(slideLeftPosition);
                    break;
                case AT_MINERAL:
                    currentState = PushLeftOrCenterMineral((int)(DrivePerInch *
                            (PushMineralDistance+1)));
                    break;
                case AT_RIGHT_MINERAL:
                    currentState = PushRightMineral((int)(DrivePerInch * (PushMineralDistance+1)));
                    break;
                case MINERAL_PUSHED:
                    currentState = BackAwayFromLeftOrCenterMineral((int)(DrivePerInch *
                            BackAwayFromMineralDistance));
                    break;
                case MINERAL_RIGHT_PUSHED:
                    currentState = BackAwayFromRightMineral((int)(DrivePerInch *
                            BackAwayFromMineralDistance));
                    break;
                case BACKED_AWAY_FROM_RIGHT_MINERAL:
                    currentState = MoveBackToMiddleFromRightMineral(slideRightPosition);
                    break;
                case BACK_AT_CENTER:
                    currentState = DropMarkerForRightMineral();
                    break;

                case BACKED_AWAY_FROM_MINERAL:
                    currentState = MoveToLeftWall(distanceFromLeftMineral,
                            slideLeftPosition + distanceFromLeftMineral,
                            slideLeftPosition  + distanceFromLeftMineral,
                            .5);
                    robot.getFrontFlip().setTargetPosition(800);
                    robot.getFrontFlip().setPower(.7);

                    while (robot.getFrontFlip().isBusy() && opModeIsActive()) {
                        idle();
                    }

                    break;
                case AT_LEFT_WALL:
                    currentState = TurnLeftTowardsCrater2();
                    break;
                case TURNED_TOWARDS_CRATER:
                    currentState = ExtendArm();
                    break;
                case ARM_EXTENDED:
                    currentState = BringLiftDown();
                    break;
                case LIFT_DOWN:
                    currentState = DropFrontFlip();
                    break;
                case FLIP_DOWN:
                    currentState = TurnIntakeOn();
                    break;
            }
        }


        ShutdownTFOD();
    }
}
