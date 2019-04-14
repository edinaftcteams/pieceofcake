package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;

@Autonomous(name="Depot And Crater with Mineral", group="Autonomous")
//@Disabled
public class DepotAndCraterWithMineralAutoOpMode extends BaseAutoOpMode {
    private int distanceFromLeftMineral = DrivePerInch * 21;

    public void runOpMode() {
        //
        // get the robot setup and ready to run
        //
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        while (opModeIsActive() && (currentState != AutonomousStates.BACKED_AWAY_FROM_MINERAL)) {
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
                    currentState = PickUpAndDepositMineral();
                case BACKED_AWAY_FROM_MINERAL:
                    mecanum.SlideLeft2(1, slideLeftPosition + distanceFromLeftMineral - (DrivePerInch * 2), this);

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
        }


        ShutdownTFOD();
    }
}
