package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;

@Autonomous(name="Depot And Crater", group="Autonomous")
public class DepotAndCraterAutoOpMode extends BaseAutoOpMode {
    private int distanceFromLeftMineral = DrivePerInch * 12;

    public void runOpMode() {
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();
        InitGyro();

        //InitSetup();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //currentState = Latch();
        currentState = AutonomousStates.LATCHED;

        waitForStart();

        LocateTFMineral();
        mineralLocation = MineralLocation.LEFT;

        while (opModeIsActive() && (currentState != AutonomousStates.AT_CRATER)) {
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
                    currentState = PushMineral(DrivePerInch * 7);
                    break;
                case MINERAL_PUSHED:
                    currentState = BackAwayFromMIneral(DrivePerInch * 7);
                    break;
                case BACKED_AWAY_FROM_MINERAL:
                    currentState = MoveToLeftWall(distanceFromLeftMineral, slideLeftPosition + distanceFromLeftMineral,
                            slideLeftPosition + slideRightPosition + distanceFromLeftMineral);
                    break;
                case AT_LEFT_WALL:
                    currentState = TurnLeftTowardsCrater();
                    break;
                case TURNED_TOWARDS_CRATER:
                    currentState = ExtendArm();
                    break;
                case ARM_EXTENDED:
                    currentState = MoveTowardsCrater();
                    break;
            }
        }

        ShutdownTFOD();
    }
}
