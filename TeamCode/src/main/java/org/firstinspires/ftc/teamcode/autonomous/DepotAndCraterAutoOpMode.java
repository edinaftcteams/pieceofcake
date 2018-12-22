package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;

@Autonomous(name="Depot And Crater", group="Autonomous")
//@Disabled
public class DepotAndCraterAutoOpMode extends BaseAutoOpMode {
    private int distanceFromLeftMineral = DrivePerInch * 12;

    public void runOpMode() {
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();
        InitGyro();

        //InitSetup();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentState = Latch();

        waitForStart();

        LocateTFMineral();

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
                    currentState = PushMineral((int)(DrivePerInch * 8.5));
                    break;
                case MINERAL_PUSHED:
                    currentState = BackAwayFromMIneral((int)(DrivePerInch * 9));
                    break;
                case BACKED_AWAY_FROM_MINERAL:
                    currentState = TurnTowardsLeftWall();
                    break;
                case TURNED_TOWARDS_LEFT_WALL:
                    currentState = DriveToLeftWall(distanceFromLeftMineral, slideLeftPosition + distanceFromLeftMineral,
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
