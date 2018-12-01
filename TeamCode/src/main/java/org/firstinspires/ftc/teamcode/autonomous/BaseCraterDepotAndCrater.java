package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;

public class BaseCraterDepotAndCrater extends BaseAutoOpMode{
    private int distanceFromLeftMineral = DrivePerInch * 12;

    protected int FirstDelay = 0;
    protected int SecondDelay = 0;

    public void runOpMode(){

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
                    currentState = MoveForward(driveForwardPosition);
                    break;
                case MOVED_FORWARD:
                    currentState = DriveToMineral(slideLeftPosition, slideRightPosition);
                    break;
                case AT_MINERAL:
                    currentState = PushMineral((int)(DrivePerInch * 8.5));
                    break;
                case MINERAL_PUSHED:
                    currentState = BackAwayFromMIneral((int)(DrivePerInch * 9));
                    break;
                case BACKED_AWAY_FROM_MINERAL:
                    sleep(FirstDelay);
                    currentState = MoveToLeftWall(distanceFromLeftMineral, slideLeftPosition + distanceFromLeftMineral,
                            slideLeftPosition + slideRightPosition + distanceFromLeftMineral);
                    break;
                case AT_LEFT_WALL:
                    currentState = TurnLeftTowardsDepot();
                    break;
                case TURNED_TOWARDS_DEPOT:
                    currentState = MoveTowardsDepot();
                    break;
                case AT_DEPOT:
                    currentState = DropMarker();
                    break;
                case DROPPED_MARKER:
                    currentState = TurnTowardsCrater();
                    break;
                case FACING_CRATER:
                    sleep(SecondDelay);
                    currentState = DriveTowardsCrater();
                    break;
                case AT_CRATER:
                    currentState = ExtendArm();
                    break;
            }
        }

    }
}
