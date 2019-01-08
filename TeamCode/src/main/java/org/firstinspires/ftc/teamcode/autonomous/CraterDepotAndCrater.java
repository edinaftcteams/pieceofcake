package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;

@Autonomous(name="CraterDepotCrater", group="Autonomous")
//@Disabled
public class CraterDepotAndCrater extends BaseAutoOpMode{
    private int distanceFromLeftMineral = (int)(DrivePerInch * 19.5);

    public void runOpMode(){

        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();
        InitGyro();

        //InitSetup();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentState = Latch();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentState = Latch();

        while (!isStarted()) {
            synchronized (this) {
                try {
                    LocateTFMineral();
                    telemetry.addData("Mineral Location", mineralLocation);
                    telemetry.addData("Last Recognition", LastRecognition);
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
                    currentState = MoveForward(driveForwardPosition);
                    break;
                case MOVED_FORWARD:
                    currentState = DriveToMineralOffLeftOffset(slideLeftPosition, slideRightPosition);
                    break;
                case AT_MINERAL:
                    currentState = PushMineral((int)(DrivePerInch * PushMineralDistance));
                    break;
                case MINERAL_PUSHED:
                    currentState = BackAwayFromMIneral((int)(DrivePerInch * BackAwayFromMineralDistance));
                    break;
/*
                case BACKED_AWAY_FROM_MINERAL:
                    currentState = TurnTowardsLeftWall();
                    break;
                case TURNED_TOWARDS_LEFT_WALL:
                    currentState = DriveToLeftWall(distanceFromLeftMineral, slideLeftPosition + distanceFromLeftMineral,
                            slideLeftPosition + slideRightPosition + distanceFromLeftMineral);
                    break;
                case AT_LEFT_WALL:
                    currentState = TurnLeftTowardsDepot();
                    break;
                case TURNED_TOWARDS_DEPOT:
                    currentState = MoveTowardsDepot();
                    break;
                case AT_DEPOT:
                    currentState = AutonomousStates.DROPPED_MARKER; //DropMarker();
                    break;
                case DROPPED_MARKER:
                    currentState = TurnTowardsCrater();
                    break;
                case FACING_CRATER:
                    currentState = DriveTowardsCrater();
                    break;
                    */
                case BACKED_AWAY_FROM_MINERAL:
                    currentState = ExtendArm();
                    break;
            }
        }
    }
}
