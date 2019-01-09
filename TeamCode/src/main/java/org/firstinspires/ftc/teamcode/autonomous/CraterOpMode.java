package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;

@Autonomous(name="Crater", group="Autonomous")
//@Disabled
public class CraterOpMode extends BaseAutoOpMode{
    private int distanceFromLeftMineral = (int)(DrivePerInch * 19.5);

    public void runOpMode(){

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
                case BACKED_AWAY_FROM_MINERAL:
                    currentState = ExtendArm();
                    break;
            }
        }
    }
}
