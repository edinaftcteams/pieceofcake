package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;

@Autonomous(name="Crater with Mineral", group="Autonomous")
//@Disabled
public class CraterWithMineralOpMode extends BaseAutoOpMode {
    private int distanceFromLeftMineral = DrivePerInch * 21;

    public void runOpMode() {
        //
        // get the robot setup and ready to run
        //
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getFrontLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontLift().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);
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
        while (opModeIsActive() && (currentState != AutonomousStates.INTAKE_ON)) {
            switch (currentState) {
                case LATCHED:
                    currentState = Drop();
                    break;
                case DROPPED:
                    currentState = MoveLeftOffLatch();
                    break;
                case MOVED_OFF_LATCH:
                    currentState = MoveForwardAndSlideBackToCenter(driveForwardCraterPosition);
                    break;
                case MOVED_BACK_TO_CENTER:
                    currentState = PickUpAndDepositMineral(true);
                    break;
                case BACKED_AWAY_FROM_MINERAL:
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
            telemetry.addData("Mineral Location", mineralLocation);
            telemetry.addData("Last Recognition", LastRecognition);
            telemetry.addData("Angle", GetImuAngle());
            telemetry.addData("Latch Power: ", robot.getBackLift().getPower());
            telemetry.addData("Flip Position", robot.getFrontFlip().getCurrentPosition());
            telemetry.addData("Lift Position ALEX", robot.getFrontLift().getCurrentPosition());
            telemetry.update();
        }


        ShutdownTFOD();
    }
}
