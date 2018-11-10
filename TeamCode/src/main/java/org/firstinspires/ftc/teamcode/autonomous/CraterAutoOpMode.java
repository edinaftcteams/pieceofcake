package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;

@Autonomous(name = "Crater",group = "Autonomous")
@Disabled
public class CraterAutoOpMode extends BaseAutoOpMode {

    public void runOpMode(){
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();
        InitGyro();

        InitSetup();

        robot.getLift().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getLift().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentState = Latch();

        SearchForTFMineral();

        while (opModeIsActive() && (currentState != AutonomousStates.DROPPED)) {
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
                    currentState = PushMineral(knockForwardPosition);
                    break;
                case MINERAL_PUSHED:
                    currentState = ExtendArm();
                    break;
                case ARM_EXTENDED:
                    currentState = FlipToBack();
                    break;
            }
        }
    }
}
