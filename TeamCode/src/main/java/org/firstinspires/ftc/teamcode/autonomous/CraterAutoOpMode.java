package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;

@Autonomous(name = "Crater",group = "Autonomous")
public class CraterAutoOpMode extends BaseAutoOpMode {

    public void runOpMode(){
        AutonomousStates currentState = AutonomousStates.START;

        InitRobot();
        InitGyro();

        //InitSetup();

        robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        currentState = Latch();

        waitForStart();

        return;
/*
        LocateTFMineral();

        while (opModeIsActive() && (currentState != AutonomousStates.FLIP_AT_BACK)) {
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
                    currentState = PushMineral(DrivePerInch * 9);
                    break;
                case MINERAL_PUSHED:
                    currentState = ExtendArm();
                    break;
                case ARM_EXTENDED:
                    currentState = FlipToBack();
                    break;
            }
        }
        */
    }
}
