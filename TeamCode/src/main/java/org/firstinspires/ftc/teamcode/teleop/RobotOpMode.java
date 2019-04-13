package org.firstinspires.ftc.teamcode.teleop;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.teamcode.robot.PieceOfCake;
import org.opencv.core.Mat;

@TeleOp(name="Teleop", group="Teleop")
public class RobotOpMode extends OpMode {
    private PieceOfCake robot = new PieceOfCake();
    private boolean intakeYPressed = false;
    private boolean intakeAPressed = false;
    private boolean flipBPressed = false;
    private boolean bumpersPressed = false;
    private boolean intakeInToggledOn = false;
    private boolean intakeOutToggledOn = false;
    private Mecanum mecanum = null;
    private boolean TopFlipSet = false;
    private boolean liftXPressed = false;
    private boolean liftYPressed = false;
    private boolean liftAPressed = false;
    private boolean liftBPressed = false;
    private boolean liftMoving = false;
    private boolean limitHit = false;
    private int liftLocation = 0;
    private boolean intakeArmActive = false;
    private int timeCount = 0;
    private MineralDeposit currentMDState = MineralDeposit.WAITING_TO_START;

    private enum MineralDeposit {
        WAITING_TO_START,
        START,
        FRONT_FLIPPING,
        FRONT_FLIPPING_AGAIN,
        FRONT_FLIPPING_BACK,
        WAITING_FOR_LIFT_AND_FLIP
    }

    @Override
    public void init(){
        robot.init(hardwareMap);


        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true, telemetry);
        mecanum.SetCurrentPower(1.4);
    }

    @Override
    public void init_loop() {
        if (gamepad2.dpad_up){
            robot.getFrontFlip().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        telemetry.addData("Front Flip", "%d", robot.getFrontFlip().getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        if (TopFlipSet == false){
            robot.getTopFlip().setPosition(1);

            TopFlipSet = true;
        }

        mecanum.Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Back Lift", "%d", robot.getBackLift().getCurrentPosition());
        telemetry.addData("Front Lift", "%d", robot.getFrontLift().getCurrentPosition());
        telemetry.addData("Front Flip", "%d", robot.getFrontFlip().getCurrentPosition());
        telemetry.addData("Lf, rf, lb, rb: ", "%d %d %d %d", robot.getFrontL().getCurrentPosition(),
                robot.getFrontR().getCurrentPosition(), robot.getBackL().getCurrentPosition(),
                robot.getBackR().getCurrentPosition());
        telemetry.addData("Limit Switch:", robot.getLimitSwitch().getState());
        telemetry.addData("Auto Intake Active:", intakeArmActive);

        if (!intakeArmActive) {
            ProcessSlide();
            ProcessIntake();
            ProcessFrontFlip();
        }

        if (!intakeArmActive && !liftMoving) {
            ProcessLift();
        }

        ProcessTopFlip();
        ProcessPower();

        ProcessLiftLocations();
        ProcessLiftButtons();

        telemetry.update();
    }

    private void ProcessSlide() {
        if ((gamepad1.left_trigger > 0)) {
            robot.getSlide().setPower(1);
        } else if ((gamepad1.right_trigger > 0)) {
            robot.getSlide().setPower(-1);
        } else {
            robot.getSlide().setPower(0);
        }
    }

    private void ProcessIntake() {
        if (gamepad1.y && !intakeYPressed) {
            intakeYPressed = true;
            intakeAPressed = false;
        } else if (!gamepad1.y && intakeYPressed) {
            if (!intakeInToggledOn) {
                robot.getIntake().setPower(-1);  // TODO - negative this if it spins the wrong way
                intakeInToggledOn = true;
                intakeOutToggledOn = false;
            } else {
                robot.getIntake().setPower(0);
                intakeInToggledOn = false;
                intakeOutToggledOn = false;
            }

            intakeYPressed = false;
            intakeAPressed = false;
        } else if (gamepad1.a && !intakeAPressed) {
            intakeAPressed = true;
            intakeYPressed = false;
        } else if (!gamepad1.a && intakeAPressed) {
            if (!intakeOutToggledOn) {
                robot.getIntake().setPower(1); // TODO - negative this is if spinis the wrong way
                intakeInToggledOn = false;
                intakeOutToggledOn = true;
            } else {
                robot.getIntake().setPower(0);
                intakeInToggledOn = false;
                intakeOutToggledOn = false;
            }

            intakeAPressed = false;
            intakeYPressed = false;
        }
    }

    private void ProcessPower(){
        if (gamepad1.dpad_down){
            mecanum.SetCurrentPower(0.6);
        } else if (gamepad1.dpad_up){
            mecanum.SetCurrentPower(1.0);
        } else if(gamepad1.dpad_left){
            mecanum.SetCurrentPower(1.4);
        }
    }

    private void ProcessFrontFlip() {
        if (gamepad1.left_bumper || gamepad2.left_bumper) {
            robot.getFrontFlip().setPower(.75);
        } else if (gamepad1.right_bumper || gamepad2.right_bumper) {
            robot.getFrontFlip().setPower(-1);
        } else {
            robot.getFrontFlip().setPower(0);
        }
    }

    private void ProcessLift() {
        robot.getBackLift().setPower(-gamepad2.left_stick_y); // TODO - change these by either flipping the negative or adding/removing a negative
        robot.getFrontLift().setPower(gamepad2.left_stick_y);
    }

    private void ProcessTopFlip() {
        if (gamepad2.left_trigger != 0) {
            robot.getTopFlip().setPosition(.35);
        } else if (gamepad2.right_trigger != 0) {
            robot.getTopFlip().setPosition(0);
        } else {
            robot.getTopFlip().setPosition(1);
        }
    }

    private void ProcessLiftLocations() {
        if (liftMoving == true) {
            if ((gamepad2.left_stick_y != 0) || (gamepad2.left_trigger != 0) || (gamepad2.right_trigger !=0)){
                robot.getBackLift().setPower(0);
                robot.getFrontLift().setPower(0);
                liftAPressed = liftBPressed = liftXPressed = liftYPressed = liftMoving = false;
            } else {
                int pos = robot.getFrontLift().getCurrentPosition();
                telemetry.addData("Difference", (liftLocation - pos));
                if ((liftLocation - pos) > 0) {
                    if (Math.abs(liftLocation - pos) < 5) {
                        // close enough so stop
                        robot.getBackLift().setPower(0);
                        robot.getFrontLift().setPower(0);
                        liftAPressed = liftBPressed = liftXPressed = liftYPressed = liftMoving = false;
                    } else if (Math.abs(liftLocation - pos) < 50) {
                        robot.getBackLift().setPower(.5);
                        robot.getFrontLift().setPower(-.5);
                    } else {
                        robot.getBackLift().setPower(1);
                        robot.getFrontLift().setPower(-1);
                    }
                } else if ((liftLocation - pos) < 0) {
                    if (Math.abs(liftLocation - pos) < 5) {
                        // close enough so stop
                        robot.getBackLift().setPower(0);
                        robot.getFrontLift().setPower(0);
                        liftAPressed = liftBPressed = liftXPressed = liftYPressed = liftMoving = false;
                    } else if (Math.abs(liftLocation - pos) < 50) {
                        robot.getBackLift().setPower(-.5);
                        robot.getFrontLift().setPower(.5);
                    } else {
                        // move down
                        robot.getBackLift().setPower(-1);
                        robot.getFrontLift().setPower(1);
                    }
                }
            }
        }

        if (gamepad2.a && !liftAPressed) {
            liftLocation = 0;
            liftMoving = liftAPressed = true;
            liftBPressed = liftXPressed = liftYPressed = false;
        } else if (gamepad2.x && !liftXPressed) {
            liftLocation = 1038;
            liftMoving = liftXPressed = true;
            liftAPressed = liftYPressed = liftBPressed = false;
        } else if (gamepad2.y && !liftYPressed) {
            liftLocation = 515;
            liftMoving = liftYPressed = true;
            liftAPressed = liftXPressed = liftBPressed = false;
        } else if (gamepad2.b && !liftBPressed) {
            liftLocation = 1300;
            liftMoving = liftBPressed = true;
            liftAPressed = liftXPressed = liftYPressed = false;
        }
    }

    private void ProcessLiftButtons () {
        boolean limit = robot.getLimitSwitch().getState();
        int pos = robot.getBackLift().getCurrentPosition();

        if (gamepad1.b && !intakeArmActive){
            currentMDState = MineralDeposit.START;
            intakeArmActive = true;
        }

        if (intakeArmActive) {
            if (!limit) {
                limitHit = true;
            }

            if ((gamepad2.left_stick_y != 0) || (gamepad2.left_bumper) ||
                    (gamepad2.right_bumper) || (gamepad1.left_bumper) ||
                    (gamepad1.right_bumper) || (gamepad2.left_trigger != 0) ||
                    (gamepad2.right_trigger !=0) || (gamepad1.left_trigger != 0) ||
                    (gamepad1.right_trigger !=0)){
                robot.getBackLift().setPower(0);
                robot.getBackLift().setPower(0);
                robot.getFrontFlip().setPower(0);
                robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.getSlide().setPower(0);
                intakeArmActive = false;
                limitHit = false;
                currentMDState = MineralDeposit.WAITING_TO_START;
            }

            switch (currentMDState) {
                case START:
                    robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.getFrontFlip().setTargetPosition(1400);
                    robot.getFrontFlip().setPower(0.7);
                    if (pos > 10) {
                        robot.getBackLift().setPower(-0.8);
                        robot.getFrontLift().setPower(0.8);
                    }

                    currentMDState = MineralDeposit.FRONT_FLIPPING;
                    break;

                case FRONT_FLIPPING:
                    if (robot.getFrontFlip().getCurrentPosition() < 1900) {
                        robot.getSlide().setPower(1);
                        currentMDState = MineralDeposit.WAITING_FOR_LIFT_AND_FLIP;
                    }
                    break;

                case WAITING_FOR_LIFT_AND_FLIP:
                    if ((pos <= 10) && (robot.getFrontFlip().getCurrentPosition() < 1410) && limitHit) {
                        currentMDState = MineralDeposit.FRONT_FLIPPING_AGAIN;
                        robot.getFrontFlip().setTargetPosition(50);
                        timeCount = 0;
                    }
                    break;

                case FRONT_FLIPPING_AGAIN:
                    if ((pos <= 10) &&(robot.getFrontFlip().getCurrentPosition() < 50)) {
                        timeCount++;
                    }

                    if (timeCount > 5) {
                        robot.getFrontFlip().setTargetPosition(750);
                        currentMDState = MineralDeposit.FRONT_FLIPPING_BACK;
                    }
                    break;

                case FRONT_FLIPPING_BACK:
                    if (robot.getFrontFlip().getCurrentPosition() > 750) {
                        robot.getFrontFlip().setPower(0);
                        robot.getFrontFlip().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        currentMDState = MineralDeposit.WAITING_TO_START;
                        intakeArmActive = false;
                        limitHit = false;
                    }
                    break;
            }

            if (limitHit) {
                robot.getSlide().setPower(0);
            }

            if (pos <= 10) {
                robot.getBackLift().setPower(0);
                robot.getFrontLift().setPower(0);
            }
        }
    }
}