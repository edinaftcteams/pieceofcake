package org.firstinspires.ftc.teamcode.autonomous;

import com.edinaftcrobotics.drivetrain.Mecanum;
import com.edinaftcrobotics.navigation.TurnOMatic;
import com.edinaftcrobotics.navigation.TurnOMatic2;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.enums.AutonomousStates;
import org.firstinspires.ftc.teamcode.enums.MineralLocation;
import org.firstinspires.ftc.teamcode.robot.PieceOfCake;

import java.util.List;

abstract class BaseAutoOpMode extends LinearOpMode {
    protected int DrivePerInch = (int)(1120 / 18.85);
    private int FlatFlip = 1800;
    private int SlideOffLatchDistance = 275;
    private int slideCenterPosition = 250;
    private int Turn90 = 1225;
    private int Turn45 = Turn90/2;
    private ElapsedTime watch = new ElapsedTime();
    protected double PushMineralDistance = 6;
    protected double BackAwayFromMineralDistance = 5.5;
    protected int slideRightPosition = DrivePerInch * 23;
    protected int slideLeftPosition = DrivePerInch * 23;
    protected int driveForwardPosition = (int)(DrivePerInch * 19);
    protected int driveForwardCraterPosition = (int)(DrivePerInch * 14);

    private static final String VUFORIA_KEY = "ASA9XvT/////AAABmUnq30r9sU3Nmf/+RS+Xx0CHgJj/JtD5ycahnuM/0B2SFvbMRPIZCbLi4LeOkfse9Dymor5W7vNMYI+vmqVx9kpEaKE8VM7cFMUb/T1LLwlCPdX9QKOruzTcRdlYswR7ULh4K11GuFZDO/45pSks+Nf25kT5cnV+IN3TsscA0o7I6XPIeUoAJJPsjw+AycsmRk2uffr3Bnupexr93iRfHylniqP+ss4cRcT1lOqS5Zhh7FQaoelR58qL/RUorGpknjy9ufCn9ervc6Mz01u3ZkM/EOa5wUPT8bDzPZ6nMDaadqumorT5Py+GtJSUosUgz4Gd3iR++fdEk6faFZq3L9xfBSagNykwhiyYx+oqwVqe";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    protected MineralLocation mineralLocation = MineralLocation.RIGHT;
    private VuforiaLocalizer vuforia;

    protected PieceOfCake robot = new PieceOfCake();
    protected Mecanum mecanum = null;

    protected BNO055IMU imu = null;
    protected TFObjectDetector tfod;
    protected Recognition LastRecognition = null;

    //
    // This is our init section.  We have all the code here that we will use to init and setup the robot for
    // autonomous.  The main parts are:
    //
    //  initRobot - Setup the drive and video
    //  initVuforia - Used to setup the camera for mineral detection
    //  initTFod - Used to setup the TensorFlow for mineral detection
    //  initGyro - Used to setup the gyro for use when we want to turn
    //
    protected void InitRobot() {
        robot.init(hardwareMap);

        mecanum = new Mecanum(robot.getFrontL(), robot.getFrontR(), robot.getBackL(), robot.getBackR(), true, telemetry);

        initVuforia();

        robot.StopAndResetAllEncoders();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }
    }

    //
    // Init the gyro for turns and general heading information.  We get the imu and set it up for
    // degrees.  We then wait for it to get ready
    //
    protected void InitGyro() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu.initialize(parameters);
        while (!imu.isGyroCalibrated()) {
            telemetry.addData("I am trying to init the gyro", "so stop moving the lander or robot Jack!");
            telemetry.update();
        }
    }

    protected double GetImuAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }

    //
    // Init the tensor flow for mineral detection.  We load the model and set the mineral location to the one
    // on the right.  You can read up on tensorflow here https://en.wikipedia.org/wiki/TensorFlow
    //
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
        mineralLocation = MineralLocation.RIGHT;
    }

    //
    // Init the camera so we can use to to find the right mineral to knock off.
    //
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    //
    // Our mineral logic is different than what was provided by FIRST.  Our camera cannot see all
    // three minerals, so we had to change the logic to look at all the things it found.
    // After some debugging, we determined the right top and bottom range of the minerals from the
    // view of the camera We then figured out the left positions and used that with the top,
    // bottom, and label to find the mineral location.  If we didn't see a gold mineral at all,
    // we then picked the right one
    //
    public void LocateTFMineral() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if ((updatedRecognitions != null) && (updatedRecognitions.size() > 0)) {
                mineralLocation = MineralLocation.RIGHT;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("Object", recognition);
                    //
                    // We are looking for a gold mineral that is between 520 and 730 units from the
                    // phone.  This number can be changed based on actual field testing at the
                    // competition
                    //
                    if ((recognition.getLabel().equals(LABEL_GOLD_MINERAL)) &&
                            (recognition.getTop() > 520) && (recognition.getBottom() < 730)) {
                        //
                        // Now we check the left position of the mineral to see if is the left or
                        // middle one.  This number can be changed based on actual field testing
                        // at the competition
                        //
                        int goldMineralX = (int) recognition.getLeft();
                        if (goldMineralX < 430) {
                            mineralLocation = MineralLocation.LEFT;
                        } else if ((goldMineralX >= 430)  && (goldMineralX <= 650)) {
                            mineralLocation = MineralLocation.MIDDLE;
                        } else {
                            mineralLocation = MineralLocation.RIGHT;
                        }

                        LastRecognition = recognition;

                        break;
                    }
                }
            } else {
                telemetry.addData("Nothing New", "Detected");
            }
        } else {
            telemetry.addData("No", "TFOD");
        }
    }

    public void ShutdownTFOD() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    //
    // The following section is our many different states for our state machine which is used
    // during autonomous.  Each state returns a finished state which the autonomous state machine
    // uses to figure out what to do next.  Each state can be used multiple times in the same
    // machine if we want to.  That is why we went with a state machine.  It was easy to
    // build, modify, and test.  To learn more about state machines, visit
    // https://en.wikipedia.org/wiki/Finite-state_machine
    //
    // Our states are:
    //  Latch - Power the lift so it will hang on the lander
    //  Drop - drops the robot from the lander
    //  MoveLeftOffLatch - we slide left to get detached from the hook and then move back to
    //      straighten out
    //  MoveForwardAndSlideBackToCenter - we move forwards and center ourselves in front of the
    //      lander
    //  DropMarker - we extend our arm out to drop the marker
    //  TurnLeftTowardsCrater2 - we turn left 135 degrees towards the crater using RUN_WITH_ENCODER
    //      vs using RUN_TO_POSITION
    //  PickUpAndDepositMineral - we pick up the gold mineral and put it in thw lander
    //  BringLiftDownAndExtendArm - we bring the lift down and extend the arm on depot side
    //  DropFrontFlip - we drop our front flip into the crater
    //  TurnIntakeOn - we turn our intake on after our arm is dropped in the crater
    //  BringLiftDownAndExtendArmForCrater - we bring our lift down and extend arm on the crater side
    //
    public AutonomousStates Latch () {
        double currentPower = .12;
        boolean aPressed = false;
        boolean bPressed = false;

        // loop while they adjust the power to get the robot to hang properly
        while (!gamepad2.x) {
            robot.getBackLift().setPower(-currentPower);
            robot.getFrontLift().setPower(currentPower);

            if (gamepad2.a) {
                aPressed = true;
            }

            if (!gamepad2.a && aPressed) {
                aPressed = false;
                currentPower -= .01;
                if (currentPower < .1) {
                    // don't let them go below 10%
                    currentPower = .1;
                }
            }

            if (gamepad2.b) {
                bPressed = true;
            }

            if (!gamepad2.b && bPressed) {
                bPressed = false;
                currentPower += .01;
                if (currentPower > .2) {
                    // don't let them go above 20%
                    currentPower = .2;
                }
            }

            // display the status on the screen
            telemetry.addData("Current Power", currentPower);
            telemetry.addData("Alex, press Gamepad2 A", " to decrease power");
            telemetry.addData("Alex, press Gamepad2 B", " to increase power");
            telemetry.addData("Alex, Press X to Exit", " and power will be locked in.");
            telemetry.update();
        }

        return AutonomousStates.LATCHED;
    }

    public AutonomousStates Drop() {
        // lock the slide
        robot.getSlide().setPower(.1);
        // flip the arm down so it doesn't get hit
        robot.getTopFlip().setPosition(1);
        robot.getFrontFlip().setTargetPosition(FlatFlip);
        robot.getFrontFlip().setPower(.7);

        while (robot.getFrontFlip().isBusy() && opModeIsActive()) {
            idle();
        }

        // land the robot by turning off the motors that made us latch
        robot.getBackLift().setPower(0);
        robot.getFrontLift().setPower(0);

        watch.reset();
        while ((watch.milliseconds() < 1000)  && opModeIsActive()){
            idle();
        }

        // unhook from the lander
        robot.getBackLift().setPower(.3);
        robot.getFrontLift().setPower(-.3);
        watch.reset();
        while ((watch.milliseconds() < 400)  && opModeIsActive()){
            idle();
        }

        robot.getBackLift().setPower(0);
        robot.getFrontLift().setPower(0);

        return AutonomousStates.DROPPED;
    }

    public AutonomousStates MoveLeftOffLatch() {
        // slide a little to the left so we can be outside the hook
        robot.getTopFlip().setPosition(1);

        mecanum.SlideLeft2(.5, SlideOffLatchDistance, this);

        watch.reset();
        // move backwards to line up against the lander
        mecanum.Move(-.3, -.3);
        while ((watch.milliseconds() < 200)  && opModeIsActive()) {
            idle();
        }

        mecanum.Stop();

        // turn if we are not straight
        if ((GetImuAngle()) < 0) {
            mecanum.TurnRight(0.3, 20, this);
        }

        return AutonomousStates.MOVED_OFF_LATCH;
    }

    public AutonomousStates MoveForwardAndSlideBackToCenter(int forwardDistance) {
        // lock the slide so it doesn't move around
        robot.getSlide().setPower(.5);

        // Move forward and then move right towards the center
        mecanum.MoveForward2(.6, forwardDistance, this);
        mecanum.SlideRight2(.7, SlideOffLatchDistance, this);

        robot.getSlide().setPower(0);

        return AutonomousStates.MOVED_BACK_TO_CENTER;
    }

    public AutonomousStates DropMarker () {
        watch.reset();
        // slide arm out
        robot.getSlide().setPower(-1);
        // run for 1000 milliseconds
        while ((watch.milliseconds() < 1000)  && opModeIsActive()) {
            idle();
        }

        robot.getSlide().setPower(0);

        // spin the intake to dump marker
        watch.reset();
        robot.getIntake().setPower(-1);
        while ((watch.milliseconds() < 750)  && opModeIsActive()) {
            idle();
        }

        robot.getIntake().setPower(0);

        // move slide back in
        watch.reset();
        robot.getSlide().setPower(1);
        // run for 1000 milliseconds
        while ((watch.milliseconds() < 1000)  && opModeIsActive()) {
            idle();
        }

        robot.getSlide().setPower(0);

        return AutonomousStates.DROPPED_MARKER;
    }

    public AutonomousStates TurnLeftTowardsCrater2() {
        // turn us left towards teh crater and get us close to the wall
        // we will turn 135 degrees
        mecanum.TurnLeft(.5, Turn45 + Turn90, this);

        mecanum.SlideRight2(.5, DrivePerInch * 15, this);

        mecanum.MoveForward2(.5,DrivePerInch * 10, this);

        return AutonomousStates.TURNED_TOWARDS_CRATER;
    }

    public AutonomousStates PickUpAndDepositMineral(boolean craterSide) {
        int counter = 0;
        int leftDepotTurnAmount = (int)(Turn45 / 1.2);
        int rightDepotTurnAmount = (int)(Turn45 / 1.35);
        int leftCrateTurnAmount = (int)(Turn45 / 1.2);
        int rightCraterturnAmount = (int)(Turn45 / 1.2);
        int pos = robot.getFrontLift().getCurrentPosition();

        robot.getTopFlip().setPosition(1);

        if (craterSide == false) {
            mecanum.MoveBackwards2(0.8, DrivePerInch * 5, this);
        }

        if (pos > 10) {
            robot.getBackLift().setPower(-0.8);
            robot.getFrontLift().setPower(0.8);
        }

        if (craterSide) {
            if (mineralLocation == MineralLocation.LEFT) {
                mecanum.TurnLeft(0.5, leftCrateTurnAmount, this);
            } else if (mineralLocation == MineralLocation.RIGHT) {
                mecanum.TurnRight(0.5, rightCraterturnAmount, this);
            }
        } else {
            if (mineralLocation == MineralLocation.LEFT) {
                mecanum.TurnLeft(0.5, leftDepotTurnAmount, this);
            } else if (mineralLocation == MineralLocation.RIGHT) {
                mecanum.TurnRight(0.5, rightDepotTurnAmount, this);
            }
        }

        while (counter != -1 && opModeIsActive()) {
            pos = robot.getFrontLift().getCurrentPosition();

            if (pos < 10) {
                robot.getBackLift().setPower(0);
                robot.getFrontLift().setPower(0);
            }

            if (counter == 0) {
                robot.getFrontFlip().setTargetPosition(2800);
                robot.getFrontFlip().setPower(1);
                if (robot.getFrontFlip().getCurrentPosition() > 2550) {
                    robot.getFrontFlip().setPower(0);
                    robot.getIntake().setPower(1);
                    counter = 1;
                    watch.reset();
                }
            } else if (counter == 1) {
                mecanum.Move(0.1, 0.1);
                robot.getSlide().setPower(-.7);
                if ((mineralLocation == MineralLocation.MIDDLE) && (watch.milliseconds() > 1200)) {
                    mecanum.Move(0,0);
                    robot.getSlide().setPower(0);
                    counter = 2;
                    watch.reset();
                } else if (watch.milliseconds() > 1600) {
                    robot.getSlide().setPower(0);
                    mecanum.TurnBrakeOff();
                    counter = 2;
                    watch.reset();
                }
            } else if (counter == 2) {
                if (watch.milliseconds() > 500) {
                    robot.getFrontFlip().setTargetPosition(800);
                    robot.getFrontFlip().setPower(1);
                    counter = 3;
                    watch.reset();
                }

            } else if (counter == 3) {
                robot.getSlide().setPower(1);
                if ((mineralLocation == MineralLocation.MIDDLE) && (watch.milliseconds() > 500)) {
                    robot.getSlide().setPower(0);
                    counter = 4;
                    watch.reset();
                } else if (watch.milliseconds() > 750) {
                    robot.getSlide().setPower(0);
                    counter = 4;
                    watch.reset();
                }

            } else if (counter == 4) {
                if (pos < 10) {
                    robot.getFrontFlip().setTargetPosition(0);     // TOO MUCH STATE MACHINES!!!
                    robot.getFrontFlip().setPower(1);           // STATE MACHINE INSIDE A STATE MACHINE !!!  MADNESS!!
                    watch.reset();
                    counter = 5;
                }
            } else if (counter == 5) {
                if (watch.milliseconds() > 750) {
                    robot.getFrontFlip().setTargetPosition(800);
                    robot.getFrontFlip().setPower(1);
                    robot.getIntake().setPower(0);
                    counter = 6;
                }
            } else if (counter == 6) {
                if (robot.getFrontFlip().getCurrentPosition() >790) {
                    robot.getFrontFlip().setPower(0);
                    counter = 7;
                }
            }
            else if (counter == 7) {
                counter = -1;
            }
        }

        if (craterSide) {
            if (mineralLocation == MineralLocation.LEFT) {
                mecanum.TurnRight(0.5, leftCrateTurnAmount, this);
            } else if (mineralLocation == MineralLocation.RIGHT) {
                mecanum.TurnLeft(0.5, rightCraterturnAmount, this);
            }
        } else {
            if (mineralLocation == MineralLocation.LEFT) {
                mecanum.TurnRight(0.5, leftDepotTurnAmount, this);
            } else if (mineralLocation == MineralLocation.RIGHT) {
                mecanum.TurnLeft(0.5, rightDepotTurnAmount, this);
            }
        }
        pos = robot.getFrontLift().getCurrentPosition();

        robot.getBackLift().setPower(1);
        robot.getFrontLift().setPower(-1);

        int liftHeight = 1038;

        if (craterSide) {
            liftHeight = 1290;
        }

        while (pos < liftHeight && opModeIsActive()) {
            pos = robot.getFrontLift().getCurrentPosition();
            idle();
        }

        robot.getBackLift().setPower(0);
        robot.getFrontLift().setPower(0);

        if (craterSide)
        {
            mecanum.SlideLeft2(0.5,DrivePerInch * 9, this );
        }

        mecanum.MoveBackwards2(0.8, DrivePerInch * 9, this);

        robot.getTopFlip().setPosition(0);
        watch.reset();

        while (watch.milliseconds() < 1000 && opModeIsActive()) {
            idle();
        }

        robot.getTopFlip().setPosition(1);

        if (craterSide) {
        mecanum.MoveForward2(0.8, DrivePerInch * 17, this);
        } else {
        mecanum.MoveForward2(0.8, DrivePerInch * 19, this);
        }

        return AutonomousStates.BACKED_AWAY_FROM_MINERAL;
    }

    public AutonomousStates BringLiftDownAndExtendArm() {
        // stick the arm out for things like crater parking
        int pos = robot.getFrontLift().getCurrentPosition();

        robot.getFrontFlip().setTargetPosition(FlatFlip);
        robot.getFrontFlip().setPower(.7);

        telemetry.addData("Lift Position", robot.getFrontLift().getCurrentPosition());
        telemetry.update();

        // slide arm out
        robot.getSlide().setPower(-1);
        robot.getBackLift().setPower(-1);
        robot.getFrontLift().setPower(1);
        watch.reset();

        while ((pos > 10 || (watch.milliseconds() < 1500)) && opModeIsActive()) {

            pos = robot.getFrontLift().getCurrentPosition();

            if (pos <= 10) {
                robot.getBackLift().setPower(0);
                robot.getFrontLift().setPower(0);
            }

            if (watch.milliseconds() > 750) {
                robot.getSlide().setPower(0);
            }

            idle();
        }

        robot.getBackLift().setPower(0);
        robot.getFrontLift().setPower(0);
        robot.getSlide().setPower(0);

        return AutonomousStates.LIFT_DOWN;
    }

    public AutonomousStates DropFrontFlip() {

        robot.getFrontFlip().setPower(0.7);
        robot.getFrontFlip().setTargetPosition(2600);

        while (robot.getFrontFlip().isBusy() && opModeIsActive()) {
            idle();
        }

        robot.getFrontFlip().setPower(0);

        return AutonomousStates.FLIP_DOWN;
    }

    public AutonomousStates TurnIntakeOn() {
        robot.getIntake().setPower(1);
        robot.getSlide().setPower(-1);
        watch.reset();

        while (watch.milliseconds() < 3000 && opModeIsActive()) {
            idle();
        }

        if (watch.milliseconds() > 750) {
            robot.getSlide().setPower(0);
        }

        return AutonomousStates.INTAKE_ON;
    }

    public AutonomousStates BringLiftDownAndExtendArmForCrater()  {
        int pos = robot.getFrontLift().getCurrentPosition();

        robot.getFrontFlip().setTargetPosition(FlatFlip);
        robot.getFrontFlip().setPower(.7);

        telemetry.addData("Lift Position", robot.getFrontLift().getCurrentPosition());
        telemetry.update();

        // slide arm out
        robot.getSlide().setPower(-1);
        robot.getBackLift().setPower(-1);
        robot.getFrontLift().setPower(1);
        watch.reset();

        while ((pos > 10 || (watch.milliseconds() < 500)) && opModeIsActive()) {

            pos = robot.getFrontLift().getCurrentPosition();

            if (pos <= 10) {
                robot.getBackLift().setPower(0);
                robot.getFrontLift().setPower(0);
            }

            if (watch.milliseconds() > 500) {
                robot.getSlide().setPower(0);
            }

            idle();
        }

        robot.getBackLift().setPower(0);
        robot.getFrontLift().setPower(0);
        robot.getSlide().setPower(0);

        DropFrontFlip();
        robot.getIntake().setPower(1);

        watch.reset();

        robot.getSlide().setPower(-0.6);
        while ((watch.milliseconds() < 1500) && opModeIsActive()) {
            idle();
        }

        robot.getSlide().setPower(0);


        return AutonomousStates.INTAKE_ON;
    }
}
