
package org.firstinspires.ftc.teamcode.test;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.*;
import com.disnodeteam.dogecv.filters.HSVColorFilter;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Scalar;

import java.io.IOException;


@Autonomous(name="DogeCV Jewel Detector", group="DogeCV")

public class JewelOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private GenericDetector genericDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        genericDetector = new GenericDetector();
        //genericDetector.colorFilter = new HSVColorFilter(new Scalar(0,150,0), new Scalar(76,255,179));
        genericDetector.colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.YELLOW);
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        genericDetector.areaWeight = 0.02;
        genericDetector.detectionMode = GenericDetector.GenericDetectionMode.MAX_AREA; // PERFECT_AREA
        //genericDeterctor.perfectArea = 6500; <- Needed for PERFECT_AREA
        genericDetector.debugContours = true;
        genericDetector.maxDiffrence = 15;
        genericDetector.ratioWeight = 15;
        genericDetector.minArea = 100;

        genericDetector.enable();


    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized.");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {



        telemetry.addData("Status", "Run Time: " + runtime.toString());

        if (genericDetector.getFound()) {

            telemetry.addData("Location: ", genericDetector.getLocation());
            telemetry.addData("Rectangle: ", genericDetector.getRect());

        }
        else {
            telemetry.addData("Object Not Found", "");
        }



    }

    @Override
    public void stop() {
        genericDetector.disable();
    }

}
