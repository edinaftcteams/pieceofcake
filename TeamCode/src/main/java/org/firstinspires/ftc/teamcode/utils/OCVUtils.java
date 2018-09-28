
package org.firstinspires.ftc.teamcode.utils;

import android.content.ContextWrapper;
import android.content.res.AssetManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;
import android.widget.ImageView;

import com.vuforia.Image;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Size;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;

import static org.opencv.imgproc.Imgproc.getRotationMatrix2D;
import static org.opencv.imgproc.Imgproc.warpAffine;

/**
 * Created by FIXIT on 16-07-04.
 */
public final class OCVUtils {

    final static String LOGTAG = "FTC OpenCV";
    static ImageView displayImage;

    public static Mat bitmapToMat (Bitmap bit, int cvType) {
        Mat newMat = new Mat(bit.getHeight(), bit.getWidth(), cvType);

        Utils.bitmapToMat(bit, newMat);

        return newMat;
    }

    public static Bitmap matToBitmap (Mat mat) {
        Bitmap newBit = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);

        Utils.matToBitmap(mat, newBit);

        return newBit;
    }


    public static void hsvNullifyValue(Mat process) {

        int size = (int) (process.total() * process.channels());

        byte[] temp = new byte[size];

        process.get(0, 0, temp);
        for (int i = 2; i < size; i += 3) {
            temp[i] = -1;
        }//for

        process.put(0, 0, temp);

    }


    public static Bitmap getVuforiaImage(VuforiaLocalizer.CloseableFrame frame, int format){
        Image img;
        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                img =  frame.getImage(i);
                Bitmap bm = Bitmap.createBitmap(img.getWidth(), img.getHeight(), Bitmap.Config.RGB_565);
                bm.copyPixelsFromBuffer(img.getPixels());
                return bm;
            }//if
        }//for
        return null;
    }


    public static Mat rotate(Mat src, double angle) {
        Mat dst = new Mat();
        Point pt = new Point(src.cols()/2, src.rows()/2);
        Mat r = getRotationMatrix2D(pt, angle, 1.0);
        warpAffine(src, dst, r, new Size(src.cols(), src.rows()));
        return dst;
    }

}