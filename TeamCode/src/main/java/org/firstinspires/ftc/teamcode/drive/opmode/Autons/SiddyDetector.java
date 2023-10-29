package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;

public class SiddyDetector extends OpenCvPipeline {
    public enum SiddyPosition {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }
    public SiddyPosition location;
    public SiddyDetector()
    {
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        if (mat.empty())
        {
            location = SiddyPosition.NONE;
            return input;
        }

        Scalar lowBlueHSV = new Scalar(94, 80, 2);
        Scalar highBlueHSV = new Scalar(120, 255, 255);

        Mat mask = new Mat();

        Core.inRange(mat, lowBlueHSV, highBlueHSV, mask);

        int height = mask.height();
        int width = mask.width();
        Rect region_1 = new Rect(0, (int)(height/2), (int)(width/3), (int)(height/2));
        Rect region_2 = new Rect();
        Rect region_3 = new Rect();

        Mat region_1_crop = new Mat(mask, region_1);
        Mat region_2_crop = new Mat(mask, region_2);
        Mat region_3_crop = new Mat(mask, region_3);


        return region_1_crop;
    }
}
