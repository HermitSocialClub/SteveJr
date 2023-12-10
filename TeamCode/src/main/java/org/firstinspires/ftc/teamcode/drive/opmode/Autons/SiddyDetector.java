package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import static org.opencv.core.Core.countNonZero;
import static org.opencv.core.Core.vconcat;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Mat;

import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

public class SiddyDetector extends OpenCvPipeline {
    public enum SiddyPosition {
        LEFT,
        CENTER,
        RIGHT,
        NONE
    }

    int red_or_blue;
    public SiddyPosition location;
    public SiddyDetector(HardwareMap hardwareMap, Telemetry telemetry, int color)
    {
        //red is 0,blue is 1
        red_or_blue = color;
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
        Scalar low_color;
        Scalar high_color;
        if (red_or_blue == 0){
            low_color = new Scalar(0, 70, 50);
            high_color = new Scalar(10, 255, 255);
        }
        else
        {
            low_color = new Scalar(94, 80, 2);
            high_color = new Scalar(120, 255, 255);
        }

        Mat mask = new Mat();

        Core.inRange(mat, low_color, high_color, mask);

        int height = mask.height();
        int width = mask.width();

        Rect half_region = new Rect(0, height/3, width, height/3);
        Mat mask_half = new Mat(mask, half_region);

        Mat display_mask_half = new Mat();
        Imgproc.cvtColor(mask_half, display_mask_half, Imgproc.COLOR_GRAY2RGB);

        Mat real_half = new Mat(input, half_region);
        Mat return_mat = new Mat();

        List<Mat> image_to_show = Arrays.asList(real_half, mask_half);

//        vconcat(image_to_show, return_mat);

        Rect region_1 = new Rect(0, height/3, width/3, height/3);
        Rect region_2 = new Rect(width/3, height/3, width/3, height/3);
        Rect region_3 = new Rect(2 * width/3, height/3, width/3, height/3);
//
        Mat region_1_crop = new Mat(mask, region_1);
        Mat region_2_crop = new Mat(mask, region_2);
        Mat region_3_crop = new Mat(mask, region_3);

        int[] pixels = {countNonZero(region_1_crop), countNonZero(region_2_crop), countNonZero(region_3_crop)};

        int maxAt = 0;

        for (int i = 0; i < pixels.length; i++) {
            maxAt = pixels[i] > pixels[maxAt] ? i : maxAt;
        }

        location = SiddyPosition.values()[maxAt];

        return mask_half;
    }
}
