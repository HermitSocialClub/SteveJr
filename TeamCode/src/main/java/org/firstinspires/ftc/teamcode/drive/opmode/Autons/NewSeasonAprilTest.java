package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous (name = "NewSeasonAprilTest")
public class NewSeasonAprilTest extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    double tagsize = 0.04;  // was .166

    // insert our tags of interest here - three or two w default if not seen
    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int BLUELEFT = 1;
    int BLUEMIDDLE = 2;
    int BLUERIGHT = 3;
    int REDLEFT = 4;
    int REDMIDDLE = 5;
    int REDRIGHT = 6;

    AprilTagDetection tagOfInterest = null;
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
