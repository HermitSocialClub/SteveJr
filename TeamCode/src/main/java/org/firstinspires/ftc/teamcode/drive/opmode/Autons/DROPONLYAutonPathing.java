package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.CENTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.ElliotDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name = "DROPONLYBLUEAutonPathing")
public class DROPONLYAutonPathing extends LinearOpMode {

    public ElliotDrive drive;
    SiddyDetector vision;
    private OpenCvWebcam webcam;

    //enum SiddyPosition;

//    SiddyDetector SiddyPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {

        ElliotDrive drive = new ElliotDrive(hardwareMap);
        vision = new SiddyDetector(hardwareMap,telemetry,1);
        
        // Vision
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(vision);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera", "alive");

            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Camera", "dead");
            }
        });

        Pose2d starting = new Pose2d(15, -60, Math.toRadians(-90));
        drive.setPoseEstimate(starting);

        TrajectorySequence toMiddleSequence = drive.trajectorySequenceBuilder(new Pose2d(starting.vec(),m(-90)))
                .back(30.5)
                .forward(27)
                .turn(m(90))
                //.back(35)
                .build();

        TrajectorySequence toRightSequence = drive.trajectorySequenceBuilder(new Pose2d(starting.vec(),m(-90)))
                .back(19)
                .waitSeconds(0.5)
                .turn(m(-47))
                .waitSeconds(0.5)
                .back(10)
                .forward(7.5)
//                .waitSeconds(0.5)
//                .turn(m(47))
//                .waitSeconds(0.5)
//                .forward(15)
//                .turn(m(90))
//                .back(35)
                .build();

        TrajectorySequence toLeftSequence = drive.trajectorySequenceBuilder(new Pose2d(starting.vec(),m(-90)))
                .back(20.5)
                .turn(m(47))
                .waitSeconds(0.5)
                .back(7)
                .forward(4.5)
//                .waitSeconds(0.5)
//                .turn(m(-47))
//                .waitSeconds(0.5)
//                .forward(17)
//                .turn(m(90))
//                .back(35)
                .build();

        Trajectory toMiddleSpike = drive.trajectoryBuilder(new Pose2d(starting.vec(),m(-90)),m(90))
                .back(30.5)
//                .forward(5)
                //.splineTo(new Vector2d(35,-35.5),m(-90))
               // .splineTo(new Vector2d(35,-41),m(90))
                .build();

        Trajectory backToStartMiddle = drive.trajectoryBuilder(toMiddleSpike.end())
                .splineToSplineHeading(new Pose2d(35,-60,m(0)),m(90))
                .build();

        Trajectory toLeftSpike = drive.trajectoryBuilder(new Pose2d(starting.vec(),m(-90)))
                .splineToSplineHeading(new Pose2d(17,-37,m(-130)),m(90))
                //.lineToSplineHeading(new Pose2d(40,-45,m(130)))
                .build();

        Trajectory toLeftButICry = drive.trajectoryBuilder(new Pose2d(starting.vec(),Math.toRadians(-90)),Math.toRadians(90))
               // .splineTo(new Vector2d(40,-45),m(130))
                .splineToSplineHeading(new Pose2d(35,-30.5,m(-50)),m(90))
                .build();

        Trajectory backToStartLeft = drive.trajectoryBuilder(toLeftSpike.end())
                .splineToSplineHeading(new Pose2d(35,-60,m(0)),m(90))
                .build();

        Trajectory toRightSpike = drive.trajectoryBuilder(new Pose2d(starting.vec(),m(-90)),m(90))
                .splineToSplineHeading(new Pose2d(30,-40,m(50)),m(90))
                .build();

        Trajectory backToStartRight = drive.trajectoryBuilder(toRightSpike.end())
                .splineToSplineHeading(new Pose2d(35,-60,m(0)),m(90))
                .build();

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("siddy position: ", vision.location);
            telemetry.update();

            sleep(20);

        }

        waitForStart();
            if(isStopRequested()) return;

            drive.ramp.setPosition(0.025);

           if (vision.location == LEFT){

               drive.followTrajectorySequence(toLeftSequence);
              // drive.followTrajectory(toLeftSpike);
              // drive.followTrajectory(toLeftButICry);

           } else if (vision.location == CENTER){

               drive.followTrajectorySequence(toMiddleSequence);

           } else if (vision.location == RIGHT){

               drive.followTrajectorySequence(toRightSequence);

           } else {

               drive.followTrajectorySequence(toLeftSequence);

           }

//            drive.followTrajectory(toMiddleSpike);
          //  drive.followTrajectory(backToStartMiddle);
//        drive.followTrajectorySequence(toMiddleSequence);

    }

    public static double m (double degrees){
        return Math.toRadians(degrees);
    }

}
