package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.CENTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.ElliotDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name = "RedClosetoWallMeet3Auton")
public class RedCloseToWallMeet3Auton extends LinearOpMode {

    public ElliotDrive drive;
    SiddyDetector vision;
    private OpenCvWebcam webcam;

    //enum SiddyPosition;

//    SiddyDetector SiddyPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {

        ElliotDrive drive = new ElliotDrive(hardwareMap);
        vision = new SiddyDetector(hardwareMap,telemetry,0);
        
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

        Pose2d starting = new Pose2d(15, -63, Math.toRadians(-90));
        drive.setPoseEstimate(starting);

        TrajectorySequence toMiddleSequence = drive.trajectorySequenceBuilder(new Pose2d(starting.vec(),m(-90)))
                .addDisplacementMarker(0,()->{
                    drive.klance.setPosition(0.5);
                })
                .lineTo(new Vector2d(15,-32))
                .lineTo(new Vector2d(15,-40))
                .turn(m(-90))
                .build();

        TrajectorySequence toMiddleBackBoard = drive.trajectorySequenceBuilder(toMiddleSequence.end())
                .lineTo(new Vector2d(47,-50))
//                .addDisplacementMarker(50,()->{
//                    drive.klance.setPosition(0);
//                })

                .build();


        TrajectorySequence toLeftSequence = drive.trajectorySequenceBuilder(new Pose2d(starting.vec(),m(-90)))
                .addDisplacementMarker(0,()->{
                    drive.klance.setPosition(0.5);
                })
                .strafeTo(new Vector2d(2,-30.5))
                .lineTo(new Vector2d(2,-45))
                .turn(m(-90))
                .build();

        TrajectorySequence toLeftBackboard = drive.trajectorySequenceBuilder(toLeftSequence.end())
                .lineTo(new Vector2d(20,-45))
                .lineTo(new Vector2d(20, -50))
//                .addDisplacementMarker(50,()->{
//                    drive.klance.setPosition(0);
//                })

                .build();

        TrajectorySequence toRightSequence = drive.trajectorySequenceBuilder(new Pose2d(starting.vec(),m(-90)))
                .addDisplacementMarker(0,()->{
                    drive.klance.setPosition(0.5);
        })
                .strafeTo(new Vector2d(25,-39))
                .lineTo(new Vector2d(25,-49))
                .turn(m(-90))
                .build();

        TrajectorySequence toRightBackboard = drive.trajectorySequenceBuilder(toLeftSequence.end())
                .lineTo(new Vector2d(61,-49))
                .lineTo(new Vector2d(61.5, -71))
//                .addDisplacementMarker(50,()->{
//                    drive.klance.setPosition(0);
//                })

                .build();



                //50,47/50,50
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("siddy position: ", vision.location);
            telemetry.update();

            sleep(20);

        }

        waitForStart();
            if(isStopRequested()) return;

            drive.klance.setPosition(0.5);
            //drive.ramp.setPosition(0.025);

           if (vision.location == LEFT){


               drive.followTrajectorySequence(toLeftSequence);
               drive.followTrajectorySequence(toLeftBackboard);

              // drive.followTrajectory(toLeftButICry);

           } else if (vision.location == CENTER){

               drive.followTrajectorySequence(toMiddleSequence);
               drive.followTrajectorySequence(toMiddleBackBoard);
            //   drive.klance.setPosition(0);

           } else if (vision.location == RIGHT){
               drive.followTrajectorySequence(toRightSequence);
               drive.followTrajectorySequence(toRightBackboard);


           }

//            drive.followTrajectory(toMiddleSpike);
          //  drive.followTrajectory(backToStartMiddle);
//        drive.followTrajectorySequence(toMiddleSequence);

    }

    public static double m (double degrees){
        return Math.toRadians(degrees);
    }

}
