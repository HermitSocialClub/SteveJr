package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.CENTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.RIGHT;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;
import org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name = "ILTTestAuto")
public class TrialAuto extends LinearOpMode {
    public NamjoonDrive drive;
    SiddyDetector vision;
    private OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException
    {
        NamjoonDrive drive = new NamjoonDrive(hardwareMap);

        //init vision
        vision = new SiddyDetector(hardwareMap,telemetry,1);

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


        //init pathing
        Pose2d startLine = new Pose2d(-40, -62, Math.toRadians(90));
        drive.setPoseEstimate(startLine);

        TrajectoryVelocityConstraint velocityConstraint = NamjoonDrive.getVelocityConstraint(15, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH);

        TrajectorySequence dropPurpleMid = drive.trajectorySequenceBuilder(startLine)
                .setVelConstraint(velocityConstraint)
                .addDisplacementMarker(() -> {
                    drive.closeLeftClaw();
                    drive.closeRightClaw();
                    drive.ARM_CONTROLLER.setTargetPosition(-100);
                })
                .forward(48)
                .back(12)
                .addDisplacementMarker(() -> {
                    drive.ARM_CONTROLLER.setTargetPosition(0);
                    drive.openRightClaw();
                })
                .waitSeconds(5)
                .build();


//        Trajectory dropPurpleMid = drive.trajectoryBuilder(new Pose2d(startLine.vec(),Math.toRadians(90)),Math.toRadians(90))
//                .splineTo(
//                        new Vector2d(-40,-33),m(90),
//                        NamjoonDrive.getVelocityConstraint(15, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
//                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
//                )
//                .addDisplacementMarker(30,()->{
////                    drive.leftRear.setPower(0);
////                    drive.leftFront.setPower(0);
////                    drive.rightFront.setPower(0);
////                    drive.rightRear.setPower(0);
//                    drive.clawFlipper.setPosition(0.5);
//                    drive.clawLeft.setPosition(0.15);
//                    drive.clawRight.setPosition(0.15);
//                    //drive.clawFlipper.setPosition(0.4);
//                })
//                .splineToConstantHeading(
//                        new Vector2d(-40,-45),m(90),
//                        NamjoonDrive.getVelocityConstraint(10, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
//                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
//                )
//                .addDisplacementMarker(32,()->{
////                    drive.ARM_CONTROLLER.setTargetPosition(0);
//                    drive.clawFlipper.setPosition(0.4);
//                })
//                .splineToSplineHeading(
//                        new Pose2d(-45,-39,m(180)),m(180),
//                        NamjoonDrive.getVelocityConstraint(10, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
//                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
//                )
//                .build();


        // running vision
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("siddy position: ", vision.location);
            telemetry.update();

            sleep(20);

        }

        waitForStart();
        if(isStopRequested()) return;

        //running trajectory sequence
        drive.followTrajectorySequence(dropPurpleMid);
    }
    public static double m (double degrees){
        return Math.toRadians(degrees);
    }

}

