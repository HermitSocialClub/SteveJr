package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.CENTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.RIGHT;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;
import org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name = "ILTMain")
public class ILTMain extends LinearOpMode {

    public NamjoonDrive drive;
    SiddyDetector vision;
    private OpenCvWebcam webcam;
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double feedforward = 0.05;
 //   public static int target = -150;

    public static double ticks_per_deg = 752/180;

    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

    PIDFController controller = new PIDFController(coeffs);


    //enum SiddyPosition;

//    SiddyDetector SiddyPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {

        NamjoonDrive drive = new NamjoonDrive(hardwareMap);
        drive.chains.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.chains.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        Pose2d startLine = new Pose2d(-40, -62, Math.toRadians(90));
        drive.setPoseEstimate(startLine);

        Trajectory dropPurpleMid = drive.trajectoryBuilder(new Pose2d(startLine.vec(),Math.toRadians(90)),Math.toRadians(90))
                .splineTo(
                        new Vector2d(-40,-33),m(90),
                        NamjoonDrive.getVelocityConstraint(15, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(30,()->{
//                    drive.leftRear.setPower(0);
//                    drive.leftFront.setPower(0);
//                    drive.rightFront.setPower(0);
//                    drive.rightRear.setPower(0);
                   // drive.clawFlipper.setPosition(0.9);
                    drive.clawLeft.setPosition(0);
                    drive.clawRight.setPosition(0);
                    //drive.clawFlipper.setPosition(0.4);
                })
                .splineToConstantHeading(
                        new Vector2d(-40,-45),m(90),
                        NamjoonDrive.getVelocityConstraint(10, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(32,()->{
                    drive.clawFlipper.setPosition(0.4);
                })
                .splineToSplineHeading(
                        new Pose2d(-45,-39,m(180)),m(180),
                        NamjoonDrive.getVelocityConstraint(10, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory pickUpWhiteMid = drive.trajectoryBuilder(dropPurpleMid.end(), m(180))
                .splineTo(
                        new Vector2d(-61,-37),m(180),
                        NamjoonDrive.getVelocityConstraint(15, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(10,()->{
                   //drive.clawFlipper.setPosition();
                    drive.clawLeft.setPosition(0.7);
                    drive.clawRight.setPosition(0.7);
                })
                .build();

        Trajectory dropYellowWhiteMid = drive.trajectoryBuilder(pickUpWhiteMid.end(),m(0))
                .splineToSplineHeading(new Pose2d(-50,-10,m(180)),m(0))
//                .splineTo(new Vector2d(0,-10),m(0))
//                .splineTo(new Vector2d(50, -35),m(0))
                .build();


        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("siddy position: ", vision.location);
            telemetry.update();

            sleep(20);

        }

        waitForStart();
        if(isStopRequested()) return;
        while (opModeIsActive()) {
            // flippy(-300);
            controller.setTargetPosition(-150);
            int armPos = drive.chains.getCurrentPosition();
            double correction = controller.update(armPos, feedforward);

            drive.chains.setPower(correction);
        }

            drive.clawFlipper.setPosition(0.87);
            drive.clawLeft.setPosition(0.7);
            drive.clawRight.setPosition(0.7);
           // drive.spool.setPower(-0.05);

           if (vision.location == LEFT){



           } else if (vision.location == CENTER){

               drive.followTrajectory(dropPurpleMid);
              // drive.clawFlipper.setPosition(0.87);
//               sleep(1000);
               while (opModeIsActive()) {
                   // flippy(-300);
                   controller.setTargetPosition(-400);
                   int armPos = drive.chains.getCurrentPosition();
                   double correction = controller.update(armPos, feedforward);

                   drive.chains.setPower(correction);
               }
               drive.followTrajectory(pickUpWhiteMid);
               drive.followTrajectory(dropYellowWhiteMid);
//               sleep(1000);
//               drive.followTrajectory(dropYellowWhiteMid);
//               drive.clawFlipper.setPosition(0.5);
//               drive.claw.setPosition(0);

           } else if (vision.location == RIGHT){



           } else {



           }

//            drive.followTrajectory(toMiddleSpike);
          //  drive.followTrajectory(backToStartMiddle);
//        drive.followTrajectorySequence(toMiddleSequence);

    }

    public static double m (double degrees){
        return Math.toRadians(degrees);
    }
//    public void flippy (double target) {
//        chains.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        chains.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//         double kP = 0.1;
//         double kI = 0;
//         double kD = 0;
//         double feedforward = 0.05;
//        //public static int target = ;
//
//        double ticks_per_deg = 752/180;
//
//        PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
//
//        PIDFController controller = new PIDFController(coeffs);
//        controller.setTargetPosition(target);
//        int armPos = chains.getCurrentPosition();
//        double correction = controller.update(armPos,feedforward);
//
//        chains.setPower(correction);
//    }

}
