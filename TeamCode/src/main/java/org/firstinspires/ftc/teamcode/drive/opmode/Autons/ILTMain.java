package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.CENTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.RIGHT;

import android.annotation.SuppressLint;

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
    public int actionIndex = 0;
 //   public static int target = -150;

    public static double ticks_per_deg = 752/180;

    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

    PIDFController controller = new PIDFController(coeffs);


    //enum SiddyPosition;

//    SiddyDetector SiddyPosition = null;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {

        NamjoonDrive drive = new NamjoonDrive(hardwareMap);

        //initialize chains

        // Vision
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


        //pathing

        Pose2d startLine = new Pose2d(-40, -62, Math.toRadians(90));
        drive.setPoseEstimate(startLine);

        Trajectory dropPurpleMid = drive.trajectoryBuilder(new Pose2d(startLine.vec(),Math.toRadians(90)),Math.toRadians(90))
                .splineTo(
                        new Vector2d(-40,-33),m(90),
                        NamjoonDrive.getVelocityConstraint(15, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory dropPurpleMidPartTwo = drive.trajectoryBuilder(dropPurpleMid.end())
                .splineToConstantHeading(
                        new Vector2d(-40,-45),m(90),
                        NamjoonDrive.getVelocityConstraint(10, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )
                .splineToSplineHeading(
                        new Pose2d(-45,-39,m(180)),m(180),
                        NamjoonDrive.getVelocityConstraint(10, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )
                .build();

        Trajectory pickUpWhiteMid = drive.trajectoryBuilder(dropPurpleMidPartTwo.end(), m(180))
                .splineTo(
                        new Vector2d(-61,-37),m(180),
                        NamjoonDrive.getVelocityConstraint(15, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )
//                .addDisplacementMarker(15,()->{
//                    drive.clawFlipper.setPosition(0.5);
//                    drive.clawLeft.setPosition(0.15);
//                    drive.clawRight.setPosition(0.8);
//                })
                .build();

        Trajectory dropYellowWhiteMid = drive.trajectoryBuilder(pickUpWhiteMid.end(),m(0))
                .splineToSplineHeading(
                        new Pose2d(-56,0,m(0)),m(180),
                        NamjoonDrive.getVelocityConstraint(15, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)

                )
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
//        while (opModeIsActive()) {
//            // flippy(-300);
//            controller.setTargetPosition(-100);
//            int armPos = drive.chains.getCurrentPosition();
//            double correction = controller.update(armPos, feedforward);
//
//            drive.chains.setPower(correction);
//        }

           // drive.clawFlipper.setPosition(0.7);
           // drive.clawLeft.setPosition(0.15);
            //drive.clawRight.setPosition(0.8);
           // drive.spool.setPower(-0.05);
        waitForStart();
        if(isStopRequested()) return;

        //init stuff
        drive.closeLeftClaw();
        drive.closeRightClaw();
        drive.flipperUp();
        drive.ARM_CONTROLLER.setTargetPosition(-100);

        if (vision.location == CENTER)
        {
            int actionIndex = 1;
            drive.followTrajectoryAsync(dropPurpleMid);

            while (opModeIsActive() && !isStopRequested()) {
                switch (actionIndex) {
                    case 1:
                        //even though we start at one we have to wait for
                        //once previous trajectory is complete
                        if (!drive.isBusy()) {
                            //update the ticker to go to the next action, where we may or may not wait for this trajectory to complete
                            drive.flipperDown();
                            sleep(1000);
                            drive.openRightClaw();
                            //claws are mentally challenged and need a second to go up/down. Since we have no PID updates at this time this is fine
                            // THIS WILL NOT WORK IN ALL SITUATIONS IT ONLY WORKS BECAUSE THERES NO PID UPDATES SO WE CAN STOP PID FOR A SECOND
                            sleep(1000);
                            drive.flipperUp();
                            drive.closeRightClaw();
                            sleep(1000);
                            actionIndex++;
                        }
                        break;
                    case 2:
                        //this is the same as the previous if statement but cleaner
                        if (drive.isBusy()) break;
                        //wait for the arm to get in position before moving, bc we have no pid correction ann we cannot lift up while moving
                        // our wheels do not have enough torque
//                        drive.ARM_CONTROLLER.setTargetPosition(-1100);
//                        if(Math.abs(drive.chains.getCurrentPosition() + 1100) > 30) break;
//
//                        //we do these seperately to make sure we arent moving the robot with too much inertia
//                        drive.LINEAR_CONTROLLER.setTargetPosition(700);
//                        if(Math.abs(drive.spoolEncoder.getCurrentPosition() - 700) > 30) break;
//
//
                        drive.followTrajectoryAsync(dropPurpleMidPartTwo);
                        //the reason we do this is because we only want to call followTrajectorySequence once, so instead of redoing the loop we can move onto the next
                        //one and wait for drive to be unbusy
                        actionIndex++;
                        break;
                    case 3:
                        //this is the same as the previous if statement but cleaner
                        if (drive.isBusy()) break;
                        //wait for the arm to get in position before moving, bc we have no pid correction ann we cannot lift up while moving
                        // our wheels do not have enough torque
                        drive.ARM_CONTROLLER.setTargetPosition(-300);
                        if(Math.abs(drive.chains.getCurrentPosition() + 300) > 30) break;


                        drive.followTrajectoryAsync(pickUpWhiteMid);

                        if (!drive.isBusy()) {
                            drive.openRightClaw();
                            drive.flipperDown();
                            drive.ARM_CONTROLLER.setTargetPosition(-200);
                            if(Math.abs(drive.chains.getCurrentPosition() + 200) > 30) break;
                        }
                        //the reason we do this is because we only want to call followTrajectorySequence once, so instead of redoing the loop we can move onto the next
                        //one and wait for drive to be unbusy
                        actionIndex++;
                        break;
                }
                drive.updateAllPIDs();
                drive.update();

            }
        }
        else if (vision.location == RIGHT)
        {
            int actionIndex = 1;
          //  drive.followTrajectorySequenceAsync(dropPurpleRight);

            while (opModeIsActive() && !isStopRequested()) {
                switch (actionIndex)
                {
                    case 1:
                        //even though we start at one we have to wait for
                        //once previous trajectory is complete
                        if (drive.isBusy()) break;
                        //update the ticker to go to the next action, where we may or may not wait for this trajectory to complete
                        drive.flipperDown();
                        sleep(1000);
                        drive.openRightClaw();
                        //claws are mentally challenged and need a second to go up/down. Since we have no PID updates at this time this is fine
                        // THIS WILL NOT WORK IN ALL SITUATIONS IT ONLY WORKS BECAUSE THERES NO PID UPDATES SO WE CAN STOP PID FOR A SECOND
                        sleep(1000);
                        drive.flipperUp();
                        drive.closeRightClaw();
                        sleep(1000);
                        actionIndex++;

                        break;
                }
                drive.updateAllPIDs();
                drive.update();

            }

        }

        else if (vision.location == LEFT)
        {
            int actionIndex = 1;
           // drive.followTrajectorySequenceAsync(dropPurpleLeft);

            while (opModeIsActive() && !isStopRequested()) {
                switch (actionIndex) {
                    case 1:
                        //even though we start at one we have to wait for
                        //once previous trajectory is complete
                        if (!drive.isBusy()) {
                            //update the ticker to go to the next action, where we may or may not wait for this trajectory to complete
                            drive.flipperDown();
                            sleep(1000);
                            drive.openRightClaw();
                            //claws are mentally challenged and need a second to go up/down. Since we have no PID updates at this time this is fine
                            // THIS WILL NOT WORK IN ALL SITUATIONS IT ONLY WORKS BECAUSE THERES NO PID UPDATES SO WE CAN STOP PID FOR A SECOND
                            sleep(1000);
                            drive.flipperUp();
                            drive.closeRightClaw();
                            sleep(1000);
                            actionIndex++;
                        }
                        break;
                }
                drive.updateAllPIDs();
                drive.update();

//           if (vision.location == LEFT){
//
//
//
//           } else if (vision.location == CENTER){
//                //fsm
//               //TODO: make a finite state machine class and clean this up
//               while(opModeIsActive() && !isStopRequested())
//               {
//                   telemetry.addData("drive busy: ", drive.isBusy());
//                   telemetry.addData("action: ", actionIndex);
//                   switch (actionIndex) {
//                       case 0:
//                           if (drive.isBusy()) break;
//                           actionIndex++;
//                           controller.setTargetPosition(-200);
//                           drive.followTrajectoryAsync(dropPurpleMid);
//                           break;
//
//                       case 1:
//                           if (drive.isBusy()) break;
//                           actionIndex++;
//                           drive.followTrajectoryAsync(pickUpWhiteMid);
//                           break;
//
//                       case 2:
//                           if (drive.isBusy()) break;
//                           actionIndex++;
//                           drive.followTrajectoryAsync(dropYellowWhiteMid);
//                           break;
//
//                       default:
//                           break;
//                   }
//                   int armPos = drive.chains.getCurrentPosition();
//                   double correction = controller.update(armPos);
//
//                   drive.chains.setPower(correction);
//                   drive.update();
//               }
//               // drive.clawFlipper.setPosition(0.87);
////               sleep(1000);
//               // flippy(-300);
//                   drive.followTrajectory(dropPurpleMid);
////                   drive.followTrajectory(pickUpWhiteMid);
////                   drive.followTrajectory(dropYellowWhiteMid);
//
////               sleep(1000);
////               drive.followTrajectory(dropYellowWhiteMid);
////               drive.clawFlipper.setPosition(0.5);
////               drive.claw.setPosition(0);
//
//           } else if (vision.location == RIGHT){
//
//
//
//           } else {
//
//

           }

//            drive.followTrajectory(toMiddleSpike);
          //  drive.followTrajectory(backToStartMiddle);
//        drive.followTrajectorySequence(toMiddleSequence);

    }
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
