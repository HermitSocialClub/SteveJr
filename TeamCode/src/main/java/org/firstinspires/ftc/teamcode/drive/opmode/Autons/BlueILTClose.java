package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.CENTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;
import org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name = "ILTBlueClose")
public class BlueILTClose extends LinearOpMode {
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

        TrajectoryVelocityConstraint velocityConstraint = NamjoonDrive.getVelocityConstraint(35, m(150), NamjoonDriveConstants.TRACK_WIDTH);

        TrajectorySequence dropPurpleMid = drive.trajectorySequenceBuilder(startLine)
                .setVelConstraint(velocityConstraint)
                .forward(35)
                .back(11)
                .build();

        TrajectorySequence dropPurpleLeft = drive.trajectorySequenceBuilder(startLine)
                .setVelConstraint(velocityConstraint)
                .forward(15)
                .strafeLeft(4)
                .forward(25)
                .back(20)
                .build();

        TrajectorySequence dropPurpleRight = drive.trajectorySequenceBuilder(startLine)
                .setVelConstraint(velocityConstraint)
                .forward(25)
                .turn(m(-90))
                .forward(15)
                .back(13)
//                .ba9-ck(20)
                .turn(m(30))
                .build();

        TrajectorySequence goToBoardRightIntermediate = drive.trajectorySequenceBuilder(dropPurpleRight.end())
                .setVelConstraint(velocityConstraint)
                .turn(m(60))
                .forward(4)
                .turn(m(-90))
                .build();

        TrajectorySequence goToBoardLeft = drive.trajectorySequenceBuilder(dropPurpleLeft.end())
                .setVelConstraint(velocityConstraint)
                //.back(3)
                .turn(m(-90))
               // .strafeLeft(10)
                .back(35)
                .strafeLeft(3)
                .build();

        TrajectorySequence goToBoardCenter = drive.trajectorySequenceBuilder(dropPurpleMid.end())
                .setVelConstraint(NamjoonDrive.getVelocityConstraint(30, m(150), NamjoonDriveConstants.TRACK_WIDTH))
                .turn(m(-90))
                .back(34)
                .strafeLeft(6)
                .back(7)
//                .turn(m(-45))
//                .back(8)
//                .turn(m(45))
//                .back(24)
                .build();

        TrajectorySequence goParkCenter = drive.trajectorySequenceBuilder(goToBoardCenter.end())
                .setVelConstraint(velocityConstraint)
                .forward(4)
                .turn(m(-90))
                .forward(26)
                .build();


        TrajectorySequence goToBoardRight = drive.trajectorySequenceBuilder(goToBoardRightIntermediate.end())
                .setVelConstraint(velocityConstraint)
                .back(37)
                .build();

        TrajectorySequence goParkRight = drive.trajectorySequenceBuilder(goToBoardRight.end())
                .setVelConstraint(velocityConstraint)
                .turn(m(-90))
                .forward(26)
                .build();

        TrajectorySequence goParkLeft = drive.trajectorySequenceBuilder(goToBoardLeft.end())
                .setVelConstraint(velocityConstraint)
                .forward(4)
                .turn(m(-90))
                .forward(26)
                .build();

        // running vision
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("siddy position: ", vision.location);
            telemetry.update();

            sleep(20);

        }

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
            drive.followTrajectorySequenceAsync(dropPurpleMid);

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
                        drive.ARM_CONTROLLER.setTargetPosition(-1000);
                        if(Math.abs(drive.chains.getCurrentPosition() + 1000) > 30) break;

                        //we do these seperately to make sure we arent moving the robot with too much inertia
                       // drive.LINEAR_CONTROLLER.setTargetPosition(200);
//                        if(Math.abs(drive.spoolEncoder.getCurrentPosition() - 700) > 30) break;


                        drive.followTrajectorySequenceAsync(goToBoardCenter);
                        //the reason we do this is because we only want to call followTrajectorySequence once, so instead of redoing the loop we can move onto the next
                        //one and wait for drive to be unbusy
                        actionIndex++;
                        break;
                    case 3:
                        if (drive.isBusy()) break;
                        drive.setFlipperPosition(0.35);
                        sleep(1000);
                        drive.openLeftClaw();
                        sleep(500);
                        drive.closeLeftClaw();
                        sleep(500);

                        actionIndex++;
                        break;
                    case 4:
                        //no drive trajectories should be running so no drive.isBusy() here needed
                        drive.ARM_CONTROLLER.setTargetPosition(-500);
                        if(Math.abs(drive.chains.getCurrentPosition() + 600) > 30) break;

                      //  drive.LINEAR_CONTROLLER.setTargetPosition(0);
//                        if(Math.abs(drive.spoolEncoder.getCurrentPosition() - 50) > 30) break;
                        actionIndex++;
                        break;
                    case 5:
                        //we do this seperate from the last one because since its a loop it tries to set arm pos to -700 and then -100 right away every
                        // time the loop breaks
                        //sleep(1000); <- the reason we cant use this is because we are running PID stuff right now. So when we turn this thread off
                        // with sleep the PID shits itself
                        //drive.ARM_CONTROLLER.setTargetPosition(-300);
                       // if(Math.abs(drive.chains.getCurrentPosition() + 300) > 30) break;
                        drive.flipperUp();

//                        drive.ARM_CONTROLLER.setTargetPosition(-300);
//                        if(Math.abs(drive.chains.getCurrentPosition() + 300) > 30) break;

                        drive.ARM_CONTROLLER.setTargetPosition(-100);
                        if(Math.abs(drive.chains.getCurrentPosition() + 300) > 30) break;

                        drive.followTrajectorySequenceAsync(goParkCenter);
                        actionIndex++;
                        break;
                    default:
                        break;
                }
                drive.updateAllPIDs();
                drive.update();

            }
        }
        else if (vision.location == LEFT)
        {
            int actionIndex = 1;
            drive.followTrajectorySequenceAsync(dropPurpleLeft);

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

                    case 2:
                        //this is the same as the previous if statement but cleaner
                        if (drive.isBusy()) break;
                        //wait for the arm to get in position before moving, bc we have no pid correction ann we cannot lift up while moving
                        // our wheels do not have enough torque
                        drive.ARM_CONTROLLER.setTargetPosition(-1100);
                        if(Math.abs(drive.chains.getCurrentPosition() + 1100) > 30) break;

                        //we do these seperately to make sure we arent moving the robot with too much inertia
                        drive.LINEAR_CONTROLLER.setTargetPosition(100);
                        if(Math.abs(drive.spoolEncoder.getCurrentPosition() - 100) > 30) break;


                        drive.followTrajectorySequenceAsync(goToBoardLeft);
                        //the reason we do this is because we only want to call followTrajectorySequence once, so instead of redoing the loop we can move onto the next
                        //one and wait for drive to be unbusy
                        actionIndex++;
                        break;
                    case 3:
                        if (drive.isBusy()) break;
                        drive.setFlipperPosition(0.4);
                        sleep(1000);
                        drive.openLeftClaw();
                        sleep(500);
                        drive.closeLeftClaw();
                        sleep(500);

                        actionIndex++;
                        break;
                    case 4:
                        //no drive trajectories should be running so no drive.isBusy() here needed
                        drive.ARM_CONTROLLER.setTargetPosition(-600);
                        if(Math.abs(drive.chains.getCurrentPosition() + 600) > 30) break;

                        break;
                    case 5:
                        //we do this seperate from the last one because since its a loop it tries to set arm pos to -700 and then -100 right away every
                        // time the loop breaks
                        //sleep(1000); <- the reason we cant use this is because we are running PID stuff right now. So when we turn this thread off
                        // with sleep the PID shits itself
                        drive.ARM_CONTROLLER.setTargetPosition(-300);
                        if(Math.abs(drive.chains.getCurrentPosition() + 300) > 30) break;
                        break;
                }
                drive.updateAllPIDs();
                drive.update();

            }

        }

        else if (vision.location == RIGHT)
        {
            int actionIndex = 1;
            drive.followTrajectorySequenceAsync(dropPurpleRight);

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
                        if (drive.isBusy()) break;
                        drive.followTrajectorySequenceAsync(goToBoardRightIntermediate);
                        actionIndex++;
                        break;
                    case 3:
                        //this is the same as the previous if statement but cleaner
                        if (drive.isBusy()) break;
                        //wait for the arm to get in position before moving, bc we have no pid correction ann we cannot lift up while moving
                        // our wheels do not have enough torque
                        drive.ARM_CONTROLLER.setTargetPosition(-1100);
                        if(Math.abs(drive.chains.getCurrentPosition() + 1100) > 30) break;

                        //we do these seperately to make sure we arent moving the robot with too much inertia
                        drive.LINEAR_CONTROLLER.setTargetPosition(700);
                        if(Math.abs(drive.spoolEncoder.getCurrentPosition() - 700) > 30) break;


                        drive.followTrajectorySequenceAsync(goToBoardRight);
                        //the reason we do this is because we only want to call followTrajectorySequence once, so instead of redoing the loop we can move onto the next
                        //one and wait for drive to be unbusy
                        actionIndex++;
                        break;
                    case 4:
                        if (drive.isBusy()) break;
                        drive.setFlipperPosition(0.15);
                        sleep(1000);
                        drive.openLeftClaw();
                        sleep(500);
                        drive.closeLeftClaw();
                        sleep(500);

                        actionIndex++;
                        break;
                    case 5:
                        //no drive trajectories should be running so no drive.isBusy() here needed
                        drive.ARM_CONTROLLER.setTargetPosition(-600);
                        if(Math.abs(drive.chains.getCurrentPosition() + 600) > 30) break;

                        drive.LINEAR_CONTROLLER.setTargetPosition(50);
                        if(Math.abs(drive.spoolEncoder.getCurrentPosition() - 50) > 30) break;
                        actionIndex++;
                        break;
                    case 6:
                        //we do this seperate from the last one because since its a loop it tries to set arm pos to -700 and then -100 right away every
                        // time the loop breaks
                        //sleep(1000); <- the reason we cant use this is because we are running PID stuff right now. So when we turn this thread off
                        // with sleep the PID shits itself
                        drive.ARM_CONTROLLER.setTargetPosition(-300);
                        if(Math.abs(drive.chains.getCurrentPosition() + 300) > 30) break;
                        break;
                    default:
                        break;

                }
                drive.updateAllPIDs();
                drive.update();
            }
        }
    }
    public static double m (double degrees){
        return Math.toRadians(degrees);
    }

}

