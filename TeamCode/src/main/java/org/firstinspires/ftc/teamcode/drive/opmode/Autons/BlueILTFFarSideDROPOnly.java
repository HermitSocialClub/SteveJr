package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.CENTER;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.LEFT;
import static org.firstinspires.ftc.teamcode.drive.opmode.Autons.SiddyDetector.SiddyPosition.RIGHT;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

@Autonomous (name = "ILTBlueFar")
public class BlueILTFFarSideDROPOnly extends LinearOpMode {
    public NamjoonDrive drive;
    SiddyDetector vision;
    private OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException
    {
        NamjoonDrive drive = new NamjoonDrive(hardwareMap);

        //init vision
        vision = new SiddyDetector(hardwareMap,telemetry,0);

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

        TrajectoryVelocityConstraint velocityConstraint = NamjoonDrive.getVelocityConstraint(15, m(150), NamjoonDriveConstants.TRACK_WIDTH);

        TrajectorySequence dropPurpleMid = drive.trajectorySequenceBuilder(startLine)
                .setVelConstraint(velocityConstraint)
                .forward(37)
                .back(12)
                .build();

        TrajectorySequence dropPurpleRight = drive.trajectorySequenceBuilder(startLine)
                .setVelConstraint(velocityConstraint)
                .forward(15)
                .turn(m(-90))
                .forward(12.5)
                .turn(m(90))
                .forward(25)
                .back(20)
                .build();

        TrajectorySequence dropPurpleLeft = drive.trajectorySequenceBuilder(startLine)
                .setVelConstraint(velocityConstraint)
                .forward(25)
                .turn(m(90))
                .forward(15)
                .back(13)
                .turn(m(-30))
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
            drive.followTrajectorySequenceAsync(dropPurpleLeft);

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
            }
        }
    }
    public static double m (double degrees){
        return Math.toRadians(degrees);
    }

}

