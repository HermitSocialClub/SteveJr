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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;
import org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Vector;

@Autonomous (name = "ILTEST")
public class ILTEST extends LinearOpMode {

    public NamjoonDrive drive;
    SiddyDetector vision;
    private OpenCvWebcam webcam;
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double feedforward = 0.05;
    public int actionIndex = 0;
    //   public static int target = -150;

    public static double ticks_per_deg = 752 / 180;

    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

    PIDFController controller = new PIDFController(coeffs);


    //enum SiddyPosition;

//    SiddyDetector SiddyPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {

        NamjoonDrive drive = new NamjoonDrive(hardwareMap);

        //initialize chains

        // Vision
        vision = new SiddyDetector(hardwareMap, telemetry, 1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setPipeline(vision);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera", "alive");

            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera", "dead");
            }
        });


        //pathing
        Pose2d startLine = new Pose2d(15, 62, Math.toRadians(-90));
        drive.setPoseEstimate(startLine);

        Trajectory dropPurpleMid = drive.trajectoryBuilder(new Pose2d(startLine.vec(), Math.toRadians(90)), Math.toRadians(90))
                .splineTo(
                        new Vector2d(15, 35), m(-90),
                        NamjoonDrive.getVelocityConstraint(25, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )

                .splineToConstantHeading(
                        new Vector2d(18, 35), m(0),
                        NamjoonDrive.getVelocityConstraint(25, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )

                .splineToConstantHeading(
                        new Vector2d(53, 35), m(0),
                        NamjoonDrive.getVelocityConstraint(25, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )

                .splineToConstantHeading(
                        new Vector2d(53, 39), m(90),
                        NamjoonDrive.getVelocityConstraint(25, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )

                .splineToConstantHeading(
                        new Vector2d(53, 52), m(-90),
                        NamjoonDrive.getVelocityConstraint(25, NamjoonDriveConstants.MAX_ANG_VEL, NamjoonDriveConstants.TRACK_WIDTH),
                        NamjoonDrive.getAccelerationConstraint(NamjoonDriveConstants.MAX_ACCEL)
                )
                .build();


        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("siddy position: ", vision.location);
            telemetry.update();

            sleep(20);

        }

        waitForStart();
        if (isStopRequested()) return;

        if (vision.location == CENTER) {

            drive.followTrajectory(dropPurpleMid);

//            while (opModeIsActive() && !isStopRequested()) {
//                telemetry.addData("drive busy: ", drive.isBusy());
//                telemetry.addData("action: ", actionIndex);
//                switch (actionIndex) {
//                    case 0:
//                        if (drive.isBusy()) break;
//                        actionIndex++;
//                        controller.setTargetPosition(-200);
//                        drive.followTrajectoryAsync(dropPurpleMid);
//                        break;
//
//                    case 1:
//                        if (drive.isBusy()) break;
//                        actionIndex++;
//                        Trajectory pickUpWhiteMid = null;
//                        drive.followTrajectoryAsync(pickUpWhiteMid);
//                        break;
//
//                    case 2:
//                        if (drive.isBusy()) break;
//                        actionIndex++;
//                        Trajectory dropYellowWhiteMid = null;
//                        drive.followTrajectoryAsync(dropYellowWhiteMid);
//                        break;
//
//                    default:
//                        break;
//                }
//                int armPos = drive.chains.getCurrentPosition();
//                double correction = controller.update(armPos);
//
//                drive.chains.setPower(correction);
//                drive.update();
//            }

        } else if (vision.location == RIGHT) {


        } else if (vision.location == LEFT) {


        } else {

        }

    }
    public static double m ( double degrees){
        return Math.toRadians(degrees);
    }
}
