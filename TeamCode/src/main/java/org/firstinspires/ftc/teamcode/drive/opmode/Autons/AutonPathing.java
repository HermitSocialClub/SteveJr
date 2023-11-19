package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ElliotDrive;

@Autonomous (name = "AutonPathing")
public class AutonPathing extends LinearOpMode {

    public ElliotDrive drive;
   // SiddyDetector vision;

    @Override
    public void runOpMode() throws InterruptedException {

        ElliotDrive drive = new ElliotDrive(hardwareMap);
       // vision = new SiddyDetector(hardwareMap,telemetry);

        Pose2d starting = new Pose2d(35, -60, Math.toRadians(90));
        drive.setPoseEstimate(starting);

        Trajectory toMiddleSpike = drive.trajectoryBuilder(new Pose2d(starting.vec(),m(90)),m(90))
                .splineTo(new Vector2d(35,-35),m(90))
                .back(6)
                .build();

        Trajectory backToStartMiddle = drive.trajectoryBuilder(toMiddleSpike.end())
                .splineToSplineHeading(new Pose2d(35,-60,m(0)),m(90))
                .build();

        Trajectory toLeftSpike = drive.trajectoryBuilder(new Pose2d(starting.vec(),m(90)),m(90))
                .splineToSplineHeading(new Pose2d(40,-45,m(130)),m(90))
                .build();

        Trajectory backToStartLeft = drive.trajectoryBuilder(toLeftSpike.end())
                .splineToSplineHeading(new Pose2d(35,-60,m(0)),m(90))
                .build();

        Trajectory toRightSpike = drive.trajectoryBuilder(new Pose2d(starting.vec(),m(90)),m(90))
                .splineToSplineHeading(new Pose2d(30,-40,m(50)),m(90))
                .build();

        Trajectory backToStartRight = drive.trajectoryBuilder(toRightSpike.end())
                .splineToSplineHeading(new Pose2d(35,-60,m(0)),m(90))
                .build();

        waitForStart();
            if(isStopRequested()) return;
            drive.followTrajectory(toMiddleSpike);

    }

    public static double m (double degrees){
        return Math.toRadians(degrees);
    }

}
