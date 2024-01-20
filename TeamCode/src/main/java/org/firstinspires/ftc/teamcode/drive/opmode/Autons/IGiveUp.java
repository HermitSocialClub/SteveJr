package org.firstinspires.ftc.teamcode.drive.opmode.Autons;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ElliotDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous (name = "IGiveUp")
public class IGiveUp extends LinearOpMode {

    public ElliotDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {

        ElliotDrive drive = new ElliotDrive(hardwareMap);

        Pose2d starting = new Pose2d(15, 63, Math.toRadians(90));
        drive.setPoseEstimate(starting);

        TrajectorySequence toMiddleSequence = drive.trajectorySequenceBuilder(new Pose2d(starting.vec(), m(90)))
//                .addDisplacementMarker(0,()->{
//                    drive.klance.setPosition(0.5);
//                })
                .lineTo(new Vector2d(15, 32))
                .lineTo(new Vector2d(15, 40))
                .turn(m(90))
                //.waitSeconds(0.5)
                // .back(32)
                .build();


        waitForStart();
        if (isStopRequested()) return;


        drive.followTrajectorySequence(toMiddleSequence);

    }

    public static double m (double degrees){
        return Math.toRadians(degrees);
    }

}
