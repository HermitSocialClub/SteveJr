package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ElliotDrive;
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;
import org.firstinspires.ftc.teamcode.drive.templates.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        NamjoonDrive drive = new NamjoonDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(((Math.abs(gamepad1.left_stick_y) < .2) ? 0 : gamepad1.left_stick_y) / .70) * (gamepad1.right_trigger > 0.05 ? 0.6 : 0.4),
                            -(((Math.abs(gamepad1.left_stick_x) < .2) ? 0 : gamepad1.left_stick_x) / .70) * (gamepad1.right_trigger > 0.05 ? 0.6 : 0.4),
                            -(((Math.abs(gamepad1.right_stick_x) < .2) ? 0 : gamepad1.right_stick_x) / .70) * 0.7 * (gamepad1.right_trigger > 0.05 ? 0.8 : 0.4)
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
