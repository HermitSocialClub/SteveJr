package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;

import kotlin.Unit;


@TeleOp (name = "ILTele", group = "Elliot")
public class ILTele extends LinearOpMode {
    NamjoonDrive drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime runtime = new ElapsedTime();
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    private boolean lastXMash = false;
    public boolean reverseDirections = false;
    public double reverseMod = 1;
    int target = 0;
    private boolean safe_mode = false;


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NamjoonDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        ElapsedTime opmodeRunTime = new ElapsedTime();
        //drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            opmodeRunTime.reset();

            drive.updateArmPID();

            if (gamepad2.x)
            {
                safe_mode = !safe_mode;
            }

            if (gamepad2.right_stick_y >= 0.1){
                target += 6;
            } else if (gamepad2.right_stick_y <= -0.1){
                target -= 6;
            }
            if (gamepad2.y)
            {
                target = -30;
            }

            if (safe_mode && target <= -30)
            {
                target = -30;
            }

            drive.ARM_CONTROLLER.setTargetPosition(target);



            int lF = drive.leftFront.getCurrentPosition();
            int lR = drive.leftRear.getCurrentPosition();
            int rR = drive.rightRear.getCurrentPosition();
            int rF = drive.rightFront.getCurrentPosition();


            double vLF = drive.leftFront.getVelocity(AngleUnit.DEGREES);
            double vLR = drive.leftRear.getVelocity(AngleUnit.DEGREES);
            double vRF = drive.rightFront.getVelocity(AngleUnit.DEGREES);
            double vRR = drive.rightRear.getVelocity(AngleUnit.DEGREES);

            telemetry.addData("leftFront ticks: ", lF);
            telemetry.addData("arm ticks: ", drive.chains.getCurrentPosition());
            telemetry.addData("arm target ticks: ", target);
            telemetry.addData("leftRear ticks: ", lR);
            telemetry.addData("rightFront ticks: ", rF);
            telemetry.addData("rightRear ticks: ", rR);
            telemetry.addData("leftFront velocity in degrees per sec: ", vLF);
            telemetry.addData("leftRear velocity in degrees per sec: ", vLR);
            telemetry.addData("rightFront velocity in degrees per sec: ", vRF);
            telemetry.addData("rightRear velocity in degrees per sec: ", vRR);
            telemetry.addData("leftFrontVolt",drive.leftFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("leftRearVolt",drive.leftRear.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightFrontVolt",drive.rightFront.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("rightRearVolt",drive.rightRear.getCurrent(CurrentUnit.AMPS));

            telemetry.update();




                if (gamepad1.right_bumper) {
                    drive.plane.setPower(-1);
                } else {
                    drive.plane.setPower(0);
                }

//                if (gamepad2.left_trigger > 0.5){
//                    drive.clawFlipper.setPosition(0.3);
//                } else if (gamepad2.right_trigger > 0.5) {
//                    drive.clawLeft.setPosition(0.15);
//                    drive.clawRight.setPosition(0.8);
//                } else {
//                    drive.clawFlipper.setPosition(0.87);
//                }

                if (gamepad2.left_bumper){
                    drive.openLeftClaw();
                }
                else
                {
                    drive.closeLeftClaw();
                }
                if (gamepad2.right_bumper){
                    drive.openRightClaw();
                }
                else
                {
                    drive.closeRightClaw();
                }

                if (gamepad1.y){
                    drive.rightFront.setPower(0.5);
                    drive.rightRear.setPower(-0.5);
                    drive.leftFront.setPower(-0.5);
                    drive.leftRear.setPower(0.5);
                } else if (gamepad1.x) {
                    drive.rightFront.setPower(-0.5);
                    drive.rightRear.setPower(0.5);
                    drive.leftFront.setPower(0.5);
                    drive.leftRear.setPower(-0.5);
                } else {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -(((Math.abs(gamepad1.left_stick_y) < .2) ? 0 : gamepad1.left_stick_y) / .70) * (gamepad1.right_trigger > 0.05 ? 1 : 0.5),
                                    -(((Math.abs(gamepad1.left_stick_x) < .2) ? 0 : gamepad1.left_stick_x) / .70) * (gamepad1.right_trigger > 0.05 ? 1 : 0.5),
                                    -(((Math.abs(gamepad1.right_stick_x) < .2) ? 0 : gamepad1.right_stick_x) / .70) * 0.7 * (gamepad1.right_trigger > 0.05 ? 1 : 0.5)
                            ));
                }

                drive.spool.setPower(-(((Math.abs(gamepad2.left_stick_y) < .2) ? 0.1 : gamepad2.left_stick_y) / .70) * 0.5);

                // Read pose
                Pose2d poseEstimate = drive.getPoseEstimate();
            }
        }
    }


