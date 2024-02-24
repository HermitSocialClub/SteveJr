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
    int armTarget = 0;
    int linearTarget = 0;
    private double flipperPosition = 0;
    private boolean safe_mode = true;
    private double servoTargetLeft = 0;
    private boolean safe_mode_drive = true;
    private boolean winchTime = false;
    public boolean debugMode = false;

    private int clawOnFloorTicks = -30;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NamjoonDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        ElapsedTime opmodeRunTime = new ElapsedTime();
        flipperPosition = drive.clawFlipper.getPosition() / 0.5;
        //drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            opmodeRunTime.reset();

            drive.updateArmPID();
            if (winchTime)
            {
                drive.spool.setPower(0);
            }
            else {
                drive.updateLinearPID();
            }

            if (gamepad2.x)
            {
                safe_mode = !safe_mode;
            }

            if (gamepad2.right_stick_y >= 0.1){
                armTarget += 10;
            } else if (gamepad2.right_stick_y <= -0.1){
                armTarget -= 10;
            }
            if (gamepad2.left_stick_y >= 0.1){
                linearTarget -= 20;
            } else if (gamepad2.left_stick_y <= -0.1){
                linearTarget += 20;
            }
//            if (gamepad2.y)
//            {
//                armTarget = -30;
//            }

            if (safe_mode)
            {
                if (armTarget >= -30) {armTarget = -30;}
                if (armTarget <= -1150) {armTarget = -1150;}
                if (linearTarget <= 0) {linearTarget = 0;}
                if (linearTarget >= 950) {linearTarget = 950;}
            }

            drive.ARM_CONTROLLER.setTargetPosition(armTarget);
            drive.LINEAR_CONTROLLER.setTargetPosition(linearTarget);



            int lF = drive.leftFront.getCurrentPosition();
            int lR = drive.leftRear.getCurrentPosition();
            int rR = drive.rightRear.getCurrentPosition();
            int rF = drive.rightFront.getCurrentPosition();


            double vLF = drive.leftFront.getVelocity(AngleUnit.DEGREES);
            double vLR = drive.leftRear.getVelocity(AngleUnit.DEGREES);
            double vRF = drive.rightFront.getVelocity(AngleUnit.DEGREES);
            double vRR = drive.rightRear.getVelocity(AngleUnit.DEGREES);

            if (debugMode) {
                telemetry.addData("leftFront ticks: ", lF);
                telemetry.addData("arm ticks: ", drive.chains.getCurrentPosition());
                telemetry.addData("arm target ticks: ", armTarget);
                telemetry.addData("linear ticks: ", drive.spoolEncoder.getCurrentPosition());
                telemetry.addData("linear target ticks: ", linearTarget);
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
            }



            if (gamepad2.a)
            {
                clawOnFloorTicks = drive.chains.getCurrentPosition();
//                debugMode = !debugMode;
            }


            telemetry.addData("Claw on Floor? ", drive.chains.getCurrentPosition() < clawOnFloorTicks);
            telemetry.update();





            if (gamepad2.right_trigger > 0.5)
            {
//                drive.closeRightClaw();
                flipperPosition += 0.05;
//                drive.setFlipperPosition(drive.clawFlipper.getPosition() + .01);
            }
            if (gamepad2.left_trigger > 0.5)
            {
                flipperPosition -= 0.05;
//                drive.closeLeftClaw();
//                drive.setFlipperPosition(drive.clawFlipper.getPosition() - .01);
            }
            if (flipperPosition <= 0)
            {
                flipperPosition = 0;
            }
            if (flipperPosition >= 1)
            {
                flipperPosition = 1;
            }
            if (gamepad2.y) {
                drive.flipperUp();
            }
            if (gamepad2.b){
                drive.flipperDown();
            }
            drive.setFlipperPosition(flipperPosition);

//                if (gamepad2.left_trigger > 0.5){
//                    drive.flipperUp();
//                } else if (gamepad2.right_trigger > 0.5) {
//                    drive.clawFlipper.setPosition(0.25);
//                } else {
//                    drive.flipperDown();
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

                if (gamepad1.right_bumper){
                    drive.plane.setPosition(1);
                } else {
                    drive.plane.setPosition(0);
                }

                if (gamepad1.a){
                    winchTime = true;
                } else {
                    winchTime = false;
                }

                if (gamepad1.left_trigger > 0.5 && winchTime == true) {
                    drive.winch.setPower(-1);
                } else if (gamepad1.left_bumper && winchTime == true){
                    drive.winch.setPower(1);
                }
                else {
                    drive.winch.setPower(0);
                }

                if (gamepad1.y){
                    safe_mode_drive = !safe_mode_drive;
                }
//                    drive.rightFront.setPower(0.5);
//                    drive.rightRear.setPower(-0.9);
//                    drive.leftFront.setPower(-0.5);
//                    drive.leftRear.setPower(0.5);
//                } else if (gamepad1.x) {
//                    drive.rightFront.setPower(-0.5);
//                    drive.rightRear.setPower(0.6);
//                    drive.leftFront.setPower(0.5);
//                    drive.leftRear.setPower(-0.5);
//                } else {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -(((Math.abs(gamepad1.left_stick_y) < .2) ? 0 : gamepad1.left_stick_y) / .70) * (gamepad1.right_trigger > 0.05 ? 1 : 0.5),
                                    -((((Math.abs(gamepad1.left_stick_x) < .2) || safe_mode_drive) ? 0 : gamepad1.left_stick_x) / .70) * (gamepad1.right_trigger > 0.05 ? 1 : 0.5),
                                    -(((Math.abs(gamepad1.right_stick_x) < .2) ? 0 : gamepad1.right_stick_x) / .70) * 0.7 * (gamepad1.right_trigger > 0.05 ? 1 : 0.5)
                            ));
//                }

                // Read pose
                Pose2d poseEstimate = drive.getPoseEstimate();
            }
        }
    }


