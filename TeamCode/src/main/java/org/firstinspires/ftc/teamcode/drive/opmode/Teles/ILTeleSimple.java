package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;


@TeleOp (name = "ILTeleSimple", group = "Elliot")
public class ILTeleSimple extends LinearOpMode {
    NamjoonDrive drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    ElapsedTime runtime = new ElapsedTime();
    int armTarget = 0;
    int linearTarget = 0;
    private double flipperPosition = 0;
    private boolean safe_mode = true;
    private double servoTargetLeft = 0;
    private boolean safe_mode_drive = true;
    private boolean winchTime = false;
    public boolean debugMode = true;

    double x = 0;
    double x2 = 0;
    double y = 0;
    double rx = 0;
    double y2 = 0;
    double driveRightPower = 0;
    double driveRight2Power = 0;
    double driveLeftPower = 0;
    double driveLeft2Power = 0;

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
           // drive.updateLinearPID();

            if (gamepad2.x)
            {
                safe_mode = !safe_mode;
            }

            if (gamepad1.x)
            {
                winchTime = !winchTime;
            }

            if (gamepad2.right_stick_y >= 0.1){
                armTarget += 20;
            } else if (gamepad2.right_stick_y <= -0.1){
                armTarget -= 20;
            }

           // drive.spool.setPower(-gamepad2.left_stick_y);

            if (safe_mode)
            {
                if (armTarget >= -30) {armTarget = -30;}
                if (armTarget <= -1150) {armTarget = -1150;}
            }

            if (winchTime)
            {
           //     linearTarget = 0;
            }

            drive.ARM_CONTROLLER.setTargetPosition(armTarget);
          //  drive.LINEAR_CONTROLLER.setTargetPosition(linearTarget);



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
            
            if (gamepad1.dpad_left){
//                drive.leftFront.setPower(-0.725);
//                drive.leftRear.setPower(0.65);
//                drive.rightFront.setPower(0.69);
//                drive.rightRear.setPower(-0.72);
                drive.leftFront.setPower(-0.69);
                drive.leftRear.setPower(0.7);
                drive.rightFront.setPower(0.7);
                drive.rightRear.setPower(-0.7125);
            } else if (gamepad1.dpad_right){
//                drive.leftFront.setPower(0.65);
//                drive.leftRear.setPower(-0.6);
//                drive.rightFront.setPower(-0.62);
//                drive.rightRear.setPower(0.67);
                drive.leftFront.setPower(0.7025);
                drive.leftRear.setPower(-0.7);
                drive.rightFront.setPower(-0.7);
                drive.rightRear.setPower(0.7125);
            } else if (gamepad1.dpad_up){
                drive.leftFront.setPower(0.525);
                drive.leftRear.setPower(0.5);
                drive.rightFront.setPower(0.52);
                drive.rightRear.setPower(0.5);
            } else if (gamepad1.dpad_down){
//                drive.leftFront.setPower(-0.525);
//                drive.leftRear.setPower(-0.5);
//                drive.rightFront.setPower(-0.52);
//                drive.rightRear.setPower(-0.5);
                drive.leftFront.setPower(-0.5);
                drive.leftRear.setPower(-0.5);
                drive.rightFront.setPower(-0.5);
                drive.rightRear.setPower(-0.5);
            }
            else {

                x = gamepad1.left_stick_x ;
                x2 = gamepad1.left_stick_x ;
                y = -gamepad1.left_stick_y;
                y2 = -gamepad1.left_stick_y ;
                rx = gamepad1.right_stick_x;

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                double denominator2 = Math.max(Math.abs(y) + Math.abs(x2) + Math.abs(rx), 1);

                driveRightPower = (((y2 - x2 - rx) / denominator2) );
                driveRight2Power = (((y + x - rx) / denominator) );
                driveLeftPower = ((y2 + x + rx) / denominator);
                driveLeft2Power = (((y - x2 + rx) / denominator2) );

            drive.rightRear.setPower(driveRight2Power);
            drive.rightFront.setPower(driveRightPower);
            drive.leftRear.setPower(driveLeft2Power);
            drive.leftFront.setPower(driveLeftPower);
//
            }


            telemetry.addData("Claw on Floor? ", drive.chains.getCurrentPosition() < clawOnFloorTicks);
            telemetry.addData("Winch Activated? ", winchTime);
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

                if (gamepad1.left_trigger > 0.5 && winchTime == true) {
                    drive.winch.setPower(-1);
                } else if (gamepad1.left_bumper && winchTime == true){
                    drive.winch.setPower(1);
                }
                else {
                    drive.winch.setPower(0);
                }

            }
        }
    }


