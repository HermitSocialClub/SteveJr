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
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;


@TeleOp (name = "ILTele", group = "Elliot")
public class ILTele extends LinearOpMode {
    NamjoonDrive drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    double linearPower;

    ElapsedTime runtime = new ElapsedTime();
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    private boolean lastXMash = false;
    public boolean reverseDirections = false;
    public double reverseMod = 1;
    double driveRightPower;
    double driveLeftPower;
    double driveRight2Power;
    double driveLeft2Power;
    double y;
    double x;
    double rx;
    double x2;
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double feedforward = 0.05;
    public static double target = 0;

    public static double ticks_per_deg = 752/180;

    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

    PIDFController controller = new PIDFController(coeffs);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NamjoonDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        ElapsedTime opmodeRunTime = new ElapsedTime();
        drive.chains.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.chains.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            opmodeRunTime.reset();
         //   int target = drive.chains.getCurrentPosition();



            if (gamepad2.right_stick_y > 0){
                target = target + 6;
            } else if (gamepad2.right_stick_y < 0){
                target = target - 6;
            }

            controller.setTargetPosition(target);
            int armPos = drive.chains.getCurrentPosition();
            double correction = controller.update(armPos);

            drive.chains.setPower(correction);

//            if (Math.abs(gamepad2.right_stick_y) < 0.1 ){
//                drive.chains.setPower((correction+feedforward));
//            } else {
//                drive.chains.setPower(gamepad2.right_stick_y*0.5);
//            }

//            else if (gamepad2.a) {
//                drive.chains.setPower(gamepad2.right_stick_y);
//            }else {
//                drive.chains.setPower(gamepad2.right_stick_y*0.4);
//            }

            telemetry.addData("ticks:", armPos);
            telemetry.addData("leftFront ticks:", drive.leftFront.getCurrentPosition());
            telemetry.addData("target position:",target);
            telemetry.addData("correction:",correction);

            int lF = drive.leftFront.getCurrentPosition();
            int lR = drive.leftRear.getCurrentPosition();
            int rR = drive.rightRear.getCurrentPosition();
            int rF = drive.rightFront.getCurrentPosition();


            double vLF = drive.leftFront.getVelocity(AngleUnit.DEGREES);
            double vLR = drive.leftRear.getVelocity(AngleUnit.DEGREES);
            double vRF = drive.rightFront.getVelocity(AngleUnit.DEGREES);
            double vRR = drive.rightRear.getVelocity(AngleUnit.DEGREES);

            telemetry.addData("leftFront ticks: ", lF);
            telemetry.addData("leftRear ticks: ", lR);
            telemetry.addData("rightFront ticks: ", rF);
            telemetry.addData("rightRear ticks: ", rR);
            telemetry.addData("leftFront velocity in degrees per sec: ", vLF);
            telemetry.addData("leftRear velocity in degrees per sec: ", vLR);
            telemetry.addData("rightFront velocity in degrees per sec: ", vRF);
            telemetry.addData("rightRear velocity in degrees per sec: ", vRR);

            telemetry.update();

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(((Math.abs(gamepad1.left_stick_y) < .2) ? 0 : gamepad1.left_stick_y) / .70) * (gamepad1.right_trigger > 0.05 ? 1 : 0.5),
                            -(((Math.abs(gamepad1.left_stick_x) < .2) ? 0 : gamepad1.left_stick_x) / .70) * (gamepad1.right_trigger > 0.05 ? 1 : 0.5),
                            -(((Math.abs(gamepad1.right_stick_x) < .2) ? 0 : gamepad1.right_stick_x) / .70) * 0.7 * (gamepad1.right_trigger > 0.05 ? 1 : 0.6)
                    ));

//            x = gamepad1.left_stick_x * 1;
//            x2 = gamepad1.left_stick_x * 1;
//            y = -gamepad1.left_stick_y;
//            rx = gamepad1.right_stick_x;
//
////            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
////            double denominator2 = Math.max(Math.abs(y) + Math.abs(x2) + Math.abs(rx), 1);
////            driveRightPower = (y-x2-rx) / denominator2;
////            driveRight2Power = (y+x-rx) / denominator;
////            driveLeftPower = (y+x+rx) / denominator;
////            driveLeft2Power = (y-x2+rx) / denominator2;
//
//            drive.rightRear.setPower(driveRight2Power*0.92);
//            drive.rightFront.setPower(driveRightPower*0.97);
//            drive.leftRear.setPower(driveLeft2Power*0.93);
//            drive.leftFront.setPower(driveLeftPower);


                if (gamepad1.right_bumper) {
                    drive.plane.setPower(-1);
                } else {
                    drive.plane.setPower(0);
                }

                if (gamepad2.left_trigger > 0.5){
                    drive.clawFlipper.setPosition(0.3);
                } else if (gamepad2.right_trigger > 0.5) {
                    drive.clawLeft.setPosition(0.15);
                    drive.clawRight.setPosition(0.8);
                } else {
                    drive.clawFlipper.setPosition(0.87);
                }

                if (gamepad2.left_bumper){
                    drive.clawLeft.setPosition(0.15);
                    //drive.clawRight.setPosition(0.8);
                } else if (gamepad2.right_bumper){
                  // drive.clawLeft.setPosition(0.7);
                    drive.clawRight.setPosition(0.8);
                }
                else {
                    drive.clawLeft.setPosition(0.7);
                    drive.clawRight.setPosition(0.15);
                }





                drive.spool.setPower(-(((Math.abs(gamepad2.left_stick_y) < .2) ? 0 : gamepad2.left_stick_y) / .70) * 0.5);


                // Read pose
                Pose2d poseEstimate = drive.getPoseEstimate();
            }
        }
    }


