package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ElliotDrive;


@TeleOp (name = "Meet3Tele", group = "Elliot")
public class Meet3Tele extends LinearOpMode {
    //public DcMotor linear = null;
    // public CRServo intake = null;


    //HardwareMap hwMap           =  new HardwareMap();
    ElliotDrive drive;
    double linearPower;
    boolean yesClaw;
    double clawPosition;
    public Servo claw;

    ElapsedTime runtime = new ElapsedTime();
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    private boolean lastXMash = false;
    public boolean reverseDirections = false;
    public double reverseMod = 1;
    //    public boolean precisionMode = false;
    public boolean lockedStrafe = false;
    //    public double precisionModifier = 0.9;
    double driveRightPower;
    double driveLeftPower;
    double driveRight2Power;
    double driveLeft2Power;
    double y;
    double x;
    double rx;
    double x2;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new ElliotDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        ElapsedTime opmodeRunTime = new ElapsedTime();


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            opmodeRunTime.reset();
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(((Math.abs(gamepad1.left_stick_y) < .2) ? 0 : gamepad1.left_stick_y) / .70) * (gamepad1.right_trigger > 0.05 ? 1 : 0.7),
                            (((Math.abs(gamepad1.left_stick_x) < .2) ? 0 : gamepad1.left_stick_x) / .70) * (gamepad1.right_trigger > 0.05 ? 1 : 0.7),
                            -(((Math.abs(gamepad1.right_stick_x) < .2) ? 0 : gamepad1.right_stick_x) / .70) * 0.7 * (gamepad1.right_trigger > 0.05 ? 1 : 0.7)
                    ));

          //  drive.linears.setPower(-(((Math.abs(gamepad2.left_stick_y) < .2) ? 0 : gamepad2.left_stick_y) / .70) * 0.5);
            drive.susPension.setPower(gamepad2.right_stick_y);

//            if (gamepad1.left_trigger > 0.5){
//                drive.ramp.setPosition(0.42 );
//                drive.trap.setPosition(0.25);
//                //  intakeOne.setPower(1);
//                //   intakeTwo.setPower(-1);
//            } else {
//                drive.ramp.setPosition(0.46);
//                drive.intakeLeft.setPower(0);
//                drive.intakeRight.setPower(0);
//            }


            if (gamepad2.left_trigger > 0.3) {
                drive.clawLeft.setPosition(0.5);
                drive.clawRight.setPosition(0.25);
                // drive.ramp.setPosition(0.5);
            } else {
                drive.clawLeft.setPosition(0.25);
                drive.clawRight.setPosition(0.5);


                if (gamepad1.a) {
                    drive.clawRight.setPosition(0);
                } else if (gamepad1.b) {
                    drive.clawLeft.setPosition(0);
                } else if (gamepad1.x) {
                    drive.clawLeft.setPosition(0.5);
                } else if (gamepad1.y) {
                    drive.clawRight.setPosition(0.5);
                }

                //else {
//                drive.clawRight.setPosition(1);
//                drive.clawLeft.setPosition(1);
//            }


//            if (gamepad1.x){
//                drive.intakeLeft.setPower(1);
//                drive.intakeRight.setPower(-1);
//            }
                if (gamepad2.right_trigger > 0.3) {
                    drive.poleGrabber.setPosition(0);
                } else {
                    drive.poleGrabber.setPosition(0.35);
                }


                //else if (gamepad2.right_bumper){
                //    drive.ramp.setPosition(0.25);
                //}
//        else {
//                drive.ramp.setPosition(0.025);
//            }

//            if (gamepad2.right_bumper){
//                drive.trap.setPosition(0.3);
//            }else {
//                drive.trap.setPosition(1);
//            }

                if (gamepad2.right_bumper) {
                    drive.plane.setPower(-1);
                } else {
                    drive.plane.setPower(0);
                }

                if (gamepad2.left_bumper) {
                    drive.ramp.setPosition(0.5);
                } else {
                    drive.ramp.setPosition(0);
                }


                // Read pose
                Pose2d poseEstimate = drive.getPoseEstimate();
            }
        }
    }
}


