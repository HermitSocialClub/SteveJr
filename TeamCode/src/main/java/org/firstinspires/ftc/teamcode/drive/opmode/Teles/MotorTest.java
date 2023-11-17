package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ElliotDrive;

@TeleOp (name = "MotorTest")
public class MotorTest extends LinearOpMode {
    ElliotDrive drive;
    DcMotor right_drive;
    DcMotor right_drive_2;
    DcMotor left_drive;
    DcMotor left_drive_2;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new ElliotDrive(hardwareMap);
//        left_drive = hardwareMap.get(DcMotor.class, "leftFront") ;
//        left_drive_2 = hardwareMap.get(DcMotor.class, "leftRear") ;
//        right_drive = hardwareMap.get(DcMotor.class, "rightFront") ;
//        right_drive_2= hardwareMap.get(DcMotor.class, "rightRear") ;
//        right_drive.setDirection(DcMotor.Direction.REVERSE);
//        left_drive.setDirection();
////        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
//        right_drive_2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a){
                drive.leftFront.setPower(0.5);
            } else {
                drive.leftFront.setPower(0);
            }
            if (gamepad1.b){
                drive.leftRear.setPower(0.5);
            } else {
                drive.leftRear.setPower(0);
            }
            if (gamepad1.x){
                drive.rightFront.setPower(0.5);
            }else {
                drive.rightFront.setPower(0);
            }
            if (gamepad1.y){
                drive.rightRear.setPower(0.5);
            }else {
                drive.rightRear.setPower(0);
            }
//            left_drive.setPower(gamepad1.left_stick_x);
        }
        }
    }