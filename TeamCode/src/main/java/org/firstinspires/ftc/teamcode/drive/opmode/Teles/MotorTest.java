package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "MotorTest")
public class MotorTest extends LinearOpMode {
    DcMotor right_drive;
    DcMotor right_drive_2;
    DcMotor left_drive;
    DcMotor left_drive_2;
    @Override
    public void runOpMode() throws InterruptedException {


        left_drive = hardwareMap.get(DcMotor.class, "left_drive") ;
        left_drive_2 = hardwareMap.get(DcMotor.class, "left_drive_2") ;
        right_drive = hardwareMap.get(DcMotor.class, "right_drive") ;
        right_drive_2= hardwareMap.get(DcMotor.class, "right_drive_2") ;
        right_drive.setDirection(DcMotor.Direction.REVERSE);
//        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
        right_drive_2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a){
                left_drive.setPower(0.5);
            } else {
                left_drive.setPower(0);
            }
            if (gamepad1.b){
                left_drive_2.setPower(0.5);
            } else {
                left_drive_2.setPower(0);
            }
            if (gamepad1.x){
                right_drive.setPower(0.5);
            }else {
                right_drive.setPower(0);
            }
            if (gamepad1.y){
                right_drive_2.setPower(0.5);
            }else {
                right_drive_2.setPower(0);
            }
            left_drive.setPower(gamepad1.left_stick_x);
        }
        }
    }