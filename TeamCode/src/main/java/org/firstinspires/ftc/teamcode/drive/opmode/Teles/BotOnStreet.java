package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.ElliotDrive;


@TeleOp (name = "BotOnStreet", group = "Elliot")
public class BotOnStreet extends LinearOpMode {
    //public DcMotor linear = null;
    // public CRServo intake = null;


    //HardwareMap hwMap           =  new HardwareMap();
    DcMotor rightFront;
    DcMotor rightRear;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor linear;
    double drivePower;
    double turnPower;
    double linearPower;

    @Override
    public void runOpMode() throws InterruptedException {

        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        linear = hardwareMap.get(DcMotor.class, "linear");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            drivePower = gamepad1.left_stick_y;
            turnPower = gamepad1.right_stick_x;

            rightFront.setPower(drivePower + turnPower);
            rightRear.setPower(drivePower + turnPower);
            leftFront.setPower(drivePower - turnPower);
            leftRear.setPower(drivePower - turnPower);

            linear.setPower(gamepad1.right_stick_y);



            }
        }
    }


