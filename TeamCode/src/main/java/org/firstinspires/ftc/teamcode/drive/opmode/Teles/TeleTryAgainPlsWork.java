package org.firstinspires.ftc.teamcode.drive.opmode.Teles;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "TeleTryAgain")

public class TeleTryAgainPlsWork extends LinearOpMode {

    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;

    HardwareMap hwmap = null;
    static final double ShoulderUp = 0.9;
    static final double Shoulder2Up = 0.1;
    static final double ShoulderDown =0.1;
    static final double Shoulder2Down =0.9;
    static final double LoadWrist = .1;
    static final double RightWrist = .20;
    static final double DumperAngle = .5;


    @Override
    public void runOpMode() throws InterruptedException {

        hwmap = hardwareMap;

        leftFront = hwmap.dcMotor.get("leftFront");
        leftRear = hwmap.dcMotor.get("leftRear");
        rightFront = hwmap.dcMotor.get("rightFront");
        rightRear = hwmap.dcMotor.get("rightRear");


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart( );

        telemetry.addData("Status", "Started");
        telemetry.update();

        while (opModeIsActive()) {


            double r = Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            leftFront.setPower(v3*0.9);
            rightFront.setPower(v4*1.1);
            leftRear.setPower(v1*1.1);
            rightRear.setPower(v2*1.1);


            telemetry.addData("Left", v1);
            telemetry.addData("Right", v2);
            telemetry.addData("LeftB", v3);
            telemetry.addData("Right", v4);

            telemetry.update();

            idle();

        }
    }
}
