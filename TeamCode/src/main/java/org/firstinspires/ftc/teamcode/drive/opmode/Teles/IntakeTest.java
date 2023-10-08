package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "IntakeTest")
public class IntakeTest extends LinearOpMode {
    public DcMotor intakeOne;

    public DcMotor intakeTwo;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeOne = hardwareMap.get(DcMotor.class, "intakeOne");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwo");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intakeOne.setPower(0.6);
                intakeTwo.setPower(-0.6);
            }else if (gamepad1.b){
                intakeOne.setPower(-0.75);
                intakeTwo.setPower(0.75);
            }else if (gamepad1.x){
                intakeOne.setPower(1);
                intakeTwo.setPower(-1);
            }
            else {
                intakeOne.setPower(0);
                intakeTwo.setPower(0);
            }

        }
    }
}
