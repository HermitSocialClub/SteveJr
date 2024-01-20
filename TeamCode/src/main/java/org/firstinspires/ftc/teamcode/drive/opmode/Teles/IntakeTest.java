package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp (name = "IntakeTest")
public class IntakeTest extends LinearOpMode {
    public DcMotor intakeRight;

    public DcMotor intakeLeft;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                intakeRight.setPower(-1);
                intakeLeft.setPower(1);
            }else if (gamepad1.b){
                intakeRight.setPower(-0.6);
                intakeLeft.setPower(0.6);
            }else if (gamepad1.x){
                intakeRight.setPower(1);
                intakeLeft.setPower(-1);
            }
            else {
                intakeRight.setPower(0);
                intakeLeft.setPower(0);
            }

        }
    }
}
