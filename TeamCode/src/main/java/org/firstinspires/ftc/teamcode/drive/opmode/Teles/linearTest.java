package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "linearTest")
public class linearTest extends LinearOpMode {

    public DcMotor linear;
    @Override
    public void runOpMode() throws InterruptedException {
        linear = hardwareMap.get(DcMotor.class, "linear");

        waitForStart();
        if (isStopRequested())return;

        while (opModeIsActive()){
            linear.setPower(gamepad1.left_stick_y);
            if (gamepad1.dpad_up){
                linear.setPower(0.75);
            }
        }

    }
}
