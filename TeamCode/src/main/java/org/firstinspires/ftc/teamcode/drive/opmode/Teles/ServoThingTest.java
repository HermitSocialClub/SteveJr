package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ElliotDrive;

@TeleOp (name = "klanceTest")
public class ServoThingTest extends LinearOpMode {
    ElliotDrive drive;
    int pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new ElliotDrive(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a){
                pos+=1;
            }
            else if (gamepad1.b){
                pos-=1;
            }
            drive.klance.setPosition(pos);
            telemetry.addData("Servo pos: ", pos);
            telemetry.update();
        }
    }
}