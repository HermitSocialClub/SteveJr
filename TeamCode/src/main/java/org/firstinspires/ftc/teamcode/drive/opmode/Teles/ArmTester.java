package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;

@Config
@TeleOp(name = "ArmTester", group = "Elliot")
public class ArmTester extends LinearOpMode {
    NamjoonDrive drive;
    public static int target = 0;
    public static double servoTarget = 0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NamjoonDrive(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            drive.updateArmPID();
//            drive.clawRight.setPosition(servoTarget);

            if (gamepad2.a)
            {
                drive.closeRightClaw();
            }
            if (gamepad2.b)
            {
                drive.closeLeftClaw();
            }
            if (gamepad2.x)
            {
                drive.openLeftClaw();
            }
            if (gamepad2.y)
            {
                drive.openRightClaw();
            }

            telemetry.addData("claw right position: ", servoTarget);
            telemetry.addData("ticks:", drive.chains.getCurrentPosition());
            telemetry.addData("leftFront ticks:", drive.leftFront.getCurrentPosition());
            telemetry.addData("target position:",target);
            telemetry.addData("controller value", gamepad2.right_stick_y);
            telemetry.update();


        }
    }
}
