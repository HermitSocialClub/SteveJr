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
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.0001;
    public static double feedforward = 0.05;
    public static int target = -100;

    public static double ticks_per_deg = 752/180;

    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);

    PIDFController controller = new PIDFController(coeffs);

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NamjoonDrive(hardwareMap);
        drive.chains.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.chains.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            controller.setTargetPosition(target);
//            int armPos = drive.chains.getCurrentPosition();
//            double correction = controller.update(armPos,feedforward);
//
//            drive.chains.setPower(correction);
            //int target = drive.chains.getCurrentPosition();
            controller.setTargetPosition(target);
            int armPos = drive.chains.getCurrentPosition();
            double correction = controller.update(armPos,feedforward);

            drive.chains.setPower(correction);

//            if (Math.abs(gamepad2.right_stick_y) < 0.1 ){
//                drive.chains.setPower(2*(correction));
//            } else {
//                drive.chains.setPower(gamepad2.right_stick_y*0.6);
//            }

            telemetry.addData("ticks:", armPos);
            telemetry.addData("leftFront ticks:", drive.leftFront.getCurrentPosition());
            telemetry.addData("target position:",target);
            telemetry.addData("correction:",correction);
            telemetry.addData("controller value", gamepad2.right_stick_y);
            telemetry.update();


        }
    }
}
