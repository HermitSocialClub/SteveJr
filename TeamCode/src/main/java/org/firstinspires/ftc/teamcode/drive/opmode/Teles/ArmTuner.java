package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;

@Config
@TeleOp(name = "ArmTuner", group = "Elliot")
public class ArmTuner extends LinearOpMode {
    NamjoonDrive drive;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0;
    public static double kA = 0;
    public static double kStatic = 0;
    public static double MAX_VEL = 0;
    public static double kG = 0.08;
    public static double target = 0;
    public static double ticks_per_deg = 752/360;
    public static PIDCoefficients ARM_PID = new PIDCoefficients(kP, kI, kD);
    public static PIDFController ARM_CONTROLLER = new PIDFController(ARM_PID, kV, kA, kStatic);
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NamjoonDrive(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        double initialLinearPos = drive.spoolEncoder.getCurrentPosition();
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a)
            {
                //update PID constants
                ARM_PID = new PIDCoefficients(kP, kI, kD);
                ARM_CONTROLLER = new PIDFController(ARM_PID, kA, kV, kStatic);
            }

            double spoolpos = drive.spoolEncoder.getCurrentPosition() - initialLinearPos;
            if (spoolpos <= 0) spoolpos = 1;
            spoolpos /= 1100;
            spoolpos += 1;
            double armPosition = -drive.chains.getCurrentPosition();
            if (armPosition <= 0) armPosition = 1;
            double armVelocity = drive.chains.getVelocity();
            double correction = ARM_CONTROLLER.update(armPosition, armVelocity);
            double feedforward = -Math.cos(Math.toRadians((armPosition/5.6) - 60)) * spoolpos * kG;
            drive.chains.setPower(feedforward + correction);

            telemetry.addData("pos: ", (armPosition/5.6) - 60);
            telemetry.addData("pos fr: ", armPosition);
            telemetry.addData("correction", correction);
            telemetry.addData("feedforward", feedforward);
            telemetry.addData("target: ", target);
            telemetry.addData("shit is ass", spoolpos);
            telemetry.update();

//            ARM_CONTROLLER.setTargetPosition(ar);

            if (gamepad1.y)
            {
                target++;
            }

        }
    }
}
