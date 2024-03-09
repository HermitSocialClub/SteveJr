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
import org.firstinspires.ftc.teamcode.drive.opmode.Utils.MathUtils;

@Config
@TeleOp(name = "ArmTuner", group = "Elliot")
public class ArmTuner extends LinearOpMode {
    NamjoonDrive drive;

    public static double kP = 0.0035;
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
    public static PIDFController ARM_CONTROLLER = new PIDFController(ARM_PID);
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    double lastPower = 0;
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
                ARM_CONTROLLER = new PIDFController(ARM_PID);
            }

            double spoolpos = drive.spoolEncoder.getCurrentPosition() - initialLinearPos;
            if (spoolpos <= 0) spoolpos = 1;
            spoolpos /= 1100;
            spoolpos += 1;
            double armPosition = -drive.chains.getCurrentPosition();
            if (armPosition <= 0) armPosition = 1;
            double armVelocity = drive.chains.getVelocity();
            double correction = ARM_CONTROLLER.update(armPosition);
            double feedforward = -Math.cos(Math.toRadians((armPosition/5.6) - 60)) * spoolpos * kG;
            double maxDiff = ((feedforward - correction)-lastPower);
            if (Math.abs(maxDiff) > .07)
            {
                double sign = Math.signum(maxDiff);
                maxDiff = 0.07 * sign;
            }
            lastPower += maxDiff;
            drive.chains.setPower(lastPower);

            telemetry.addData("pos: ", (armPosition/5.6) - 60);
            telemetry.addData("pos fr: ", armPosition);
            telemetry.addData("correction", correction);
            telemetry.addData("feedforward", feedforward);
            telemetry.addData("target: ", target);
            telemetry.addData("shit is ass", spoolpos);
            telemetry.update();

            ARM_CONTROLLER.setTargetPosition(target);

            if (gamepad1.y)
            {
                target++;
            }

        }
    }
}
