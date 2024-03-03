package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.NamjoonDrive;

@TeleOp(name = "Distance", group = "Elliot")
public class TestDistanceSensor extends LinearOpMode {
    NamjoonDrive drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());

        drive = new NamjoonDrive(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            if (drive.backDistanceSensor instanceof DistanceSensor)
            {
                telemetry.addData("distance: ", ((DistanceSensor) drive.backDistanceSensor).getDistance(DistanceUnit.CM));
            }
        }
    }
}
