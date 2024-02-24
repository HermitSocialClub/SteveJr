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

    public static int armTarget = -200;
    public static double servoTarget = 0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private double servoTargetLeft = 0;
    private double servoTargetRight = 0;

    public static PIDCoefficients LINEAR_PID = new PIDCoefficients(0, 0, 0);

    public static PIDFController LINEAR_CONTROLLER = new PIDFController(LINEAR_PID);
    public static int linearTarget = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new NamjoonDrive(hardwareMap);

        drive.spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            drive.updateArmPID();

            drive.spool.setPower(-(((Math.abs(gamepad2.left_stick_y) < .2) ? 0 : gamepad2.left_stick_y) / .70) * 0.5);

            drive.clawFlipper.setPosition(servoTargetLeft);
            drive.clawRight.setPosition(servoTargetRight);

            if (gamepad1.y) {
                drive.flipperUp();
            }
            if (gamepad1.b){
                drive.flipperDown();
            }

            if (gamepad2.a)
            {
//                drive.closeRightClaw();
                servoTargetLeft += .01;
            }
            if (gamepad2.b)
            {
//                drive.closeLeftClaw();
                servoTargetRight += .01;
            }
            if (gamepad2.x)
            {
                servoTargetLeft -= .01;
            }
            if (gamepad2.y)
            {
//                drive.openRightClaw();
                servoTargetRight -= 0.01;
            }
            drive.ARM_CONTROLLER.setTargetPosition(armTarget);
            LINEAR_CONTROLLER.setTargetPosition(linearTarget);

            int spoolPos = drive.spoolEncoder.getCurrentPosition();

            double correction = LINEAR_CONTROLLER.update(spoolPos);
            drive.spool.setPower(correction);



            telemetry.addData("claw right position: ", servoTargetRight);
            telemetry.addData("claw left position: ", servoTargetLeft);
            telemetry.addData("ticks:", drive.chains.getCurrentPosition());
            telemetry.addData("leftFront ticks:", drive.leftFront.getCurrentPosition());
            telemetry.addData("target position:",target);
            telemetry.addData("controller value", gamepad2.right_stick_y);
            telemetry.addData("spoolPos: ", spoolPos);
            telemetry.addData("target spool pos: ", linearTarget);
            telemetry.update();


        }
    }
}
