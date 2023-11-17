package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "MainTele")
public class MainTele extends LinearOpMode {

    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor left_drive_2;
    DcMotor right_drive_2;
    DcMotor linear;
    double x;
    double x2;
    double y;
    double rx;
    double driveRightPower;
    double driveRight2Power;
    double driveLeftPower;
    double driveLeft2Power;
    double precisionModifier;
    boolean precisionMode = false;
    private boolean lastAMash = false;
   // Servo claw;
    DcMotor susPension;
    DcMotor intakeOne;
    DcMotor intakeTwo;
    Servo ramp;
    ElapsedTime runtime = new ElapsedTime();
  //  double linearPower;

    @Override
    public void runOpMode() throws InterruptedException {
        left_drive = hardwareMap.get(DcMotor.class, "left_drive");
        left_drive_2 = hardwareMap.get(DcMotor.class, "left_drive_2");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        right_drive_2 = hardwareMap.get(DcMotor.class, "right_drive_2");
        right_drive = hardwareMap.get(DcMotor.class, "right_drive");
        linear = hardwareMap.get(DcMotor.class, "linear");
        susPension = hardwareMap.get(DcMotor.class, "suspension");
        intakeOne = hardwareMap.get(DcMotor.class, "intakeOne");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeTwo");
        ramp = hardwareMap.get(Servo.class, "ramp");
       // claw = hardwareMap.get(Servo.class, "claw");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
//        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
        right_drive_2.setDirection(DcMotor.Direction.REVERSE);
//        right_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_drive = hardwareMap.get(DcMotor.class,"left_drive");
//        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_drive_2 = hardwareMap.get(DcMotor.class,"left_drive_2");
//        left_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ElapsedTime opmodeRunTime = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            opmodeRunTime.reset();
//            if (gamepad1.a) {
//                precisionMode = !precisionMode;
//                precisionModifier = precisionMode ? 1 : 0.5;
//                telemetry.speak((precisionMode ? "precision" : "speed"));
//                telemetry.addLine((precisionMode ? "ON" : "OFF"));
//            }
//            if (gamepad1.a) {
//
//                if (precisionMode) {
//                    precisionMode = false;
//                    precisionModifier = 1;
//                    // telemetry.addLine("Precision Mode DEACTIVATED!");
//                    telemetry.speak("speed");
//                    telemetry.addLine("precision mode deactivate");
//
//                } else {
//                    precisionMode = true;
//                    precisionModifier = 0.5;
//                    //telemetry.addLine("Precision Mode ACTIVATED!");
//                    telemetry.speak("precision");
//                    telemetry.addLine("precision mode activate");
//                }
//                runtime.reset();
//            }
//            if (gamepad1.a){
//                precisionMode = 0.5;
//            }else {
//                precisionMode = 0.8;
//            }

//            if (gamepad1.a = true){
//                precisionMode = true;
//                telemetry.addLine("precisionModeON");
//            } else if (gamepad1.a = false){
//                precisionMode = false;
//                telemetry.addLine("precisionModeOFF");
//            }else {
//                precisionMode = false;
//                telemetry.addLine("precisionModeOFF");
//            }



//            if (gamepad1.a) {
//                precisionMode = !precisionMode;
//                precisionModifier = precisionMode ? 1 : 0.5;
//                telemetry.speak((precisionMode ? "precision" : "speed"));
//                telemetry.addLine((precisionMode ? "ON" : "OFF"));
//            }

            if (gamepad1.a) {
                precisionMode = true;
                telemetry.speak("precision");
            }

            if (gamepad1.b) {
                precisionMode = false;
                telemetry.speak("speed");
            }

                    x = gamepad1.left_stick_x * 1;
                    x2 = gamepad1.left_stick_x * 1.5;
                    y = -gamepad1.left_stick_y;
                    rx = gamepad1.right_stick_x;
                    //     linearPower = gamepad2.right_stick_y;

                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double denominator2 = Math.max(Math.abs(y) + Math.abs(x2) + Math.abs(rx), 1);

            if (gamepad1.left_trigger > 0.5){
                driveRightPower = (((y - x - rx) / denominator) * 0.9);
                driveRight2Power = (((y + x - rx) / denominator) * 0.9);
                driveLeftPower = (((y + x + rx) / denominator) * 0.9);
                driveLeft2Power = (((y - x + rx) / denominator) * 0.9);
            } else {
                driveRightPower = (((y - x - rx) / denominator) * 0.5);
                driveRight2Power = (((y + x - rx) / denominator) * 0.5);
                driveLeftPower = (((y + x + rx) / denominator) * 0.5);
                driveLeft2Power = (((y - x + rx) / denominator) * 0.5);
            }
//            else {
//                driveRightPower = (((y - x - rx) / denominator) * 0.75);
//                driveRight2Power = (((y + x - rx) / denominator) * 0.75);
//                driveLeftPower = (((y + x + rx) / denominator) * 0.75);
//                driveLeft2Power = (((y - x + rx) / denominator) * 0.75);
//            }

//                    driveRightPower = (((y - x - rx) / denominator) * precisionModifier);
//                    driveRight2Power = (((y + x - rx) / denominator) * precisionModifier);
//                    driveLeftPower = (((y + x + rx) / denominator) * precisionModifier);
//                    driveLeft2Power = (((y - x + rx) / denominator) * precisionModifier);

                    left_drive.setPower(driveLeftPower);
                    left_drive_2.setPower(driveLeft2Power);
                    right_drive.setPower(driveRightPower);
                    right_drive_2.setPower(driveRight2Power);
                    linear.setPower(gamepad2.left_stick_y);
                    susPension.setPower(gamepad2.right_stick_y);

//                    if (gamepad2.right_trigger > 0.5){
//                        intakeOne.setPower(1);
//                        intakeTwo.setPower(-1);
//                    } else if (gamepad2.left_trigger > 0.5){
//                        intakeOne.setPower(-1);
//                        intakeTwo.setPower(1);
//                    }else {
//                        intakeOne.setPower(0);
//                        intakeTwo.setPower(0);
//                    }

                    if (gamepad1.left_trigger > 0.5){
                        ramp.setPosition(0.45);
                        intakeOne.setPower(1);
                        intakeTwo.setPower(-1);
                    } else {
                        ramp.setPosition(0.3);
                        intakeOne.setPower(0);
                        intakeTwo.setPower(0);
                    }

                    if (gamepad1.x){
                        intakeOne.setPower(-1);
                        intakeTwo.setPower(1);
                    }


//                    if (gamepad2.left_bumper) {
//                        // claw.setPower(-1);
//                        claw.setPosition(0);
//                    } else if (gamepad2.right_bumper) {
//                        //claw.setPower(0.4);
//                        claw.setPosition(0.7);
//                    } else {
//                        // claw.setPower(0);
//                        claw.setPosition(0);
//                    }

//            if (Math.abs(linearPower)<0.02){
//                linear.setPower(0);
//            } else {
//                linear.setPower(-linearPower);
//            }

                }
            }
        }