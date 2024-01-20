package org.firstinspires.ftc.teamcode.drive.opmode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp (name = "MainTele")
public class MainTele extends LinearOpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
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
    Servo trap;
    Servo poleGrabber;
    CRServo plane;
  //  double suspensionSpeed;
    ElapsedTime runtime = new ElapsedTime();
   // ElliotDrive drive;
  //  double linearPower;

    @Override
    public void runOpMode() throws InterruptedException {
      // drive = new ElliotDrive(hardwareMap);
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
       // rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        linear = hardwareMap.get(DcMotor.class, "linears");
        susPension = hardwareMap.get(DcMotor.class, "susPension");
        intakeOne = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeTwo = hardwareMap.get(DcMotor.class, "intakeRight");
        ramp = hardwareMap.get(Servo.class, "ramp");
        trap = hardwareMap.get(Servo.class, "trap");
        poleGrabber = hardwareMap.get(Servo.class, "poleGrabber");
        plane = hardwareMap.get(CRServo.class, "plane");

       // claw = hardwareMap.get(Servo.class, "claw");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
//        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

            if (gamepad1.left_bumper){
                driveRightPower = (((y - x - rx) / denominator) * 0.9);
                driveRight2Power = (((y + x - rx) / denominator) * 0.9);
                driveLeftPower = (((y + x + rx) / denominator) * 0.95);
                driveLeft2Power = (((y - x + rx) / denominator) * 0.9);
            } else {
                driveRightPower = (((y - x - rx) / denominator) * 0.8);
                driveRight2Power = (((y + x - rx) / denominator) * 0.8);
                driveLeftPower = (((y + x + rx) / denominator) * 0.9);
                driveLeft2Power = (((y - x + rx) / denominator) * 0.8);
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

//            if (gamepad2.right_stick_y >= 0.1){
//                suspensionSpeed = 1;
//            } if (gamepad2.right_stick_y <= -0.1) {
//                suspensionSpeed = -1;
//            } else {
//                suspensionSpeed = 0;
//            }

                    leftFront.setPower(driveLeftPower);
                    leftRear.setPower(driveLeft2Power);
                    rightFront.setPower(driveRightPower);
                    rightRear.setPower(driveRight2Power);
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
                        ramp.setPosition(0.42 );
                      //  intakeOne.setPower(1);
                     //   intakeTwo.setPower(-1);
                    } else {
                        ramp.setPosition(0.46);
                        intakeOne.setPower(0);
                        intakeTwo.setPower(0);
                    }

                    if (gamepad1.right_trigger > 0.5){
                        intakeOne.setPower(0.9);
                        intakeTwo.setPower(-0.9);
                    }

                    if (gamepad1.x){
                        intakeOne.setPower(-1);
                        intakeTwo.setPower(1);
                    }

//                    if (gamepad2.left_trigger > 0.5){
//                        trap.setPosition(1);
//                    } else {
//                        trap.setPosition(0.5);
//                    }

//                    if (gamepad2.a){
//                        trap.setPosition(1);
//                    } else if (gamepad2.b){
//                        trap.setPosition(0.7);
//                    } else if (gamepad2.x){
//                        trap.setPosition(0);
//                    } else if (gamepad2.y){
//                        trap.setPosition(0.4);
//                    }

                    if (gamepad2.right_trigger > 0.5){
                        poleGrabber.setPosition(0.75);
                    } else {
                        poleGrabber.setPosition(0);
                    }

            if (gamepad2.left_trigger > 0.3){
                trap.setPosition(0.7);
            } else {
                trap.setPosition(0.2);
            }

            if (gamepad2.left_bumper){
                plane.setPower(1);
            } else {
                plane.setPower(0);
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