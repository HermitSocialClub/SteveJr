package org.firstinspires.ftc.teamcode.drive.opmode.Autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

@Autonomous (name = "MeetZB1")
public class MeetZB1 extends LinearOpMode {

    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor left_drive_2;
    DcMotor right_drive_2;
    Double width = 16.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement
    //
    Double conversion = cpi * bias;
    Boolean exit = false;

    @Override

    public void runOpMode() throws InterruptedException {
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    left_drive_2 = hardwareMap.get(DcMotor.class, "left_drive_2");
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");
    right_drive_2= hardwareMap.get(DcMotor.class, "right_drive_2");
        left_drive.setDirection(DcMotor.Direction.REVERSE);
//        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
        left_drive_2.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();
    if (isStopRequested())return;

    while (opModeIsActive()){

        moveToPosition(0.1,60);

    }

    }

    public void goGo(double speed, double inches) {
        left_drive.setPower(speed);
    }
    public void moveToPosition(double speed, double inches){
        int move = (int)(Math.round(inches * conversion));

        left_drive.setTargetPosition(left_drive.getCurrentPosition() + move);
        left_drive_2.setTargetPosition((left_drive_2.getCurrentPosition() + move));
        right_drive.setTargetPosition(right_drive.getCurrentPosition() + move);
        right_drive_2.setTargetPosition(right_drive_2.getCurrentPosition() + move);


        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        left_drive.setPower(speed);
        right_drive.setPower(speed);
        left_drive_2.setPower(speed*0.5);
        right_drive_2.setPower(speed);
        //
        while (left_drive.isBusy() && left_drive_2.isBusy() && right_drive.isBusy() && right_drive_2.isBusy()){
            if (exit){
                left_drive_2.setPower(0);
                left_drive.setPower(0);
                right_drive_2.setPower(0);
                right_drive.setPower(0);
                return;
            }
        }
        left_drive_2.setPower(0);
        left_drive.setPower(0);
        right_drive_2.setPower(0);
        right_drive.setPower(0);
        return;
    }
}
