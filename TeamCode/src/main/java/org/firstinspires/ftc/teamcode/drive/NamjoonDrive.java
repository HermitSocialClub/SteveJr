package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.DIRECTIONS;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREVColorDistance;
import org.firstinspires.ftc.teamcode.drive.opmode.Utils.MathUtils;
import org.firstinspires.ftc.teamcode.drive.templates.BigNamjoonLocalizer;
import org.firstinspires.ftc.teamcode.drive.NamjoonDriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class NamjoonDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(12.5,0,1);//(3,0,1)(8,0,1);//(2.5, 0, 1.5); 3,0,1 but it broke life
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(15,0,1);//8,0,1);//(5 , 0, 1);
    public static PIDCoefficients ARM_PID = new PIDCoefficients(0.01, 0, 0.0001);
    public static PIDCoefficients LINEAR_PID = new PIDCoefficients(0.01, 0, 0.0001);
    public static PIDFController ARM_CONTROLLER = new PIDFController(ARM_PID);
    public static PIDFController LINEAR_CONTROLLER = new PIDFController(LINEAR_PID);
    public static double LATERAL_MULTIPLIER = 1.41676714; //2.67;//1.55;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static double MAX_VEL_CHANGABLE = MAX_VEL;
    public static double MAX_ACC_CHANGABLE = MAX_ACCEL;
    public static double MAX_ANG_VEL_CHANGABLE = MAX_ANG_VEL;//Math.toRadians(220.1650190537944);
    public static double MAX_ANG_ACC_CHANGABLE = MAX_ANG_ACCEL;//Math.toRadians(220.1650190537944);
    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL_CHANGABLE, MAX_ANG_VEL_CHANGABLE, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACC_CHANGABLE);

    private TrajectoryFollower follower;
    int targetDist = 0;

    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public DcMotorEx spool;
    public Encoder spoolEncoder;
    public DcMotorEx chains;
    public Servo clawLeft;
    public Servo clawRight;
    public ColorSensor backDistanceSensor;
    public Servo clawFlipper;
    public Servo plane;

    public DcMotorEx winch;

    private List<DcMotorEx> motors;
    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    private List<Integer> lastEncPositions = new ArrayList<>();
    private List<Integer> lastEncVels = new ArrayList<>();

    public NamjoonDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);


        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        spool = hardwareMap.get(DcMotorEx.class, "spool");
        chains = hardwareMap.get(DcMotorEx.class, "chains");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        clawFlipper = hardwareMap.get(Servo.class,"clawFlipper");
        spoolEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,"leftFront"));
        winch = hardwareMap.get(DcMotorEx.class, "winch");
        backDistanceSensor = hardwareMap.get(ColorSensor.class, "backSensor");

        plane = hardwareMap.get(Servo.class, "plane");
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //linears.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        chains.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        //setLocalizer(new StandardTrackingWheelLocalizer(hardwareMap));

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        //NOTE: WE CANNOT USE RUN_USING_ENCODER WE DO NOT HAVE ENOUGH ENCODERS
        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        //set chains to PID only and set them to dead
        chains.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chains.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);         //this port has the spool encoder
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LINEAR_CONTROLLER.setTargetPosition(0);
        ARM_CONTROLLER.setTargetPosition(0);

        // TODO: reverse any motors using DcMotor.setDirection()
        setMotorDirections(DIRECTIONS);

        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();

        // TODO: if desired, use setLocalizer() to change the localization method
         setLocalizer(new BigNamjoonLocalizer(hardwareMap));

        trajectorySequenceRunner = new TrajectorySequenceRunner(
                follower, HEADING_PID, batteryVoltageSensor,
                lastEncPositions, lastEncVels, lastTrackingEncPositions, lastTrackingEncVels
        );    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }


    public void openLeftClaw()
    {
        openLeftClaw(0.2);
    }

    public void openLeftClaw(double position)
    {
        clawLeft.setPosition(position);
    }

    public void openRightClaw()
    {
        openRightClaw(1);
    }

    public void openRightClaw(double position)
    {
        clawRight.setPosition(position);
    }

    public void closeLeftClaw()
    {
        closeLeftClaw(0.65);
    }

    public void closeLeftClaw(double position)
    {
        clawLeft.setPosition(position);
    }

    public void closeRightClaw()
    {
        closeRightClaw(0.65);
    }

    public void fastMode (){
        MAX_ANG_ACC_CHANGABLE = Math.toRadians(382.81942382812497);
        MAX_ANG_VEL_CHANGABLE = Math.toRadians(382.81942382812497);
        MAX_ACC_CHANGABLE = 82;
        MAX_VEL_CHANGABLE = 82;
    }

    public void slowMode () {
        MAX_ANG_ACC_CHANGABLE = MAX_ANG_ACCEL;
        MAX_ANG_VEL_CHANGABLE = MAX_ANG_VEL;
        MAX_ACC_CHANGABLE = MAX_ACCEL;
        MAX_VEL_CHANGABLE = MAX_VEL;
    }

    public void flipperUp()
    {
        clawFlipper.setPosition(0);
    }

    public void flipperDown() {
        clawFlipper.setPosition(0.5);
    }

    public void setFlipperPosition(double pos)
    {
        if (pos > 1)
        {
            pos = 1;
        }
        if (pos < 0)
        {
            pos = 0;
        }
        clawFlipper.setPosition(pos * 0.5);
    }

    public void closeRightClaw(double position)
    {
        clawRight.setPosition(position);
    }


    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }


    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL_CHANGABLE, MAX_ANG_ACC_CHANGABLE
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void updateArmPID()
    {
        int armPos = chains.getCurrentPosition();
        double correction = ARM_CONTROLLER.update(armPos);
        correction = MathUtils.clamp(correction, -0.7, 0.7);
        chains.setPower(correction);
    }
    public void updateLinearPID()
    {
        int linearPos = spoolEncoder.getCurrentPosition();
        double correction = LINEAR_CONTROLLER.update(linearPos);
        spool.setPower(correction);
    }

    public void updateAllPIDs()
    {
        updateLinearPID();
        updateArmPID();
    }
    public void update() {

        //arm PID loop
//        updateArmPID();
//        updateLinearPID();
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setMotorDirections (DcMotorSimple.Direction[] directions){

        int i = 0;
        for (DcMotorSimple.Direction direction : directions) {
            motors.get(i).setDirection(direction);
            i++;
        }

    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        lastEncPositions.clear();

        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int position = motor.getCurrentPosition();
            lastEncPositions.add(position);
            wheelPositions.add(NamjoonDriveConstants.encoderTicksToInches(position));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        lastEncVels.clear();

        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            int vel = (int) motor.getVelocity();
            lastEncVels.add(vel);
            wheelVelocities.add(NamjoonDriveConstants.encoderTicksToInches(vel));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return 0;
    }
    @Override
    public Double getExternalHeadingVelocity() {
        return (double) imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }
}
