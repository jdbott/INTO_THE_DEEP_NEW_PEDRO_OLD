package org.firstinspires.ftc.teamcode.Legacy.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Mec Drive")
public class MecDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    private IMU imu;
    private YawPitchRollAngles orientation;

    private static final double DEADBAND_THRESHOLD = 0.05;
    private static final double ACCELERATION_RATE = 0.5;

    boolean headingLock = false;
    boolean rightStickPressed = false;
    private static final double HEADING_TARGET = 0.0;

    @Override
    public void runOpMode() {

        initRobot();

        waitForStart();

        runtime.reset();
        imu.resetYaw();

        while (opModeIsActive()) {
            driveCode();

            checkForImuReset();

            orientation = imu.getRobotYawPitchRollAngles();
            double currentHeading = orientation.getYaw(AngleUnit.DEGREES);

            telemetry.addData("Heading:", String.format("%.1f", currentHeading));
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }

    public void initRobot() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        orientation = imu.getRobotYawPitchRollAngles();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void driveCode() {

        if ((gamepad1.right_stick_button && !rightStickPressed)) {
            // Toggle the heading lock
            headingLock = !headingLock;
            rightStickPressed = true; // Set the flag to indicate the stick is pressed

        } else if ((gamepad1.right_stick_x > 0.5 || gamepad1.right_stick_x < -0.5) && headingLock) {
            // Toggle the heading lock
            headingLock = !headingLock;

        } else if (!gamepad1.right_stick_button) {
            rightStickPressed = false; // Reset the flag when the stick is released
        }

        if (!headingLock) {

            double max;

            double axial = -applyDeadband(gamepad1.left_stick_y, DEADBAND_THRESHOLD);  // Note: pushing stick forward gives negative value
            double lateral = applyDeadband(gamepad1.left_stick_x, DEADBAND_THRESHOLD);
            double yaw = applyDeadband(gamepad1.right_stick_x, DEADBAND_THRESHOLD);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (Math.abs(leftFrontPower - leftFront.getPower()) > ACCELERATION_RATE) {
                leftFrontPower = leftFront.getPower() + Math.signum(leftFrontPower -
                        leftFront.getPower()) * ACCELERATION_RATE;
            }
            if (Math.abs(rightFrontPower - rightFront.getPower()) > ACCELERATION_RATE) {
                rightFrontPower = rightFront.getPower() + Math.signum(rightFrontPower -
                        rightFront.getPower()) * ACCELERATION_RATE;
            }
            if (Math.abs(leftBackPower - leftBack.getPower()) > ACCELERATION_RATE) {
                leftBackPower = leftBack.getPower() + Math.signum(leftBackPower -
                        leftBack.getPower()) * ACCELERATION_RATE;
            }
            if (Math.abs(rightBackPower - rightBack.getPower()) > ACCELERATION_RATE) {
                rightBackPower = rightBack.getPower() + Math.signum(rightBackPower -
                        rightBack.getPower()) * ACCELERATION_RATE;
            }

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);

        } else {

            double max;

            double axial = -applyDeadband(gamepad1.left_stick_y, DEADBAND_THRESHOLD);  // Note: pushing stick forward gives negative value
            double lateral = applyDeadband(gamepad1.left_stick_x, DEADBAND_THRESHOLD);
            double yaw = calculateTurnPower(HEADING_TARGET);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            if (Math.abs(leftFrontPower - leftFront.getPower()) > ACCELERATION_RATE) {
                leftFrontPower = leftFront.getPower() + Math.signum(leftFrontPower -
                        leftFront.getPower()) * ACCELERATION_RATE;
            }
            if (Math.abs(rightFrontPower - rightFront.getPower()) > ACCELERATION_RATE) {
                rightFrontPower = rightFront.getPower() + Math.signum(rightFrontPower -
                        rightFront.getPower()) * ACCELERATION_RATE;
            }
            if (Math.abs(leftBackPower - leftBack.getPower()) > ACCELERATION_RATE) {
                leftBackPower = leftBack.getPower() + Math.signum(leftBackPower -
                        leftBack.getPower()) * ACCELERATION_RATE;
            }
            if (Math.abs(rightBackPower - rightBack.getPower()) > ACCELERATION_RATE) {
                rightBackPower = rightBack.getPower() + Math.signum(rightBackPower -
                        rightBack.getPower()) * ACCELERATION_RATE;
            }

            leftFront.setPower(leftFrontPower);
            rightFront.setPower(rightFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }
    }

    private double applyDeadband(double input, double threshold) {
        if (Math.abs(input) < threshold) {
            return 0.0;
        } else {
            return input;
        }
    }

    public double calculateTurnPower(double targetHeading) {
        // Constants for power limits
        double MAX_POWER = 0.9;
        double MIN_POWER = -0.9;

        // Proportional gain (adjust as needed)
        double KP = 0.02;

        double output;

        // Get current heading from the IMU
        orientation = imu.getRobotYawPitchRollAngles();
        double currentHeading = orientation.getYaw(AngleUnit.DEGREES);

        // Calculate the error (difference between target and current heading)
        double error = targetHeading - currentHeading;

        if (error > 3 || error < -3) {
            // Calculate proportional output (note the negative sign)
            output = -KP * error;
        } else {
            output = 0.0;
        }
        // Clip output to desired range
        output = Math.max(Math.min(output, MAX_POWER), MIN_POWER);

        return output;
    }

    public void checkForImuReset() {

        if (gamepad1.left_stick_button) {
            imu.resetYaw();
        }
    }
}