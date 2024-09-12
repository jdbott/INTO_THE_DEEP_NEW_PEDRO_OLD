package org.firstinspires.ftc.teamcode.Legacy.RoadRunner.examples;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Legacy.RoadRunner.follower.MecanumDriveJasonChassis;

@TeleOp(name = "Jason Gamepad1 Controls")
public class JasonGamepad1Controls extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize the drive system with the starting position of the robot
        MecanumDriveJasonChassis drive = new MecanumDriveJasonChassis(hardwareMap, new Pose2d(60, -60, Math.toRadians(90)));

        // Initialize variable for heading lock, flags for modes and buttons,
        // and variables to control the robot’s movement
        boolean headingLock = false, rightStickPressed = false;
        double x, y, heading;

        // Wait for the start of the op mode
        waitForStart();

        // Main loop that runs while the op mode is active
        while (opModeIsActive()) {
            // Toggle heading lock mode when the right stick button is pressed
            if ((gamepad1.right_stick_button && !rightStickPressed) ||
                    ((gamepad1.right_stick_x > 0.5 || gamepad1.right_stick_x < -0.5) && headingLock)) {

                headingLock = !headingLock;
                rightStickPressed = gamepad1.right_stick_button; // Update the flag based on the button state
            } else if (!gamepad1.right_stick_button) {
                rightStickPressed = false; // Reset the flag when the stick is released
            }

            // Apply nonlinear scaling function to joystick inputs for finer control at low speeds
            x = scaleInput(-gamepad1.left_stick_x);
            y = scaleInput(-gamepad1.left_stick_y);

            if (headingLock) {
                // If heading lock is enabled, set auto heading and use stick input for x and y
                Rotation2d targetHeading = Rotation2d.exp(Math.toRadians(180));
                heading = targetHeading.minus(drive.pose.heading); // Auto heading when heading lock is enabled
            } else {
                // If heading lock is disabled, use stick input for x, y, and heading
                heading = scaleInput(-gamepad1.right_stick_x);
            }

            // Set the drive powers based on calculated x, y, and heading
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            y, // Yes, x and y are switched its correct
                            x
                    ),
                    heading
            ));

            // Update the robot's position estimate
            drive.updatePoseEstimate();
        }
    }

    private double scaleInput(double input) {
        // Example of a nonlinear scaling function (cubic scaling)
        return Math.pow(input, 3);
    }
}