package org.firstinspires.ftc.teamcode.Legacy.RoadRunner.examples;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Legacy.RoadRunner.follower.MecanumDriveJasonChassis;

@TeleOp(name = "Lock To Test")
public class LockTo extends LinearOpMode {

    // Declare MecanumDrive object
    MecanumDriveJasonChassis drive;

    RevBlinkinLedDriver lights;

    @Override
    public void runOpMode() {
        // Initialize the drive with starting position at (0, 0, 0) radians
        drive = new MecanumDriveJasonChassis(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");

        // Wait for the start button to be pressed
        waitForStart();

        // Exit if stop is requested
        if (isStopRequested()) return;

        // Main loop - runs while the op mode is active and stop is not requested
        while (opModeIsActive() && !isStopRequested()) {
            lockTo(new Pose2d(0, 0, Math.toRadians(0)));
            drive.updatePoseEstimate();
        }
    }

    // Function to lock the robot to a specified target position
    public void lockTo(Pose2d targetPos) {
        // Proportional constants for position and heading control
        double xyP = 0.06;
        double headingP = 1.0;

        // Get the current position of the robot
        Pose2d currentPos = drive.pose;
        // Calculate the difference between the target position and the current position
        Pose2d difference = Pose2d.exp(targetPos.minus(currentPos));

        // Extract the positional and heading differences
        Vector2d xy = difference.position;
        double heading = difference.heading.toDouble();

        if (xy.norm() > 1.5 || heading > Math.toRadians(15)) {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        } else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }

        // Set the drive powers based on the proportional control calculations
        drive.setDrivePowers(new PoseVelocity2d(xy.times(xyP), heading * headingP));
    }
}