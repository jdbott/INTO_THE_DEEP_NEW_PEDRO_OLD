package org.firstinspires.ftc.teamcode.Legacy.RoadRunner.examples;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Legacy.RoadRunner.follower.MecanumDriveTeamChassis;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Lock To Tag")
public class LockToTagRR extends LinearOpMode {

    // Declare MecanumDrive object
    MecanumDriveTeamChassis drive;

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static int DESIRED_TAG_ID = -1;          // Choose the tag you want to approach or set to -1 for ANY tag.
    double DESIRED_DISTANCE = 6.0;                   //  this is how close the camera should get to the target (inches)
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private boolean isAutoAligned = false;
    Pose2d realTagPoseToRobot = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() {
        // Initialize the drive with starting position at (0, 0, 0) radians
        drive = new MecanumDriveTeamChassis(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        initAprilTag();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.update();

        // Wait for the start button to be pressed
        waitForStart();

        // Exit if stop is requested
        if (isStopRequested()) return;

        // Main loop - runs while the op mode is active and stop is not requested
        while (opModeIsActive() && !isStopRequested()) {
            processAprilTagDetections();

            // Call the lockTo function with the target position (0, 0, 0) radians
            lockTo(realTagPoseToRobot);

            // Optional sleep to prevent excessive looping
            sleep(50);
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

        // Set the drive powers based on the proportional control calculations
        drive.setDrivePowers(new PoseVelocity2d(xy.times(xyP), heading * headingP));
        // Update the robot's pose estimate
        drive.updatePoseEstimate();
    }

    public void processAprilTagDetections() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {
                // AprilTag is detected, calculate the robot's position
                Pose2d robotPose = calculateRobotPose(detection);
                realTagPoseToRobot = drive.pose.minusExp(robotPose);
                telemetry.addData("Robot Position", robotPose);
                telemetry.update();
            }
        }
    }

    public Pose2d calculateRobotPose(AprilTagDetection detection) {
        // Assume the tag is at the origin (0, 0, 0)
        // Convert the tag pose to robot pose in the field coordinate system
        // You can adjust this logic based on your specific field and robot configuration
        double x = detection.ftcPose.x; // Convert meters to inches
        double y = detection.ftcPose.y; // Convert meters to inches
        double heading = Math.toRadians(detection.ftcPose.yaw);

        // Return the robot's pose
        return new Pose2d(x, y, heading);
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}