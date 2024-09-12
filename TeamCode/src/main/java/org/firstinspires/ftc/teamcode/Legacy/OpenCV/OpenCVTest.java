package org.firstinspires.ftc.teamcode.Legacy.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * OpenCVTest.java
 *
 * This code is an FTC (FIRST Tech Challenge) teleop mode that uses OpenCV to process images from a webcam.
 * It identifies which section of the image has the most red and blue colors and displays this information
 * on the telemetry. The image is divided into three equal horizontal sections, and the red and blue color
 * ratios are calculated for each section.
 */
@TeleOp
public class OpenCVTest extends LinearOpMode {
    // Declare variables for the webcam and the OpenCV pipeline
    private CameraManagerTeamProp cameraManager;

    /**
     * The main method for the op mode. This method sets up and runs the OpenCV pipeline.
     */
    @Override
    public void runOpMode() {

        cameraManager = new CameraManagerTeamProp(hardwareMap);

        // Notify that the op mode is waiting for the start signal
        telemetry.addLine("Waiting for start");
        telemetry.update();

        // Wait for the start signal
        waitForStart();

        // Run the op mode while it is active
        while (opModeIsActive()) {
            // Get the sections with the most red and blue colors
            String redSection = cameraManager.getMostRedSection();

            // Display the sections with the most red and blue
            telemetry.addData("Section with most red", redSection);

            // Update the telemetry display
            telemetry.update();

            // Sleep for a short duration to avoid excessive processing
            sleep(50);
        }
    }
}