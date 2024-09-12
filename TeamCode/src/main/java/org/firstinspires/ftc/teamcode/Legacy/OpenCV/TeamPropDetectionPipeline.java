package org.firstinspires.ftc.teamcode.Legacy.OpenCV;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * TeamPropDetectionPipeline.java
 *
 * This class extends OpenCvPipeline and processes each frame from the webcam.
 * It segments the image into three equal horizontal sections and calculates
 * the red and blue color ratios for each section. The sections with the highest
 * red and blue ratios are highlighted.
 */
public class TeamPropDetectionPipeline extends OpenCvPipeline {
    // Declare variables to store the current frame and the sections with the most red and blue colors
    private Mat currentFrame = new Mat();
    private int mostRedSection = -1;
    private int mostBlueSection = -1;

    /**
     * Processes each frame from the webcam. This method segments the image, calculates
     * color ratios, and highlights the sections with the most red and blue colors.
     *
     * @param input The input frame from the webcam.
     * @return The processed frame with highlighted sections.
     */
    @Override
    public Mat processFrame(Mat input) {
        // Copy the input frame to the currentFrame variable
        input.copyTo(currentFrame);

        // Get the dimensions of the image
        int height = currentFrame.rows();
        int width = currentFrame.cols();
        int sectionWidth = width / 3; // Width of each section
        int sectionHeight = sectionWidth; // Height of each section (assumed square)

        // Calculate the vertical offset to center the sections horizontally
        int yOffset = (height - sectionHeight) / 2;

        // Define rectangles for the three sections
        Rect section1 = new Rect(0, yOffset, sectionWidth, sectionHeight);
        Rect section2 = new Rect(sectionWidth, yOffset, sectionWidth, sectionHeight);
        Rect section3 = new Rect(2 * sectionWidth, yOffset, sectionWidth, sectionHeight);

        // Draw green rectangles around each section for visualization
        Imgproc.rectangle(input, section1, new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, section2, new Scalar(0, 255, 0), 2);
        Imgproc.rectangle(input, section3, new Scalar(0, 255, 0), 2);

        // Calculate the red, green, and blue ratios for each section
        double[] rgbRatio1 = calculateRGBRatio(currentFrame.submat(section1));
        double[] rgbRatio2 = calculateRGBRatio(currentFrame.submat(section2));
        double[] rgbRatio3 = calculateRGBRatio(currentFrame.submat(section3));

        // Determine the section with the highest red and blue ratios
        double[] redRatios = {rgbRatio1[0], rgbRatio2[0], rgbRatio3[0]};
        double[] blueRatios = {rgbRatio1[2], rgbRatio2[2], rgbRatio3[2]};
        mostRedSection = maxIndex(redRatios);
        mostBlueSection = maxIndex(blueRatios);

        // Highlight the sections with the most red or blue
        if (mostRedSection != -1) {
            highlightSection(input, mostRedSection, sectionWidth, sectionHeight, yOffset, new Scalar(255, 0, 0));
        }
        if (mostBlueSection != -1) {
            highlightSection(input, mostBlueSection, sectionWidth, sectionHeight, yOffset, new Scalar(0, 0, 255));
        }

        // Return the processed frame
        return input;
    }

    /**
     * Calculates the red, green, and blue ratios for a given image section.
     *
     * @param section The image section to calculate the color ratios for.
     * @return An array containing the red, green, and blue ratios.
     */
    private double[] calculateRGBRatio(Mat section) {
        // Initialize arrays to store the sum of RGB values and the total intensity
        double[] sum = {0, 0, 0};
        double totalIntensity = 0;

        // Iterate over each pixel in the section
        for (int row = 0; row < section.rows(); row++) {
            for (int col = 0; col < section.cols(); col++) {
                // Get the RGB values of the current pixel
                double[] rgb = section.get(row, col);
                // Add the RGB values to the sum
                sum[0] += rgb[0];
                sum[1] += rgb[1];
                sum[2] += rgb[2];
                // Add the total intensity of the pixel to the total intensity
                totalIntensity += rgb[0] + rgb[1] + rgb[2];
            }
        }

        // Calculate the ratios by dividing the sum of each color by the total intensity
        double[] ratios = {sum[0] / totalIntensity, sum[1] / totalIntensity, sum[2] / totalIntensity};
        // Return the calculated ratios
        return ratios;
    }

    /**
     * Finds the index of the maximum value in an array.
     *
     * @param values The array of values.
     * @return The index of the maximum value.
     */
    private int maxIndex(double[] values) {
        // Initialize variables to store the index and maximum value
        int index = -1;
        double max = -1;

        // Iterate over the array to find the maximum value
        for (int i = 0; i < values.length; i++) {
            if (values[i] > max) {
                max = values[i];
                index = i;
            }
        }

        // Return the index of the maximum value
        return index;
    }

    /**
     * Highlights a specified section with a given color.
     *
     * @param input The input frame to draw on.
     * @param sectionIndex The index of the section to highlight.
     * @param sectionWidth The width of each section.
     * @param sectionHeight The height of each section.
     * @param yOffset The vertical offset to center the sections horizontally.
     * @param color The color to use for highlighting the section.
     */
    private void highlightSection(Mat input, int sectionIndex, int sectionWidth, int sectionHeight, int yOffset, Scalar color) {
        // Calculate the x-coordinate of the section based on its index
        int x = sectionIndex * sectionWidth;
        // Draw a rectangle around the section with the specified color and thickness
        Imgproc.rectangle(input, new Rect(x, yOffset, sectionWidth, sectionHeight), color, 6);
    }

    /**
     * Returns the section with the most red color as a string.
     *
     * @return The section with the most red color ("left", "center", "right", or "none").
     */
    public String getMostRedSection() {
        return sectionToString(mostRedSection);
    }

    /**
     * Returns the section with the most blue color as a string.
     *
     * @return The section with the most blue color ("left", "center", "right", or "none").
     */
    public String getMostBlueSection() {
        return sectionToString(mostBlueSection);
    }

    /**
     * Converts a section index to a string representation.
     *
     * @param sectionIndex The index of the section.
     * @return The string representation of the section ("left", "center", "right", or "none").
     */
    private String sectionToString(int sectionIndex) {
        switch (sectionIndex) {
            case 0:
                return "left";
            case 1:
                return "center";
            case 2:
                return "right";
            default:
                return "none";
        }
    }
}