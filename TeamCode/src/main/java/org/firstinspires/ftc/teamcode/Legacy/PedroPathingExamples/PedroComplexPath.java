package org.firstinspires.ftc.teamcode.Legacy.PedroPathingExamples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.Flag;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.Lights;
import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;

@Autonomous(name = "Pedro Pathing Complex Path", group = "Pedro Auto")
public class PedroComplexPath extends OpMode {

    private Telemetry telemetryA;
    private Follower follower;
    private Lights lights;
    private Flag flag;
    private PathChain circle;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(15, -63, Math.toRadians(90)));

        lights = new Lights(hardwareMap);
        flag = new Flag(hardwareMap);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        buildPaths();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_GRADIENT);
        flag.flagDown();

        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void start() {
        follower.followPath(circle);
    }

    @Override
    public void loop() {
        try {
            follower.update();
        } catch (NullPointerException e) {
            telemetryA.addData("Error", "NullPointerException in follower.update()");
            telemetryA.addData("Message", e.getMessage());
            telemetryA.update();
        }
        telemetryA.update();
    }

    public void buildPaths() {
        circle = follower.pathBuilder()
                // Move to the starting point of the circle
                .addPath(new BezierLine(
                        new Point(60, -60, Point.CARTESIAN),
                        new Point(0, -60, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                // Create a circle using a cubic Bezier curve with a radius of 20 inches
                .addPath(new BezierCurve(
                        new Point(0, -60, Point.CARTESIAN),      // Start point
                        new Point(-20, -60, Point.CARTESIAN),    // First control point
                        new Point(-40, -40, Point.CARTESIAN),    // Second control point
                        new Point(-40, -20, Point.CARTESIAN),    // Third control point
                        new Point(-20, 0, Point.CARTESIAN),      // Fourth control point
                        new Point(0, 0, Point.CARTESIAN),        // Fifth control point
                        new Point(20, 0, Point.CARTESIAN),       // Sixth control point
                        new Point(40, -20, Point.CARTESIAN),     // Seventh control point
                        new Point(40, -40, Point.CARTESIAN),     // Eighth control point
                        new Point(20, -60, Point.CARTESIAN),     // Ninth control point
                        new Point(0, -60, Point.CARTESIAN)))     // End point
                // Move to the ending point
                .addPath(new BezierLine(
                        new Point(0, -60, Point.CARTESIAN),
                        new Point(-60, -60, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}