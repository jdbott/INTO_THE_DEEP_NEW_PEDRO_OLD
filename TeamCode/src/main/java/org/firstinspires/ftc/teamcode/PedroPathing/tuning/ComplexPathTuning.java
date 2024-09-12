package org.firstinspires.ftc.teamcode.PedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.PedroPathing.util.Timer;

@Config
@Autonomous(name = "Complex Path Tuning", group = "Autonomous Pathing Tuning")
public class ComplexPathTuning extends OpMode {

    public final Point startPose = new Point(15, -63, Point.CARTESIAN);
    public final Point spikeMarkMiddlePose = new Point(15, -37.2, Point.CARTESIAN);
    public final Point spikeMarkPose = new Point(9.6, -34.8, Point.CARTESIAN);
    public final Point backdropPose = new Point(49.5, -26.2, Point.CARTESIAN);
    public final Point toStartMiddlePose = new Point(55, -60, Point.CARTESIAN);

    private Telemetry telemetryA;
    private Follower follower;
    private Timer pathTimer;
    private Path toSpikeMark, toBackdrop, backToStart;
    private int pathState;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(15, -63, Math.toRadians(90)));
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        buildPaths();
        pathState = 0;
        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void start() {
        follower.followPath(toSpikeMark, true);
        setPathState(0);
        resetRuntime();
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
        follower.telemetryDebug(telemetryA);
        autoPathUpdate();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    setPathState(1);
                }
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(toBackdrop, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(backToStart, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    follower.followPath(toSpikeMark, true);
                    setPathState(0);
                }
                break;

            default:
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        toSpikeMark = new Path(new BezierCurve(
                startPose,
                spikeMarkMiddlePose,
                spikeMarkPose));
        toSpikeMark.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(137.6));

        toBackdrop = new Path(new BezierLine(
                spikeMarkPose,
                backdropPose));
        toBackdrop.setLinearHeadingInterpolation(Math.toRadians(137.6), Math.toRadians(180));

        backToStart = new Path(new BezierCurve(
                backdropPose,
                toStartMiddlePose,
                startPose));
        backToStart.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90));
    }
}