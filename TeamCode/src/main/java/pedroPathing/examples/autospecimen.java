package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Auto Specimen", group = "Examples")
public class autospecimen extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;
    private Telemetry telemetryA;


    private final Pose startPose = new Pose(10, 48, Math.toRadians(0));
    private PathChain pathChain;

    // Build the PathChain
    private void buildPathChain() {
        pathChain = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(10.000, 48.000, Point.CARTESIAN),
                                new Point(23.800, 48.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Point(23.800, 48.000, Point.CARTESIAN),
                                new Point(45.531, 36.599, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Point(45.531, 36.599, Point.CARTESIAN),
                                new Point(72.109, 24.000, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Point(72.109, 24.000, Point.CARTESIAN),
                                new Point(20.000, 24.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    // Update the robot's movement using path states
    private void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Start following the path chain
                follower.followPath(pathChain, true);
                setPathState(1);
                break;
            case 1:
                // Check if the robot has completed the path
                if (!follower.isBusy()) {
                    telemetry.addData("Path", "Completed");
                    setPathState(-1); // Stop pathing logic
                }
                break;
        }
    }

    // Set the current path state and reset the path timer
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPathChain(); // Generate the paths
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();

    }

    @Override
    public void start() {
        setPathState(0); // Start the pathing process
    }

    @Override
    public void loop() {
        // Update follower and path logic

        follower.update();
        follower.telemetryDebug(telemetryA);

        autonomousPathUpdate();

        // Feedback to the Driver Hub
        telemetry.addData("path state", pathState);

        telemetry.update();
    }
}
