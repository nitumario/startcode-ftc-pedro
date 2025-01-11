package pedroPathing.startcode;

import android.graphics.Bitmap;
import org.opencv.android.Utils;
import com.acmerobotics.dashboard.FtcDashboard;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "Robot-Centric Sample Detection Teleop", group = "Startcode")
public class RobotCentricSampleDetectingTeleop extends OpMode {

    /**
     * Movement/localization declaration
     **/
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0);

    /**
     * Movement/localization declaration
     **/

    private OpenCvCamera camera;
    private DetectionR detectionPipeline;
    private FtcDashboard dashboard;

    /**
     * This method is call once when init is played, it initializes the follower
     **/
    @Override
    public void init() {

        /** Movement/localization init **/

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        /** Movement/localization init **/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detectionPipeline = new DetectionR();
        camera.setPipeline(detectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                // Start camera streaming
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // Handle the error if the camera fails to open
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
        // Initialize the FTC Dashboard
        dashboard = FtcDashboard.getInstance();
    }

    /**
     * This method is called continuously after Init while waiting to be started.
     **/
    @Override
    public void init_loop() {
    }

    /**
     * This method is called once at the start of the OpMode.
     **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    /**
     * This is the main loop of the opmode and runs continuously after play
     **/
    @Override
    public void loop() {

        // Default robot movement based on gamepad input
        if (!gamepad1.square) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        } else {
            // Centering logic when square button is pressed
            double centerX = detectionPipeline.getCenterX();
            double frameCenterX = 1280 / 2.0; // Assuming 1280 width resolution
            double error = centerX - frameCenterX; // Error in pixels

            // Adjust robot's left/right movement to reduce the error
            double kP = 0.005; // Proportional control constant
            double adjustment = kP * error; // Proportional adjustment

            follower.setTeleOpMovementVectors(0, -adjustment, 0, true);

            telemetry.addData("Centering Mode", "Active");
            telemetry.addData("Center Error", error);
        }

        follower.update();

        // Display telemetry data
        double centerX = detectionPipeline.getCenterX();
        double centerY = detectionPipeline.getCenterY();
        telemetry.addData("Object Center (X, Y)", String.format("(%.2f, %.2f)", centerX, centerY));
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Frames Processed", detectionPipeline.framesTotal());
        telemetry.addData("Object Width (px)", detectionPipeline.getWidth());
        telemetry.addData("Distance (in)", detectionPipeline.getDistance());

        telemetry.update();

        // Convert the latest Mat to a Bitmap
        Bitmap bitmap = matToBitmap(detectionPipeline.getLatestMat());

        if (bitmap != null) {
            // Send the converted Bitmap to the FTC Dashboard
            dashboard.sendImage(bitmap);
        } else {
            telemetry.addData("Error", "Failed to convert Mat to Bitmap");
        }

    }

    /**
     * We do not use this because everything automatically should disable
     **/
    @Override
    public void stop() {
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    private Bitmap matToBitmap(Mat mat) {
        if (mat == null || mat.empty() || mat.cols() <= 0 || mat.rows() <= 0) {
            telemetry.addData("Error", "Invalid Mat object. Width and height must be > 0.");
            telemetry.update();
            return null;
        }

        Bitmap bitmap = Bitmap.createBitmap(mat.cols(), mat.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(mat, bitmap);
        return bitmap;

    }
}
