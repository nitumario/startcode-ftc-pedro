package pedroPathing.startcode;

import android.graphics.Bitmap;
import org.opencv.android.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.opencv.core.Mat;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "Color Detection TeleOp", group = "TeleOp")
public class ColorDetectionTeleOp extends OpMode {
    private OpenCvCamera camera;
    private DetectionR detectionPipeline;
    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize the pipeline
        detectionPipeline = new DetectionR();
        camera.setPipeline(detectionPipeline);

        // Open the camera device and start streaming
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
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

    @Override
    public void loop() {
        // Convert the latest Mat to a Bitmap
        Bitmap bitmap = matToBitmap(detectionPipeline.getLatestMat());

        if (bitmap != null) {
            // Send the converted Bitmap to the FTC Dashboard
            dashboard.sendImage(bitmap);
        } else {
            telemetry.addData("Error", "Failed to convert Mat to Bitmap");
        }

        // Display information on the Driver Station telemetry
        telemetry.addData("Frames Processed", detectionPipeline.framesTotal());
        telemetry.addData("Object Width (px)", detectionPipeline.width);
        telemetry.addData("Distance (in)", detectionPipeline.getDistance(detectionPipeline.width));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (camera != null) {
            camera.stopStreaming();
            camera.closeCameraDevice();
        }
    }

    /**
     * Utility method to convert an OpenCV Mat to an Android Bitmap.
     * @param mat The Mat to be converted.
     * @return A Bitmap representation of the Mat, or null if the Mat is invalid.
     */
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
