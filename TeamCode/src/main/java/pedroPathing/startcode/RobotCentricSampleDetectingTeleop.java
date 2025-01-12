package pedroPathing.startcode;

import static java.lang.Math.abs;

import android.graphics.Bitmap;

import org.opencv.android.Utils;
import com.acmerobotics.dashboard.FtcDashboard;
import org.opencv.core.Mat;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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
     * Hardware declaration
     **/
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;

    public DcMotor motorVert1;
    public DcMotor motorVert2;
    public DcMotor motorOriz;

    public Servo claw;
    public CRServo clawRotate;
    public Servo specimen;

    private boolean claw_state = false;
    private boolean specimen_state = false;


    // Add PID constants
    private double kP = 0.005; // Proportional gain
    private double kI = 0.0001; // Integral gain
    private double kD = 0.0005; // Derivative gain

    // PID variables
    private double targetPosition = 0; // Desired position in encoder ticks
    private double integralSum = 0;    // Integral accumulator
    private double lastError = 0;      // Previous error for derivative calculation
    private long lastTime = 0;         // Previous timestamp for time-based calculations

    /**
     * This method is called once when init is played, it initializes the follower
     **/
    @Override
    public void init() {

        /** Movement/localization init **/

        /** Hardware init **/
        motorVert1 = hardwareMap.dcMotor.get("motorvert1");
        motorVert2 = hardwareMap.dcMotor.get("motorvert2");
        motorOriz = hardwareMap.dcMotor.get("motororiz");

        motorVert1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVert2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVert1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorVert2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorVert2.setDirection(DcMotorSimple.Direction.REVERSE);

        claw = hardwareMap.servo.get("claw");
        clawRotate = hardwareMap.crservo.get("clawRotate");
        specimen = hardwareMap.servo.get("specimen");

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        // Camera initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detectionPipeline = new DetectionR();
        camera.setPipeline(detectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        dashboard = FtcDashboard.getInstance();

        // Set the initial target position
        targetPosition = motorVert1.getCurrentPosition();
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {

        // Default robot movement
        if (!gamepad1.square) {
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        } else {
            double centerX = detectionPipeline.getCenterX();
            double frameCenterX = 1280 / 2.0;
            double error = centerX - frameCenterX;
            double kP = 0.00025;
            double adjustment = kP * error;
            follower.setTeleOpMovementVectors(0, -adjustment, 0, true);

            telemetry.addData("Centering Mode", "Active");
            telemetry.addData("Center Error", error);
        }

        follower.update();

        // Manual control for vertical arm
        double vert_gamepad = gamepad2.left_stick_y;
        double manualPower = vert_gamepad * 4;

        if (Math.abs(manualPower) > 0.1) {
            targetPosition += manualPower * 10;
        }

        // PID control for holding position
        double currentPosition = motorVert1.getCurrentPosition();
        double error = targetPosition - currentPosition;

        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastTime) / 1000.0;

        double proportional = kP * error;
        integralSum += error * deltaTime;
        double integral = kI * integralSum;
        double derivative = kD * (error - lastError) / deltaTime;
        double pidOutput = proportional + integral + derivative;

        motorVert1.setPower(pidOutput);
        motorVert2.setPower(pidOutput);

        telemetry.addData("Target Position", targetPosition);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Error", error);
        telemetry.addData("PID Output", pidOutput);
        telemetry.addData("servo claw", claw.getPosition());
        telemetry.addData("servo clawRotate", clawRotate.getPower());
        telemetry.addData("servo specimen", specimen.getPosition());

        lastError = error;
        lastTime = currentTime;

        // Horizontal arm control
        motorOriz.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double powerOriz = gamepad2.left_stick_x;
        motorOriz.setPower(powerOriz);

        // Specimen control
        if (gamepad2.left_bumper && !specimen_state) {

            specimen.setPosition(0.4);
            specimen_state = true;
        } else if (gamepad2.left_bumper && specimen_state){
            specimen.setPosition(0.0);
            specimen_state = false;
        }




        // Claw control
        if (gamepad2.right_bumper && !claw_state) {
            claw.setPosition(0.4);
            claw_state = true;
        } else if (gamepad2.right_bumper && claw_state) {
            claw.setPosition(0.1);
            claw_state = false;
        }

        double powerRotateClaw = gamepad2.right_stick_x;
        clawRotate.setPower(powerRotateClaw/2);

        // Display detection data
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

        Bitmap bitmap = matToBitmap(detectionPipeline.getLatestMat());
        if (bitmap != null) {
            dashboard.sendImage(bitmap);
        } else {
            telemetry.addData("Error", "Failed to convert Mat to Bitmap");
        }
    }

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
