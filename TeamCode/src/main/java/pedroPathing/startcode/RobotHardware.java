package pedroPathing.startcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


public class RobotHardware {
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

    public void initTop(HardwareMap hardwareMap) {
        motorVert1 = hardwareMap.dcMotor.get("motorvert1");
        motorVert2 = hardwareMap.dcMotor.get("motorvert2");
        motorOriz = hardwareMap.dcMotor.get("motororiz");

        motorVert1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVert2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOriz.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorVert1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorVert2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOriz.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        claw = hardwareMap.servo.get("claw");
        clawRotate = hardwareMap.crservo.get("clawRotate");
        specimen = hardwareMap.servo.get("specimen");

    }
    public void initChasis(HardwareMap hardwareMap){
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
    }
}
