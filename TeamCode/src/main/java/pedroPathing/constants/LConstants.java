package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        ThreeWheelIMUConstants.forwardTicksToInches = 0.002950229710734186; //002950229710734186 028806062646941907 0029881807382524193 002990257862168864 002992400922390326
        ThreeWheelIMUConstants.strafeTicksToInches = 0.0029027835101480635; // 0.0029027835101480635 0030074393770277608 0029881807382524193 002990257862168864 0030271166968095465 0030279074226234456
        ThreeWheelIMUConstants.turnTicksToInches = 0.0029; // 0.0031 0.0048
        ThreeWheelIMUConstants.leftY = 4.015748;
        ThreeWheelIMUConstants.rightY = -4.015748;
        ThreeWheelIMUConstants.strafeX = -6.2992125;
        ThreeWheelIMUConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelIMUConstants.rightEncoder_HardwareMapName = "ododreapta";
        ThreeWheelIMUConstants.strafeEncoder_HardwareMapName = "rightFront";
        ThreeWheelIMUConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.rightEncoderDirection = Encoder.FORWARD;
        ThreeWheelIMUConstants.strafeEncoderDirection = Encoder.REVERSE;
        ThreeWheelIMUConstants.IMU_HardwareMapName = "imu";
        ThreeWheelIMUConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
    }
}




