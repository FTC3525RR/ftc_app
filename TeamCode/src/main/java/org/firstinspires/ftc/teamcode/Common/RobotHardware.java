package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Common.CommonLibrary;

public class RobotHardware {
        CommonLibrary cl;

        public DcMotor frontLeftMotor;
        public DcMotor frontRightMotor;
        public DcMotor backLeftMotor;
        public DcMotor backRightMotor;

        HardwareMap hardwareMap;

        public void init(HardwareMap hwMap){

            hardwareMap = hwMap;
            frontLeftMotor = hardwareMap.dcMotor.get("front left motor");
            frontRightMotor = hardwareMap.dcMotor.get("front right motor");
            backLeftMotor = hardwareMap.dcMotor.get("back left motor");

        }
}
