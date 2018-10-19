package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLibrary;

public class RobotHardware {

        HardwareMap hardwareMap;
        CommonLibrary cl;
        public DcMotor frontLeftMotor;
        public DcMotor backRightMotor;
        public DcMotor frontRightMotor;
        public DcMotor backLeftMotor;

        public void init(HardwareMap hwMap, Telemetry telemetry){

            hardwareMap = hwMap;
            frontRightMotor = hardwareMap.dcMotor.get("front right motor");
            if(frontRightMotor == hardwareMap.dcMotor.get("front right motor")){telemetry.addData("Motor isn't null?", frontRightMotor.getPortNumber());}
            frontLeftMotor = hardwareMap.dcMotor.get("front left motor");
            backLeftMotor = hardwareMap.dcMotor.get("back left motor");
            backRightMotor = hardwareMap.dcMotor.get("back right motor");

            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);

            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        }
}
