package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

        HardwareMap hardwareMap;
        CommonLibrary cl;
        public DcMotor frontLeftMotor = null;
        public DcMotor backRightMotor = null;
        public DcMotor frontRightMotor = null;
        public DcMotor backLeftMotor = null;
        public DcMotor liftMotor = null;
        public DigitalChannel liftMotorTouchSensor;


        public void init(HardwareMap hwMap, Telemetry telemetry){

            hardwareMap = hwMap;
            frontRightMotor = hardwareMap.dcMotor.get("front right motor");
            if(frontRightMotor == hardwareMap.dcMotor.get("front right motor")) {
                telemetry.addData("Motor isn't null?", frontRightMotor.getPortNumber());
            }
            frontLeftMotor = hardwareMap.dcMotor.get("front left motor");
            backLeftMotor = hardwareMap.dcMotor.get("back left motor");
            backRightMotor = hardwareMap.dcMotor.get("back right motor");
            liftMotor = hardwareMap.dcMotor.get("lift motor");
            liftMotorTouchSensor = hardwareMap.digitalChannel.get("lift touch sensor");

            liftMotorTouchSensor.setMode(DigitalChannel.Mode.INPUT);

            telemetry.addData("","Ready to start");
            telemetry.update();

        }

        public void runMethod(){



            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            liftMotor.setPower(0);

            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
}
