package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

@Autonomous (name = "Auto Driver test")

public class AutonomousDriverTest extends LinearOpMode {
    RobotHardware rh;

    public void runOpMode(){

        rh = new RobotHardware();
        rh.init(hardwareMap, telemetry);
//        AutonomousLibrary al = new AutonomousLibrary();
//        CommonLibrary cl = new CommonLibrary();
        waitForStart();

        runMethod();
//        turnOnWheels();

        while(opModeIsActive()){
            //turnOnWheels();
            encoderTicksToInchesTest(telemetry);
        }

    }

    public void runMethod(){

        rh.frontRightMotor.setPower(0);
        rh.frontLeftMotor.setPower(0);
        rh.backLeftMotor.setPower(0);
        rh.backRightMotor.setPower(0);
        //liftMotor.setPower(0);

//        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        //liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnOnWheels(){

        rh.frontRightMotor.setPower(1);
        rh.backRightMotor.setPower(1);
        rh.backLeftMotor.setPower(1);
        rh.frontLeftMotor.setPower(1);
    }

    public void encoderTicksToInchesTest (Telemetry telemetry) {

        while (opModeIsActive()){
            rh.frontRightMotor.setPower(1);
            rh.frontLeftMotor.setPower(1);
            rh.backRightMotor.setPower(1);
            rh.backLeftMotor.setPower(1);
            telemetry.addData("ticks", rh.backRightMotor.getCurrentPosition());
            telemetry.update();

            if (isStopRequested()) {break;}
        }

        rh.frontRightMotor.setPower(0);
        rh.frontLeftMotor.setPower(0);
        rh.backRightMotor.setPower(0);
        rh.backLeftMotor.setPower(0);
        telemetry.addData("done", "");
        telemetry.update();
    }
}
