package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;


public class AutonomousLibrary {

    RobotHardware rh;
    CommonLibrary cl;

    public void driveAuto(LinearOpMode caller, double angle, double distace) {

        resetMotorEncoders(caller);

        double driveAngle = angle * Math.PI / 180;

        double xInput = Math.cos(driveAngle);
        double yInput = Math.sin(driveAngle);
        double flPower = Range.clip((yInput - xInput), -1, 1);
        double frPower = Range.clip((yInput + xInput), -1, 1);
        double blPower = Range.clip((yInput - xInput), -1, 1);
        double brPower = Range.clip((yInput + xInput), -1, 1);

        double frontLeftMotorRatio = 1 / Math.sin(driveAngle);
        double frontRightMotorRatio = 1 / Math.sin(driveAngle);
        double rearLeftMotorRatio = 1 / Math.sin(driveAngle);
        double rearRightMotorRatio = 1 / Math.sin(driveAngle);

        //Put target stuff here, don't run until that happens. Delete comment when done.

        rh.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void resetMotorEncoders(LinearOpMode caller){

        rh.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void encoderTicksToInchesTest (Telemetry telemetry, LinearOpMode caller) {

        long startTime = System.currentTimeMillis();
        while(caller.opModeIsActive() && !caller.isStopRequested()) {
            long currentTime = System.currentTimeMillis();
            double timeChange = (currentTime - startTime);
            if (timeChange <= 1000) {
                rh.frontRightMotor.setPower(1);
                rh.backLeftMotor.setPower(1);
                rh.backRightMotor.setPower(1);

                int currentEncoderTicks = rh.frontRightMotor.getCurrentPosition();
                telemetry.addData("Current ticks", currentEncoderTicks);
                telemetry.update();
            }
            else {break;}
        }
    }

}
