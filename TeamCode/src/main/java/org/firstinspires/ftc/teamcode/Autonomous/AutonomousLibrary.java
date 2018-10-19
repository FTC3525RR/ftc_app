package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLibrary;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;


public class AutonomousLibrary {

    RobotHardware robot = new RobotHardware();
    HardwareMap hardwareMap;
    CommonLibrary cl;


    public void driveAuto(double angle, double distance) {

                resetMotorEncoders();

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

        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void resetMotorEncoders(){

        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void encoderTicksToInchesTest (Telemetry telemetry, LinearOpMode caller) {

        //frontRightMotor = robot.frontRightMotor;
        long startTime = System.currentTimeMillis();
        while(caller.opModeIsActive() && !caller.isStopRequested()) {
            long currentTime = System.currentTimeMillis();
            double timeChange = (currentTime - startTime);
            if (timeChange <= 1000) {
                robot.frontRightMotor.getCurrentPosition();
                /*rh.frontRightMotor.setPower(1);
                rh.backLeftMotor.setPower(1);
                rh.backRightMotor.setPower(1);

                int tickTock;
                tickTock = rh.frontRightMotor.getCurrentPosition();
                telemetry.addData("Current ticks:", tickTock);*/
                telemetry.addData("The time has changed", timeChange);
                telemetry.update();
            }
            else {break;}
        }

        robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

    public void turnOnWheels(){

        robot.frontRightMotor.setPower(1);
        robot.backRightMotor.setPower(1);
        robot.backLeftMotor.setPower(1);
    }

}
