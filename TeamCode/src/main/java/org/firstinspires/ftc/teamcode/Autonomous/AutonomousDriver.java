package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Common.RobotHardware;

@Autonomous (name = "Auto Driver")

public class AutonomousDriver extends LinearOpMode {
    RobotHardware rh;
//    Orientation angles;
//    Orientation startAngles;

    public void runOpMode(){

        telemetry.addData("Robot", "0% initialised");
        telemetry.update();
        rh = new RobotHardware();
        rh.init(hardwareMap, telemetry);
        telemetry.addData("Robot", "42% initialised");
        telemetry.update();
        rh.initGyro(this);
        telemetry.addData("Robot", "Ready to start");
        telemetry.update();
        //inits(telemetry, hardwareMap);
        waitForStart();

        runMethod();

        while(opModeIsActive()){
            resetlift();
            liftDown(telemetry);
            sideDrive();
            //turnRelativeToStart(45,telemetry);
            dropMarker();
            break;
        }

    }

    public void inits(Telemetry telemetry, HardwareMap hardwareMap){
        telemetry.addData("Robot", "Starting init");
        telemetry.update();
        while (!opModeIsActive()){
            if (isStopRequested()){break;}
            rh.initAuto(hardwareMap, telemetry, this);
            if (isStopRequested()){break;}
            telemetry.addData("Init", "50%");
            telemetry.update();
            rh.initGyro(this);
            if (isStopRequested()){break;}
            telemetry.addData("Robot", "Ready to start");
            telemetry.update();
            break;
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

        while (rh.frontRightMotor.getCurrentPosition() <= 1000){
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

        rh.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rh.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void resetMotorEncoders(){

        rh.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rh.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void runWithoutEncoders(){

        rh.frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rh.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void runWithEncoder(){
        rh.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rh.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rh.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rh.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void liftUp(){
        rh.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(rh.liftMotor.getCurrentPosition() < 20500) {

            rh.liftMotor.setPower(1);
            telemetry.addData("Lift Ticks", rh.liftMotor.getCurrentPosition());
            telemetry.update();
            if(isStopRequested()){
                break;
            }
        }

        rh.liftMotor.setPower(0);
    }

    public void liftDown(Telemetry telemetry){

        rh.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(rh.liftMotor.getCurrentPosition() < 21700) {

            rh.liftMotor.setPower(1);
            telemetry.addData("Lift Ticks", rh.liftMotor.getCurrentPosition());
            telemetry.update();
            if(isStopRequested()){
                break;
            }
        }
        rh.liftMotor.setPower(0);
        while(rh.frontRightMotor.getCurrentPosition() < 1000){
            rh.frontRightMotor.setPower(1);
            rh.frontLeftMotor.setPower(-1);
            rh.backRightMotor.setPower(1);
            rh.backLeftMotor.setPower(-1);
        }
        rh.frontRightMotor.setPower(0);
        rh.frontLeftMotor.setPower(0);
        rh.backRightMotor.setPower(0);
        rh.backLeftMotor.setPower(0);

    }

    public void resetlift(){
        while (rh.liftMotorTouchSensor.getState() == true){
            rh.liftMotor.setPower(-1);
            if (!rh.liftMotorTouchSensor.getState() == true){
                rh.liftMotor.setPower(0);
                break;
            }
        }
        telemetry.addData("lift motor state", rh.liftMotorTouchSensor.getState());
        telemetry.update();
        rh.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void turnOnLift(){

        rh.liftMotor.setPower(1);
    }
    public void dropMarker(){
        rh.markerServo.setPosition(1);
    }
    public void sideDrive(){
        double startTicks = rh.frontRightMotor.getCurrentPosition();
        double currentTicks = rh.frontRightMotor.getCurrentPosition();;
        while (currentTicks < startTicks + 6000) {
            rh.frontRightMotor.setPower(1);
            rh.frontLeftMotor.setPower(-1);
            rh.backRightMotor.setPower(-1);
            rh.backLeftMotor.setPower(1);
            currentTicks = rh.frontRightMotor.getCurrentPosition();;
        }
        rh.frontRightMotor.setPower(0);
        rh.frontLeftMotor.setPower(0);
        rh.backRightMotor.setPower(0);
        rh.backLeftMotor.setPower(0);
    }

}
