package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;


@Autonomous
@Disabled
public class DucksAutonomous5 extends LinearOpMode {
    DucksProgrammingBoard1_4 board = new DucksProgrammingBoard1_4();
    double forwardconstant = Math.PI * 75 * 523.875/457.2 * 514.35/457.2 * 417.5125/457.2 * 665/635 * 641/635 * 638/635;
    double rotationConstant = 360*((75 * Math.PI)/(533.4 * Math.PI)) * 92/90 *90.7/90 * 88.8103/90;
    double sideconstant = Math.PI * 75 * 534/508 * 510/508 * 512/508;
    double armconstant = 360 * 30/125 * 30/125 ;
    int state;



    @Override
    public void runOpMode() {


        board.init(hardwareMap);
        telemetry.addData("rotations init", board.getMotorRotations());
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (state == 0) {
                board.setClaw_1Active();
                board.setClaw_2Active();
                board.setClawRotation(0);
                state = 1;
            }
            if (state == 1) {
                sleep(2000);
                state = 2;
            }
            if (state == 2){
                MoveArmDegrees(15,0.3);
                state = 3;
            }
            if (state == 3){
                leftPlacement();
                state = 4;
            }
        }
    }
    public void MoveForwardDistance(double distance, double forwardSpeed){
        telemetry.addData("rotations forward", board.getMotorRotations());
        telemetry.update();
        double initialWheelRotation = board.getMotorRotations();
        double millimeters = (forwardconstant * (board.getMotorRotations()-initialWheelRotation));
        telemetry.addData("millimeters", millimeters);
        telemetry.update();
        if (distance > 0) {
            while (millimeters < distance) {
                //elevatorheight();
                board.setForwardSpeed(forwardSpeed);
                millimeters = (forwardconstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("millimeter slow", millimeters);
                telemetry.update();
            }
        }
        else if (distance < 0) {
            while (millimeters > distance) {
                //elevatorheight();
                board.setForwardSpeed(-forwardSpeed);
                millimeters = (forwardconstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("millimeter slow", millimeters);
                telemetry.update();
            }
        }
        board.setForwardSpeed(0);
    }
    public void MoveRotateDegrees(double degrees, double rotateSpeed) {
        double initialWheelRotation = board.getMotorRotations();
        double mm = (rotationConstant * (board.getMotorRotations()-initialWheelRotation));
        if (degrees > 0) {
            while (mm < degrees) {
                //elevatorheight();
                board.setRotateSpeed(rotateSpeed);
                mm = (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("rotations (mm?)=", mm);
                telemetry.update();
            }
        }
        else if (degrees < 0) {
            while (mm > degrees) {
                //elevatorheight();
                board.setRotateSpeed(-rotateSpeed);
                mm = (rotationConstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("rotations (mm?)=", mm);
                telemetry.update();
            }
        }
        board.setRotateSpeed(0);
    }
    public void MoveSidewaysDistance(double distance) {
        double initialWheelRotation = board.getMotorRotations();
        double xx = Math.abs(sideconstant * (board.getMotorRotations() - initialWheelRotation));
        if (distance > 0) {
            while (xx < distance) {
                board.setSideMotorSpeed(.2);
                xx = (sideconstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("millimeter slow", xx);
                telemetry.update();
            }
        }
        else if (distance < 0) {
            while (xx > distance) {
                board.setSideMotorSpeed(-.2);
                xx = (sideconstant * (board.getMotorRotations() - initialWheelRotation));
                telemetry.addData("millimeter slow", xx);
                telemetry.update();
            }
        }
        board.setForwardSpeed(0);
    }
    public void MoveArmDegrees(double degrees, double Speed){
        double initialArmRotation = board.getArmMotorRotations();
        double angleDeg = Math.abs(armconstant * (board.getArmMotorRotations() - initialArmRotation));
        if (degrees > 0) {
            while (angleDeg < degrees) {
                //elevatorheight();
                board.setArmSpeed(Speed);
                angleDeg = (armconstant * (board.getArmMotorRotations() - initialArmRotation));
                telemetry.addData("arm angle (degrees) ", angleDeg);
                telemetry.update();
            }
        }
        else if (degrees < 0) {
            while (angleDeg > degrees) {
                //elevatorheight();
                board.setArmSpeed(-Speed);
                angleDeg = (armconstant * (board.getArmMotorRotations() - initialArmRotation));
                telemetry.addData("arm angle (degrees) ", angleDeg);
                telemetry.update();
            }
        }
        board.setArmSpeed(0);
        telemetry.addData("degrees?", angleDeg);
        telemetry.update();
    }

    public void leftPlacement () {
        int state = 0;

        if (state == 0) {
            MoveSidewaysDistance(-90);
            state =1;
        }
        if (state == 1) {
            sleep(1000);
            state = 2;
        }
        if (state == 2) {
            MoveForwardDistance(350, 0.4);
            state = 3;
        }
        if (state == 3) {
            sleep(1000);
            state = 4;
        }
        if (state == 4){
            MoveArmDegrees(-13, 0.3);
            state = 5;
        }
        if (state == 5){
            sleep(1000);
            state = 6;
        }
        if (state == 6) {
            board.setClaw_1Inactive();
            state = 7;
        }
        if (state == 7) {
            sleep(1000);
            state = 8;
        }
        if (state == 8){
            MoveArmDegrees(15, 0.3);
            state = 9;
        }
        if (state == 9) {
            MoveForwardDistance(-320, 0.4);
            state = 10;
        }

    }
    public void centerPlacement () {
        int state = 0;

        if (state == 0) {
            MoveSidewaysDistance(-160);
            state = 1;
        }
        if (state == 1){
            MoveForwardDistance(485, 0.4);
            state = 2;
        }
        if (state == 2) {
            sleep(1000);
            state = 3;
        }
        if (state == 3) {
            MoveRotateDegrees(15, 0.1);
            state = 4;
        }
        if (state == 4) {
            sleep(1000);
            state = 5;
        }
        if (state == 5){
            MoveForwardDistance(120, 0.4);
            state = 6;
        }
        if (state == 6){
            sleep(1000);
            state = 7;
        }
        if (state == 7) {
            MoveArmDegrees(-13, 0.3);
            state = 8;
        }
        if (state == 8) {
            sleep(1000);
            state = 9;
        }
        if (state == 9){
            board.setClaw_1Inactive();
            state = 10;
        }
        if (state == 10) {
            sleep(1000);
            state = 11;
        }
        if (state == 11){
            MoveArmDegrees(15, 0.3);
            state = 12;
        }
        if (state == 12){
            MoveForwardDistance(-200, 0.4);
            state = 13;
        }
        if (state == 13){
            MoveRotateDegrees(-130, 0.1);
            state = 14;
        }

    }
}

