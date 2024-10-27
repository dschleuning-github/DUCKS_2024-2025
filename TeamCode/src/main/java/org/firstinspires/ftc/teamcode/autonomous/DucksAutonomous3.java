package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;

@Autonomous
@Disabled
public class DucksAutonomous3 extends LinearOpMode {
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
                //MoveArmDegrees(30, 0.1);
                //MoveForwardDistance(695, .2);
                //board.setClawRotation(-0.7);
                board.setClaw_1Active();
                telemetry.addData("end state 0", state);
                telemetry.update();
                state = 1;
            }
            else if (state ==1){
                sleep(1000);
                telemetry.addData("end state 1", state);
                telemetry.update();
                state = 2;
            }
            else if (state == 2){
                MoveArmDegrees(40,1);
                telemetry.addData("end state 2", state);
                telemetry.update();
                state =3;
            }
            else if (state == 3) {
                //sleep(1000);
                //claw open -- 0.65
                MoveForwardDistance(750,0.5);
                telemetry.addData("end state 3", state);
                telemetry.update();
                state = 4;
            }else if (state == 4) {
                //negative rotate is left, positive is right
                //MoveRotateDegrees(-89.5,.1);
                sleep(1000);
                telemetry.addData("end state 4", state);
                telemetry.update();
                state = 5;
            }
            else if (state == 5) {
                //sleep(1000);
                MoveRotateDegrees(-90,0.1);
                telemetry.addData("end state 6", state);
                telemetry.update();
                state = 6;
            }
            else if (state == 6){
                //MoveForwardDistance(1085, .2);
                //telemetry.addData("end state 5", state);
                board.setClawRotation(0.1);
                telemetry.update();
                state = 7;
            }
            else if (state == 7){
                sleep(1000);
                telemetry.addData("end state 6", state);
                telemetry.update();
                state = 8;
            }
            else if (state == 8){
                MoveForwardDistance(390, 0.5);
                telemetry.addData("end state 7", state);
                telemetry.update();
                state = 9;
            }
            else if (state == 9){
                board.setClaw_1Inactive();
                state = 10;
            }
            else if (state == 10){
                sleep(1000);
                telemetry.addData("end state 10", state);
                telemetry.update();
                state = 11;
            }
            else if (state == 11){
                MoveForwardDistance(-390, 0.5);
                telemetry.addData("end state 9", state);
                telemetry.update();
                state = 12;
            }
            else if (state == 12){
                MoveSidewaysDistance(-750);
                telemetry.addData("end state 8", state);
                telemetry.update();
                state = 13;
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
}
