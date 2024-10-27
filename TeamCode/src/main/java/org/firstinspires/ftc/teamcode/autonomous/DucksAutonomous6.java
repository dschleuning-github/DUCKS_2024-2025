package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.mechanisms.DucksProgrammingBoard1_4;
import org.firstinspires.ftc.teamcode.processors.FirstVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous
//@Disabled
public class DucksAutonomous6 extends OpMode {
    DucksProgrammingBoard1_4 board = new DucksProgrammingBoard1_4();
    double forwardconstant = Math.PI * 75 * 523.875 / 457.2 * 514.35 / 457.2 * 417.5125 / 457.2 * 665 / 635 * 641 / 635 * 638 / 635;
    double rotationConstant = 360 * ((75 * Math.PI) / (533.4 * Math.PI)) * 92 / 90 * 90.7 / 90 * 88.8103 / 90;
    double sideconstant = Math.PI * 75 * 534 / 508 * 510 / 508 * 512 / 508;
    double armconstant = 360 * 30 / 125 * 30 / 125;
    int state;
    int position;
    FirstVisionProcessor.Selected Position;

    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        visionProcessor = new FirstVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class,
                        "Webcam 1"),
                visionProcessor);
//        telemetry.addData("Identified", visionProcessor.getSelection());
//        telemetry.addData("test", visionProcessor.getData());
        telemetry.update();

        board.init(hardwareMap);
        telemetry.addData("rotations init", board.getMotorRotations());
        telemetry.update();
    }

    @Override
    public void init_loop(){
        telemetry.addData("Identified_loop", visionProcessor.getSelection());

    }

    @Override
    public void start(){

    }

    @Override
    public void loop(){
        if (state == 0) {
            board.setClaw_1Active();
            board.setClaw_2Active();
            board.setClawRotation(0);
            state = 1;
        }
        else if (state == 1) {
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state = 2;
        }
        else if (state == 2){
            MoveArmDegrees(15,0.5);
            state = 3;
        }
        else if (state == 3){
            board.setClawRotation(0.7);
            state = 4;
        }
        else if (state == 4){
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state = 5;
        }
        else if (state == 5){
            MoveSidewaysDistance(75);
            state = 6;
        }
        else if (state == 6){
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            state = 7;
        }
        else if (state == 7){
            Position = visionProcessor.getSelection();

            if (Position .equals(FirstVisionProcessor.Selected.MIDDLE)){
                position = 2;
                MoveSidewaysDistance(65);
                centerPlacement();
                state = 8;
            }
            else if (Position == FirstVisionProcessor.Selected.LEFT) {
                position = 1;
                MoveSidewaysDistance(-75);
                leftPlacement();
                state = 8;
            }
            else {
                position = 3;
                //MoveSidewaysDistance(-75);
                rightPlacement();
                state = 8;
            }
        }
        else if (state == 8){
            visionPortal.stopStreaming();
            state = 9;
        }
        telemetry.addData("state = ", state);
        telemetry.update();

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
                if (millimeters > distance* 3/4){
                    forwardSpeed = .2;
                }

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
                board.setArmSpeed(Speed);
                angleDeg = (armconstant * (board.getArmMotorRotations() - initialArmRotation));
                telemetry.addData("arm angle (degrees) ", angleDeg);
                telemetry.update();
            }
        }
        else if (degrees < 0) {
            while (angleDeg > degrees) {
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
        MoveSidewaysDistance(-90);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveForwardDistance(400, 0.4);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveArmDegrees(-13, 0.3);
        board.setClawRotation(0.0);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        board.setClaw_1Inactive();
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveArmDegrees(15, 0.3);
        MoveForwardDistance(-200, 0.4);
        MoveRotateDegrees(-90, 0.1);
    }
    public void centerPlacement () {
        MoveForwardDistance(1050, 0.4);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveForwardDistance(-175, 0.2);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        board.setClawRotation(0.0);
        MoveArmDegrees(-13, 0.3);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        board.setClaw_1Inactive();
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveArmDegrees(15, 0.3);
        board.setClawRotation(0.7);
        MoveForwardDistance(-35, 0.2);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveRotateDegrees(-91, 0.1);
        board.setSideMotorSpeed(0.0);
        board.setForwardSpeed(0);
    }
    public void rightPlacement() {
        //MoveSidewaysDistance(-90);
        MoveForwardDistance(800, 0.5);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveRotateDegrees(90, 0.1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveForwardDistance(380, 0.4);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveForwardDistance(-195, 0.4);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        board.setClawRotation(0.0);
        MoveArmDegrees(-13, 0.5);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        board.setClaw_1Inactive();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveArmDegrees(15, 0.5);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveForwardDistance(-50, 0.4);
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        MoveRotateDegrees(180, 0.1);
    }


}

