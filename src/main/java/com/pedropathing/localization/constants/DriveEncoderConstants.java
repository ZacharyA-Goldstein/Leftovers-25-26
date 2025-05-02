package com.pedropathing.localization.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Encoder;

@Config
public class DriveEncoderConstants {

    public static double forwardTicksToInches = 1;
    public static double strafeTicksToInches = 1;
    public static double turnTicksToInches = 1;

    public static double robot_Width = 1;
    public static double robot_Length = 1;

    public static double leftFrontEncoderDirection = Encoder.REVERSE;
    public static double rightFrontEncoderDirection = Encoder.FORWARD;
    public static double leftRearEncoderDirection = Encoder.REVERSE;
    public static double rightRearEncoderDirection = Encoder.FORWARD;

    public static String leftFrontMotorName = "leftFront";
    public static String leftRearMotorName = "leftRear";
    public static String rightFrontMotorName = "rightFront";
    public static String rightRearMotorName = "rightRear";

    public DriveEncoderConstants forwardTicksToInches(double forwardTicksToInches) {
        DriveEncoderConstants.forwardTicksToInches = forwardTicksToInches;
        return this;
    }

    public DriveEncoderConstants strafeTicksToInches(double strafeTicksToInches) {
        DriveEncoderConstants.strafeTicksToInches = strafeTicksToInches;
        return this;
    }

    public DriveEncoderConstants turnTicksToInches(double turnTicksToInches) {
        DriveEncoderConstants.turnTicksToInches = turnTicksToInches;
        return this;
    }

    public DriveEncoderConstants robotWidth(double robot_Width) {
        DriveEncoderConstants.robot_Width = robot_Width;
        return this;
    }

    public DriveEncoderConstants robotLength(double robot_Length) {
        DriveEncoderConstants.robot_Length = robot_Length;
        return this;
    }

    public DriveEncoderConstants leftFrontEncoderDirection(double leftFrontEncoderDirection) {
        DriveEncoderConstants.leftFrontEncoderDirection = leftFrontEncoderDirection;
        return this;
    }

    public DriveEncoderConstants rightFrontEncoderDirection(double rightFrontEncoderDirection) {
        DriveEncoderConstants.rightFrontEncoderDirection = rightFrontEncoderDirection;
        return this;
    }

    public DriveEncoderConstants leftRearEncoderDirection(double leftRearEncoderDirection) {
        DriveEncoderConstants.leftRearEncoderDirection = leftRearEncoderDirection;
        return this;
    }

    public DriveEncoderConstants rightRearEncoderDirection(double rightRearEncoderDirection) {
        DriveEncoderConstants.rightRearEncoderDirection = rightRearEncoderDirection;
        return this;
    }

    public DriveEncoderConstants leftFrontMotorName(String leftFrontMotorName) {
        DriveEncoderConstants.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public DriveEncoderConstants leftRearMotorName(String leftRearMotorName) {
        DriveEncoderConstants.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public DriveEncoderConstants rightFrontMotorName(String rightFrontMotorName) {
        DriveEncoderConstants.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public DriveEncoderConstants rightRearMotorName(String rightRearMotorName) {
        DriveEncoderConstants.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public void defaults() {
        forwardTicksToInches = 1;
        strafeTicksToInches = 1;
        turnTicksToInches = 1;

        robot_Width = 1;
        robot_Length = 1;

        leftFrontEncoderDirection = Encoder.REVERSE;
        rightFrontEncoderDirection = Encoder.FORWARD;
        leftRearEncoderDirection = Encoder.REVERSE;
        rightRearEncoderDirection = Encoder.FORWARD;

        leftFrontMotorName = "leftFront";
        leftRearMotorName = "leftRear";
        rightFrontMotorName = "rightFront";
        rightRearMotorName = "rightRear";
    }
}