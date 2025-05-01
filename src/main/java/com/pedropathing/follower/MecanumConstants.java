package com.pedropathing.follower;

import com.pedropathing.geometry.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class MecanumConstants extends DrivetrainConstants {
    public abstract double maxPower();
    public abstract Vector frontLeftVector();
    public abstract String leftFrontMotorName();
    public abstract String leftRearMotorName();
    public abstract String rightFrontMotorName();
    public abstract String rightRearMotorName();
    public abstract DcMotorSimple.Direction leftFrontMotorDirection();
    public abstract DcMotorSimple.Direction leftRearMotorDirection();
    public abstract DcMotorSimple.Direction rightFrontMotorDirection();
    public abstract DcMotorSimple.Direction rightRearMotorDirection();
}
