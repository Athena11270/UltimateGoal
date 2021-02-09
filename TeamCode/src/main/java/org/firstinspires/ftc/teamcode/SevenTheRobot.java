/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// test

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.nio.file.DirectoryIteratorException;

// This is not an OpMode.  It is a class that holds all the boring stuff

public class SevenTheRobot {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // create arrays for motors
    public DcMotor[] LeftMotors = new DcMotor[2];
    public DcMotor[] RightMotors = new DcMotor[2];
    public DcMotor[] AllMotors = new DcMotor[4];

    // Motors
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;

    // arm and stuff
    public DcMotor Arm = null;
    public Servo Claw = null;

    // Outtake/Launchers
    /*private DcMotor LR = null;
    private DcMotor LL = null;*/
    //in case of 1 motor for launcher
    private DcMotor OT = null;

    // Intake
    private DcMotor IT = null;

    // servo for reservoir
    public Servo BP = null;

    // just gonna define some variables for encoders real quick dont mind me
    static final double mmPerInch               = 25.4f;    // this is jus math tho
    static final double countsPerRevolution     = 383.6f;   // Gobilda Yellowjacket 435
    static final double wheelDiameterMM         = 100;      // For figuring circumference
    static final double WheelDiameterIn         = wheelDiameterMM * mmPerInch;
    static final double wheelCircumferenceIn    = WheelDiameterIn * Math.PI;
    static final double countsPerInch         = (countsPerRevolution / wheelCircumferenceIn);

    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;
    public SevenTheRobot(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    public void initialize() {
        // wheels
        FL = OpModeReference.hardwareMap.get(DcMotor.class, "FL");
        FR = OpModeReference.hardwareMap.get(DcMotor.class, "FR");
        BL = OpModeReference.hardwareMap.get(DcMotor.class, "BL");
        BR = OpModeReference.hardwareMap.get(DcMotor.class, "BR");

        BP = OpModeReference.hardwareMap.get(Servo.class, "BP");

        // arm and stuff electric boogaloo

        Arm = OpModeReference.hardwareMap.get(DcMotor.class, "Arm");
        Claw = OpModeReference.hardwareMap.get(Servo.class, "Claw");

        // outtake
        /*LR = OpModeReference.hardwareMap.get(DcMotor.class, "LR");
        LL = OpModeReference.hardwareMap.get(DcMotor.class, "LL");*/
        OT = OpModeReference.hardwareMap.get(DcMotor.class, "OT");
        IT = OpModeReference.hardwareMap.get(DcMotor.class, "IT");
        // motor arrays
        // left
        LeftMotors[0] = FL;
        LeftMotors[1] = BL;
        // right
        RightMotors[0] = FR;
        RightMotors[1] = BR;
        // all
        AllMotors[0] = FL;
        AllMotors[1] = FR;
        AllMotors[2] = BL;
        AllMotors[3] = BR;

        /*LR.setDirection(DcMotor.Direction.FORWARD);
        LL.setDirection(DcMotor.Direction.REVERSE);*/
        OT.setDirection(DcMotor.Direction.FORWARD);
        IT.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor l : LeftMotors)
            l.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor r : RightMotors)
            r.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotor m : AllMotors){
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setDirection(DcMotorSimple.Direction.FORWARD);
        Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Autonomous Methods:

    public void stopDriving (){
        for (DcMotor m : AllMotors)
            m.setPower(0);
    }

    public void openClaw () {
        Claw.setPosition(1);
    }

    public void closeClaw () {
        Claw.setPosition(0);
    }

    public void setArm (int pos) {
        Arm.setTargetPosition(pos);
        if (pos > Arm.getCurrentPosition()) {
            Arm.setPower(-0.5);
        }
        else {
            Arm.setPower(0.5);
        }
    }

    public void armUp () {
        setArm(620);
    }

    public void armDown () {
        setArm(1155);
    }


    public void drive (double inches, double speed){
        if (OpModeReference.opModeIsActive()) {
            //calculate how many ticks we need to go
            int targetTicks = (int) (-inches * countsPerInch);
            //reset ticks to 0
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            for (DcMotor m : AllMotors) {
                m.setTargetPosition(targetTicks);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            for (DcMotor m : AllMotors)
                m.setPower(speed/2);

            while (OpModeReference.opModeIsActive() && ((FL.isBusy() && FR.isBusy()) && (BL.isBusy() && BR.isBusy()))) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("fr ticks", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("fl ticks", FL.getCurrentPosition());
                OpModeReference.telemetry.update();
            }

            stopDriving();
            }

    }

    private void strafe (double inches, double speed){
        if (OpModeReference.opModeIsActive()) {
            int targetTicks = (int) (countsPerInch * inches * (-12 / 11));
            for (DcMotor m : AllMotors) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            FL.setTargetPosition(targetTicks);
            FR.setTargetPosition(-targetTicks);
            BL.setTargetPosition(-targetTicks);
            BR.setTargetPosition(targetTicks);

            for (DcMotor m : AllMotors)
                m.setPower(speed / Math.sqrt(2));

            while (OpModeReference.opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy()
                    && BR.isBusy())) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.update();
            }
            stopDriving();
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void strafeL (double inches, double speed) {
        strafe(-inches, speed);
    }
    public void strafeR (double inches, double speed) {
        strafe(inches, speed);
    }

    private void diagonalLBR (double inches, double speed){
        if (OpModeReference.opModeIsActive()) {
            int targetTicks = (int) (countsPerInch * inches * (-12 / 11));
            for (DcMotor m : AllMotors) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            FL.setTargetPosition(targetTicks);
            //FR.setTargetPosition(-targetTicks);
            //BL.setTargetPosition(-targetTicks);
            BR.setTargetPosition(targetTicks);

            for (DcMotor m : AllMotors)
                m.setPower(speed / Math.sqrt(2));

            while (OpModeReference.opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy()
                    && BR.isBusy())) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.update();
            }
            stopDriving();
            for (DcMotor m : AllMotors) {
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }


    }
    private void diagonalFRBL (double inches, double speed) {
        if (OpModeReference.opModeIsActive()) {
            int targetTicks = (int) (countsPerInch * inches * (-12 / 11));
            for (DcMotor m : AllMotors) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            //FL.setTargetPosition(targetTicks);
            FR.setTargetPosition(-targetTicks);
            BL.setTargetPosition(-targetTicks);
            //BR.setTargetPosition(targetTicks);

            for (DcMotor m : AllMotors)
                m.setPower(speed / Math.sqrt(2));

            while (OpModeReference.opModeIsActive() && (FL.isBusy() && FR.isBusy() && BL.isBusy()
                    && BR.isBusy())) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.update();
            }
            stopDriving();
            for (DcMotor m : AllMotors) {
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }


    //Tele-op Methods:

    public void launcherMono (double outtakePower) {
        OT.setPower(-outtakePower);
    }

    public void intakeMono  (double intakePower) {
        IT.setPower(intakePower);
    }

    public void bumper (boolean trig) {

        if (trig)
            BP.setPosition(0);
        else
            BP.setPosition(0.55);
    }

    public void mecanum () {

        double speed = -OpModeReference.gamepad1.left_stick_y / Math.sqrt(2);
        double strafe = -OpModeReference.gamepad1.left_stick_x;
        double rotate = -OpModeReference.gamepad1.right_stick_x;
        double movingSpeed;

        if (OpModeReference.gamepad1.left_bumper) {
            movingSpeed = 0.4;
        }
        else {
            movingSpeed = 0.8;
        }

        double leftFrontDir = Range.clip((speed - strafe - rotate), -1, 1) * movingSpeed;
        double rightFrontDir = Range.clip((speed + strafe + rotate), -1, 1) * movingSpeed;
        double leftBackDir = Range.clip((speed + strafe - rotate), -1, 1) * movingSpeed;
        double rightBackDir = Range.clip((speed - strafe + rotate), -1, 1) * movingSpeed;

        FL.setPower(leftFrontDir);
        FR.setPower(rightFrontDir);
        BL.setPower(leftBackDir);
        BR.setPower(rightBackDir);

//        OpModeReference.telemetry.addData("Central Velocity", speed*movingSpeed);
//        OpModeReference.telemetry.addData("Lateral Velocity", strafe*movingSpeed);
//        OpModeReference.telemetry.addData("Rotation", rotate*movingSpeed);
    }

}
