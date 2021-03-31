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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="One Person Drive Static", group="Linear Opmode")
public class MainDriveArmUp extends LinearOpMode {

    @Override
    public void runOpMode() {
        SevenTheRobot seven = new SevenTheRobot(this);
        seven.initialize();
        waitForStart();

        while (opModeIsActive()){
            double intakePower = 0;
            double outtakePower = gamepad1.right_trigger;

            if (gamepad1.x)
                //seven.armDown();
                seven.setArm(535);
            else
                //seven.armUp();
                seven.setArm(0);

            if (gamepad1.a)
                intakePower = 0.8;
            else if (gamepad1.b)
                intakePower = 0.7;
            else if (gamepad1.y)
                intakePower = -1;

            seven.bumper(gamepad1.right_bumper);

            if (gamepad1.left_trigger > 0.25)
                seven.closeClaw();
            else
                seven.openClaw();

            seven.mecanumInv();
            seven.launcherMono(outtakePower);
            seven.intakeMono(intakePower);
//            telemetry.addData("yo", seven.Arm.getCurrentPosition());
//            telemetry.update();
        }

    }
}
