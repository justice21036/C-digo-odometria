/* Copyright (c) 2021 FIRST. All rights reserved.
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

import androidx.core.math.MathUtils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu''''''' of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp", group="Linear Opmode")
//@Disabled
public class teleOp extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    DcMotor frente_d; // Definição de dos motores em seus respectivos lados (direito e esquerdo)
    DcMotor frente_e;
    DcMotor tras_d;
    DcMotor tras_e;

    @Override
    public void runOpMode() {


// Mapeamento de Hardware ( Definição dos motores como "frente" e "trás")
        frente_d = hardwareMap.get(DcMotor.class, "frente_d");
        frente_e = hardwareMap.get(DcMotor.class, "frente_e");
        tras_d = hardwareMap.get(DcMotor.class, "tras_d");
        tras_e = hardwareMap.get(DcMotor.class, "tras_e");
        frente_d.setDirection(DcMotor.Direction.REVERSE);
        tras_d.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();



        while (opModeIsActive()) {

            telemetry.addData("frente_d", frente_d.getCurrentPosition());
            telemetry.addData("frente_e", frente_e.getCurrentPosition());
            telemetry.update();


            //MOVIMENTAÇÃO DO ROBÔ
            if (gamepad1.right_trigger > 0) {
                frente_d.setPower((gamepad1.right_trigger));
                frente_e.setPower((-gamepad1.right_trigger));
                tras_d.setPower((-gamepad1.right_trigger));
                tras_e.setPower((gamepad1.right_trigger));
            }

            if (gamepad1.left_trigger > 0) {
                frente_d.setPower((-gamepad1.left_trigger));
                frente_e.setPower((gamepad1.left_trigger));
                tras_d.setPower((gamepad1.left_trigger));
                tras_e.setPower((-gamepad1.left_trigger));
            }

            if ((gamepad1.left_stick_x > -0.75) || (gamepad1.left_stick_x < 0.25)) {
                if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                    frente_e.setPower((gamepad1.left_stick_y) + (gamepad1.left_stick_x));
                    tras_e.setPower((gamepad1.left_stick_y) + (gamepad1.left_stick_x));
                    frente_d.setPower((gamepad1.left_stick_y) - (gamepad1.left_stick_x));
                    tras_d.setPower((gamepad1.left_stick_y) - (gamepad1.left_stick_x));
                } else if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0) &&
                        (gamepad1.right_stick_x == 0) && (gamepad1.right_stick_y == 0)) {
                    frente_e.setPower(0);
                    tras_e.setPower(0);
                    frente_d.setPower(0);
                    tras_d.setPower(0);
                }
            } else if (gamepad1.left_stick_x > 0) {
                frente_e.setPower((-1));
                tras_e.setPower((-1));
                frente_d.setPower((1));
                tras_d.setPower((1));
            } else {
                frente_e.setPower((1));
                tras_e.setPower((1));
                frente_d.setPower((-1));
                tras_d.setPower((-1));
            }

            if ((gamepad1.right_stick_x > -0.75) || (gamepad1.right_stick_x < 0.25)) {
                if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                    frente_e.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 5);
                    tras_e.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 5);
                    frente_d.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 5);
                    tras_d.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 5);
                }
            } else if (gamepad1.right_stick_x > 0) {
                frente_e.setPower(-0.2);
                tras_e.setPower(-0.2);
                frente_d.setPower(0.2);
                tras_d.setPower(0.2);
            } else {
                frente_e.setPower(0.2);
                tras_e.setPower(0.2);
                frente_d.setPower(-0.2);
                tras_d.setPower(-0.2);
            }

        }

    }
}