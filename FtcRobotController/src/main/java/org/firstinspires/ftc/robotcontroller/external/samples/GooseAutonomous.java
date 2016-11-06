/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

@Autonomous(name = "Goose: Autonomous", group = "Goose")
@Disabled
public class GooseAutonomous extends LinearOpMode {

  static final double COUNTS_PER_MOTOR_REV = 1440;

  DcMotor leftMotor = null;
  DcMotor  rightMotor = null;
  DcMotor  shooter = null;
  LightSensor lightSensor = null;
  UltrasonicSensor ultrasonicSensor = null;

  @Override
  public void runOpMode() {

    lightSensor = hardwareMap.lightSensor.get("sensor_light");
    lightSensor.enableLed(true);

    ultrasonicSensor = hardwareMap.ultrasonicSensor.get("distance");

    leftMotor   = hardwareMap.dcMotor.get("m1");
    rightMotor  = hardwareMap.dcMotor.get("m2");
    leftMotor.setDirection(DcMotor.Direction.REVERSE);

    shooter = hardwareMap.dcMotor.get("shooter");

    leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    waitForStart();
    leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Немного проезжаем
    driveEncoder(0.6,0.6,8);

    // Стреляем шарами
    shooter.setPower(1);
    sleep(1000);

    // Поворачиваем к цели - белой линии и маяку
    driveEncoder(-1,1,5);

    // Вижу цель, не вижу препядствий
    driveEncoder(0.6,0.6,10);

    while(ultrasonicSensor.getUltrasonicLevel() > 20) {
      double left; double right;
      if (lightSensor.getLightDetected() < 0.50) {
        left = 0.6;
        right = 0.0;
      } else {
        left = 0.0;
        right = 0.6;
      }
      rightMotor.setPower(left);
      leftMotor.setPower(right);

      telemetry.addData("Distance: ", String.valueOf(ultrasonicSensor.getUltrasonicLevel()));
      telemetry.update();
    }
  }

  public void driveEncoder(double lpower, double rpower, int distance) {

    leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    leftMotor.setTargetPosition(distance);
    rightMotor.setTargetPosition(distance);

    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    leftMotor.setPower(lpower);
    rightMotor.setPower(rpower);

  }

  public void stopDriving() {
    leftMotor.setPower(0);
    rightMotor.setPower(0);
  }
}
