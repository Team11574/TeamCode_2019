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

package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="opNoDist (USE THIS)", group="Iterative Opmode")
//@Disabled
public class opNoDist extends OpMode
{
    // Declare OpMode members.
    private int frame = 0;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        Robot.init_teleOp(telemetry, hardwareMap,gamepad1,gamepad2);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        Robot.runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    private boolean max_mode = true;
    private boolean a_cur = false;
    private boolean orientation_mode = false;
    private boolean b_cur = false;
    private boolean x_cur = false;
    private boolean slow_intake = false;
    private boolean previous_block_state = false;
    @Override
    public void loop() {
        frame++;
        frame = frame % 100; //up to 100

        double vx = Robot.gamepad1.left_stick_x;
        double vy = Robot.gamepad1.left_stick_y;
        double rot = -Robot.gamepad1.right_stick_y;
        double rot2 = Robot.gamepad1.right_stick_x;


        if (Robot.gamepad1.a){
            if ( !a_cur ) {
                a_cur = true;
                max_mode = !max_mode;
            }
        }
        else {
            if (a_cur) {
                a_cur = false;
            }
        }
        if(Robot.gamepad1.b){
            if ( !b_cur ) {
                b_cur = true;
                orientation_mode = !orientation_mode;
            }

        }
        else {
            if (b_cur) {
                b_cur = false;
            }
        }

        if(Robot.gamepad1.x){
            if ( !x_cur ) {
                x_cur = true;
                slow_intake= !slow_intake;
            }

        }
        else {
            if (x_cur) {
                x_cur = false;
            }
        }
        double div = 1;
        if (slow_intake) {
            div = 2 ;
        }
        //test if block already there

        //left is intake

        //telemetry.addData("Block found?:",isBlock());
        if( ( (gamepad1.left_trigger - gamepad1.right_trigger) > .15)) {
            if (frame % 1 == 0) { //check every frame
                //previous_block_state = Robot.isBlock(); //only check isBlock every once in a while
                previous_block_state = false;

            }
            if(!previous_block_state) { //need to check based on previous state
                Robot.intakeR.setPower((Robot.gamepad1.left_trigger - Robot.gamepad1.right_trigger) / div);
                Robot.intakeL.setPower((Robot.gamepad1.left_trigger - Robot.gamepad1.right_trigger) / div);
            }

        }
        else {
            Robot.intakeR.setPower((Robot.gamepad1.left_trigger - Robot.gamepad1.right_trigger) / div);
            Robot.intakeL.setPower((Robot.gamepad1.left_trigger - Robot.gamepad1.right_trigger) / div);
        }


        //intakeR.setPower((gamepad1.left_trigger - gamepad1.right_trigger) / div);
        //intakeL.setPower((gamepad1.left_trigger - gamepad1.right_trigger) / div);


        Robot.pantagraph.setPower( ((Robot.gamepad1.left_bumper ? 1 : 0) -(Robot.gamepad1.right_bumper ? 1 : 0 )) );




        Robot.telemetry.addData("Orietnation_mode on?:", orientation_mode);
        Robot.telemetry.addData("test_mode on?:", slow_intake);
        Robot.telemetry.addData("Max_mode on?:", max_mode);
        if(!orientation_mode) {

            Robot.setMotors(-vx,vy,rot2*1.5,max_mode); //maybe I can edit how strong the turn factor is with another variable
            //maybe could control with another controller, and it shoudl print out the current value
            // Show the elapsed game time and wheel power.
            //telemetry.addData("Motors", ("Vx: " + -vx + "----Vy" + vy + "---Rot2" + rot2));
            //double[] powers = motorPower.calcMotorsFull(vx, vy, rot2);//can also calc max, which always goes the fastest
            //telemetry.addData("Motors", "v0 (%.2f), v1 (%.2f), v2 (%.2f), v3 (%.2f)", powers[0], powers[1], powers[2], powers[3]);
            //telemetry.addData("First ang", "Ang1:" + imu.getAngularOrientation().firstAngle);
        }
        else {
            //TEST WITH MAX MODE OFF
            rot2 = -rot2;
            //maybe need to swap rot2 and rot1
            //somethign else is a problem though,s
            double ang = Math.atan2(vy,-vx);
            double mag = Math.sqrt(vy*vy + vx*vx);
            //atan works like this
            //         90
            //        ^
            // +-180  <   > 0
            //      -90 V
            //having some trouble with resetting
            //When both 0, shoudl be equal to -90
            //At 0, as if 90, so goes forward (CHECK)
            //At 90, as if
            double imu_ang =(Robot.imu.getAngularOrientation().firstAngle + 0*Math.PI/2.0) % Math.PI;
            double goal_ang = (ang + (-imu_ang))  ; //This needs to be offset by some number of degrees, because when both are zero, it should move foward
            double ang_rot = Math.atan2(rot,rot2)- Math.PI/2.0; //angle your game stick is facing
            double mag_rot = Math.sqrt(rot*rot + rot2*rot2);
            double goal_rot_ang = Robot.distanceTurn(-imu_ang,ang_rot); //
            double nrot = 0;
            double nvx = Math.cos(goal_ang) * mag;
            double nvy = Math.sin(goal_ang) * mag;
            if(max_mode) {
                if (Math.abs(goal_rot_ang) > 0.40) { //amount goal angle needs to be off to actually turn in that direction

                    nrot = goal_rot_ang * (mag_rot / 2.0); //rotation is the direction of rotation
                    //test value of /2.0 just ot
                    //maybe do if cur_ang + ang_turn > goal_ang then turn only goal_ang-cur_ang
                } else {

                    nrot = goal_rot_ang * (mag_rot) * (Math.abs(goal_rot_ang) + .1); //rotation is the direction of rotation
                    // nrot = nrot>0?.1 + nrot:-.1+nrot;
                }
            }
            else {
                nrot = goal_rot_ang;
            }
            //maybe need to make the turn less dominant
            //-90first angle to 180 goal angle
            double mult = 1;
            Robot.setMotors(nvx,nvy,nrot*mult,max_mode);
            //only going from -90 to 90 for some reason, getting to zero at two different places
            double[] powers = motorPower.calcMotorsFull(nvx, nvy, nrot);//can also calc max, which always goes the fastest
            //telemetry.addData("Ang of right stick",ang * 180/Math.PI);
            telemetry.addData("Ang of left stick",ang_rot * 180/Math.PI); //wait, how did this get to 270?
            telemetry.addData("Info", ("Goal rot _ang: " + goal_rot_ang * 180/Math.PI));
            telemetry.addData("Motors", ("Vx: " + nvx + "----Vy" + nvy + "---Rot" + nrot));
            telemetry.addData("Motors", "v0 (%.2f), v1 (%.2f), v2 (%.2f), v3 (%.2f)", powers[0], powers[1], powers[2], powers[3]);
            telemetry.addData("First ang", "Ang1:" + (Robot.imu.getAngularOrientation().firstAngle) * 180/Math.PI );
            //I think it swaps at 180 instead of 360 on the angle orientation mode with first angle
        }
        //assuming that this returns as its supposed to
        //imu.getAngularVelocity();
        //imu.getAcceleration();
        //imu.getPosition(); //coudl use this to check if everythign isin the eiright position during autonomous
        //use these to check how close to the goal the robot is actually achieving



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

    @Override
    public void stop() {
    }





    public double atan2(double y, double x) {
        if(x==0) {
            if (y > 0) {
                return Math.PI/2.0;
            }
            else {
                return 3*Math.PI/2.0;
            }
        }
        return Math.atan(y/x)+((((Math.abs(x)/x)-1)/2) * Math.PI);
    }







}