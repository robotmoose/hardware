//mid_arm.scad
//Laurin Fisher
//To make the middle segment of the robot arm. 

//Provides a robot arm segment that a servo can move. Change the variables below to get the size of the arm or servo size you need (in Milimeters). 
//Note: (Units are Milimeters)

use <servo_6_star.scad>;
use <robo_arm.scad>; 

////Make servo slots////

$fn = 30; 
error = 0.5; 
servo_arm_x_radius = 2.5; //Half an inch for the x-axis
servo_arm_y_radius = 2.5; //Same for the y-axis if using cross

arm_depth = 62 + error; 
arm_width = 20*2+ 60 + 2 * error; 
arm_height = 8 + error; 

servo_arm_width = 3.8; 
servo_arm_height = 1.8; 
servo_depth = 0.5; 

servo_inner_circle_radius = 0.35; 

servo_inner_radius = 5.5 + error; 
servo_peg_width = 9.5 + error; 
servo_peg_depth = 7 + error; 
servo_height = arm_height * 2; 

end_buffer = 1 + error ; //2mm filled on one end to hold servo

union(){
difference(){
    armBase(arm_width, arm_height, arm_depth); 
    translate([arm_width/2, 0, end_buffer])
        servo_end_6_star(servo_inner_radius, servo_height, servo_peg_width, servo_peg_depth);
}

translate([0, 0, 59 + error + arm_height]){
    rotate([0, 180, 0]){
difference(){
    armBase(arm_width, arm_height, arm_depth); 
    translate([arm_width/2, 0, end_buffer])
        servo_end_6_star(servo_inner_radius, servo_height, servo_peg_width, servo_peg_depth);
    }
}
}
}

//Inner supports/connections
wall_depth = 6 + error; 
wall_width = 23 + error; 
wall_height = 59 + error; 

translate([-wall_width/2, -arm_depth/2 ,0 ])
cube([wall_width, wall_depth, wall_height]); 

translate([-wall_width/2, arm_depth/2 - wall_depth ,0 ])
cube([wall_width, wall_depth, wall_height]); 
