//robo_arm.scad
//Provides a robot arm segment that a servo can move. Change the variables below to get the size of the arm or servo size you need (in Milimeters). 
//Note: (Units are Milimeters)

use <servo_6_star.scad>;

//Module armBase
//Provide the (width, height, depth) in centimeters of the robot arm segment (no servo holes included)
module armBase(w, h, d) {
////Robot Arm Base////
$fn = 50; //Round the Cylinder edges to make it more smooth

//Changeable Arm dimensions
width = w; 
height = h; 
depth = d; 

//Middle arm (rectangle)
translate([-width/2, -depth/2 , 0])
    cube([width, depth, height], 0); 

//Positive rounded end of arm
translate([width/2, 0, 0])
    cylinder(height, depth/2, depth/2); 

//Positive rounded end of arm
translate([-width/2, 0, 0])
    cylinder(height, depth/2, depth/2);
}

////Make servo slots////

$fn = 30; 
error = 0.5; 
servo_arm_x_radius = 2.5; //Half an inch for the x-axis
servo_arm_y_radius = 2.5; //Same for the y-axis if using cross

arm_depth = 62 + error; 
arm_width = 214 - arm_depth + 2 * error; 
arm_height = 8 + error; 

servo_arm_width = 3.8; 
servo_arm_height = 1.8; 
servo_depth = 0.5; 

servo_inner_circle_radius = 0.35; 

servo_inner_radius = 5.5 + error; 
servo_peg_width = 9.5 + error; 
servo_peg_depth = 7 + error; 
servo_height = arm_height * 2; 

end_buffer = 2 + error ; //2mm filled on one end to hold servo

servo_width = 20 + error; 
servo_length = 41 + error; 
/*
rotate([0,180,180]){
    translate([0,0,-arm_height]){
difference(){
union(){
difference(){
    armBase(arm_width, arm_height, arm_depth); 
    translate([arm_width/2, 0, end_buffer])
        servo_end_6_star(servo_inner_radius, servo_height, servo_peg_width, servo_peg_depth);
}
}

//Servo hole

translate([-arm_width/2-arm_depth/4,-servo_width/2,-1])
cube([servo_length, servo_width, arm_height*2]); 
}
}
}
*/
//2nd arm
difference(){
translate([0,0,27+error])
difference(){
    armBase(arm_width, arm_height, arm_depth); 
    translate([arm_width/2, 0, end_buffer])
        servo_end_6_star(servo_inner_radius, servo_height, servo_peg_width, servo_peg_depth);
}
translate([-arm_width/2-arm_depth/4,-servo_width/2, 27+error - arm_height -2])
cube([servo_length, servo_width, arm_height*2]); 
}
/*
//the attachment piece to be screwed on
translate([-20/2, -3/2 - arm_depth, 0])
cube([20, 3, arm_height + 27+error]); 