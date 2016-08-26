//arm_body.scad
//Laurin Fisher
//Makes the holding for the servos along the arm, the arm itself. Units are in Milimeters. 

error = 0.5; 
$fn = 30; 

//The wrist dimensions
side_wall_depth = 6 + error; 

wrist_width = 41 + side_wall_depth*2 + error; 
wrist_length = 48 + side_wall_depth*2 + error; 
wrist_height = wrist_width; 

//Servo dimensions
servo_width = 41 + error; 
servo_length = 20 + error; 

difference(){
union(){
difference(){
union(){
difference(){
union(){ 
difference(){
//Make the cube/holding 
translate([-wrist_width/2, -wrist_length/2, 0])
    cube([wrist_width, wrist_length, wrist_height]); 

//Cut out the sides and middle of the cube for lightness
union(){
translate([-(wrist_width-side_wall_depth*2)/2,-(wrist_length-side_wall_depth*2)/2, -1])
    cube([wrist_width-side_wall_depth*2, wrist_length-side_wall_depth*2, 100]); //100 = arbitrary big number
    
translate([-50,-(wrist_length-side_wall_depth*2)/2, side_wall_depth])
    cube([100, wrist_length-side_wall_depth*2, wrist_height-side_wall_depth*2]); 
}
}
}

translate([-servo_width/2, -servo_length/2 + wrist_length/2, -side_wall_depth + wrist_height/2])
    cube([servo_width, servo_length, side_wall_depth*2]); 
}

//Make the side opening for the bottom servo
side_wall_servo_hole_y = -wrist_length/2 + side_wall_depth + servo_length; 
translate([-side_wall_depth/2 + wrist_width/2 -error*6.5, -side_wall_depth/2 + side_wall_servo_hole_y , 0])
    cube([side_wall_depth, side_wall_depth, wrist_height]); 

//Make the supports
//Bottom side 

translate([-wrist_width/2+side_wall_depth/2, -wrist_height/2+ side_wall_depth/2, 0])
rotate([0,0,-40])
    cube([side_wall_depth, wrist_height + side_wall_depth*2, side_wall_depth]);

//Top Side

translate([-wrist_width/2+side_wall_depth/2, -wrist_height/2+ side_wall_depth/2, wrist_height-side_wall_depth])
rotate([0,0,-40])
    cube([side_wall_depth, wrist_height + side_wall_depth*2, side_wall_depth]);
    
//Side w/ Servo
rotate([60,0,0])
translate([wrist_width/2-side_wall_depth, 0, 0])
    cube([side_wall_depth, wrist_height, side_wall_depth]); 
    

translate([wrist_width/2-side_wall_depth, -side_wall_depth/2, wrist_height-side_wall_depth])
rotate([-60,0,0])
    cube([side_wall_depth, wrist_height, side_wall_depth]);
}

//Make the bottom opening for the wires
wire_cube_size = 13 + error; 
translate([-wire_cube_size/2, -wire_cube_size/2 - wrist_length/2, -wire_cube_size/2 + wrist_height/2])
cube([wire_cube_size,wire_cube_size,wire_cube_size]); 


//Make the opening for the pegs on the left side
rad_error = 0.1; 
//Bottom Right when facing side
translate([-wrist_width/2 - side_wall_depth/4, -wrist_length/2 + side_wall_depth/2, side_wall_depth/2])
rotate([0,90,0])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

//Top Right
translate([-wrist_width/2 - side_wall_depth/4, -wrist_length/2 + side_wall_depth/2, wrist_height-side_wall_depth/2])
rotate([0,90,0])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

//Top Left
translate([-wrist_width/2 - side_wall_depth/4, wrist_length/2 - side_wall_depth/2, wrist_height-side_wall_depth/2])
rotate([0,90,0])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

//Bottom Left
translate([-wrist_width/2 - side_wall_depth/4, wrist_length/2 - side_wall_depth/2, side_wall_depth/2])
rotate([0,90,0])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 


}
}
/*
// TOP OF THE WRIST BOX
translate([-wrist_width, -wrist_length, -side_wall_depth])
cube([wrist_width*2, wrist_length*2, wrist_height]); 

//Make the pegs
rad_error = 0.1; 
//Top Right
translate([wrist_width/2-side_wall_depth/2, wrist_length/2- side_wall_depth/2, wrist_height - side_wall_depth])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

//Bottom Right
translate([wrist_width/2-side_wall_depth/2, -wrist_length/2 + side_wall_depth/2, wrist_height - side_wall_depth])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

//Bottom Left
translate([-wrist_width/2+side_wall_depth/2, -wrist_length/2 + side_wall_depth/2, wrist_height - side_wall_depth])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

//Top Left
translate([-wrist_width/2+side_wall_depth/2, wrist_length/2 - side_wall_depth/2, wrist_height - side_wall_depth])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

*/
translate([-wrist_width/2 -0.5, -wrist_length/2 -0.5, wrist_height - side_wall_depth])
cube([wrist_width +1, wrist_length+1, wrist_height]); 

}

rad_error = 0.1; 
//Top Right
translate([wrist_width/2-side_wall_depth/2, wrist_length/2- side_wall_depth/2, wrist_height - side_wall_depth -side_wall_depth/4])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

//Bottom Right
translate([wrist_width/2-side_wall_depth/2, -wrist_length/2 + side_wall_depth/2, wrist_height - side_wall_depth -side_wall_depth/4])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

//Bottom Left
translate([-wrist_width/2+side_wall_depth/2, -wrist_length/2 + side_wall_depth/2, wrist_height - side_wall_depth -side_wall_depth/4])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 

//Top Left
translate([-wrist_width/2+side_wall_depth/2, wrist_length/2 - side_wall_depth/2, wrist_height - side_wall_depth -side_wall_depth/4])
cylinder(side_wall_depth/2, 1+rad_error, 1+rad_error); 


/*
//The removable side?

//Bottom Right when facing side
translate([-side_wall_depth/2 - wrist_width/2 -15 + side_wall_depth - side_wall_depth/4, -wrist_length/2 + side_wall_depth/2, side_wall_depth/2])
rotate([0,90,0])
cylinder(side_wall_depth/2, 1, 1); 

//Top Right
translate([-side_wall_depth/2 - wrist_width/2 -15 + side_wall_depth - side_wall_depth/4, -wrist_length/2 + side_wall_depth/2, wrist_height-side_wall_depth/2])
rotate([0,90,0])
cylinder(side_wall_depth/2, 1, 1); 

//Top Left
translate([-side_wall_depth/2 - wrist_width/2 -15 + side_wall_depth - side_wall_depth/4, wrist_length/2 - side_wall_depth/2, wrist_height-side_wall_depth/2])
rotate([0,90,0])
cylinder(side_wall_depth/2, 1, 1); 

//Bottom Left
translate([-side_wall_depth/2 - wrist_width/2 -15 + side_wall_depth - side_wall_depth/4, wrist_length/2 - side_wall_depth/2, side_wall_depth/2])
rotate([0,90,0])
cylinder(side_wall_depth/2, 1, 1); 

//Make the removable side wall
translate([-side_wall_depth/2 - wrist_width/2 -15 +side_wall_depth/2, -wrist_length/2, 0])
cube([side_wall_depth/2, wrist_length, wrist_height]); 
*/