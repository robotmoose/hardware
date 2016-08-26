//gear.scad
//Laurin Fisher
//June 23, 2016

//Makes a generic gear. The code can be copied then modified for customized use. 

use <robo_arm.scad>; 

pegs = 16; //how many pegs stick out from the circle
gear_height = 5; //The inner circle height
gear_radius = 5; 

peg_width = 15; //Make this bigger than the gear_radius
peg_height = 5; 
peg_depth = 1.5; 


//Module to make a gear with rounded edges using armBase()
module gear(gear_height, gear_radius, pegs, peg_width, peg_depth, peg_height){
    
    //The cylinder size (inner circle)
    cylinder(gear_height, gear_radius, gear_radius); 

    //The pegs, each distributed evenly over the 360 deg
    angle = 360/pegs; 
    for( i = [0:pegs]){
        rotate([0, 0, angle*i])
            armBase(peg_width, peg_height, peg_depth); 
            //comment armBase and uncomment line below for non-rounded gear pegs
            //cube([peg_width, peg_depth, peg_height], 0); 
    }
}

gear(gear_height, gear_radius, pegs, peg_width, peg_depth, peg_height); 