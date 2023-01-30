$fs = .2;
$fa = 2;

shaft_hgt = 15;
shaft_rad = 2;
ball_rad = 4.7;
hole_rad = 1.05;

//difference() {
//  union () {
    cylinder(shaft_hgt,shaft_rad, shaft_rad);
    translate([0,0,6]) cylinder(2,ball_rad,ball_rad);
    translate([0,0, shaft_hgt - 1]) sphere(ball_rad);
//  }
//  translate([0,0,-.1]) 
//     cylinder(shaft_hgt+.2, hole_rad, hole_rad);
//}