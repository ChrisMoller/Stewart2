$fs = .2;
$fa = 2;

module triangle(side, thickness) {
 radius = 2 * side * cos(60);
 x0 = radius * cos(0);
 y0 = radius * sin(0);
 x1 = radius * cos(120);
 y1 = radius * sin(120);
 x2 = radius * cos(240);
 y2 = radius * sin(240);
 linear_extrude(height=thickness) {
   polygon(points=[[x0,y0],[x1,y1], [x2,y2]]);
 }
}

board_len    =  80;
board_wid    =  50;
board_thk    =   1;
standoff_hgt =   6;
standoff_rad =   2.5;
mnt_hgt      =   2.5;
mnt_rad      =   1;
base_thk     =   2;
base_side    = 170;

servo_cc  = 38;
servo_wid = 12;
trim_side = servo_cc + servo_wid;

hole_x_sep   = 22;
hole_y_sep   = 10;
hole_rad     =  1.9;
hole_setback = 17;

module servos(x) {
    translate([x, 30,board_thk]) cube([20, 12,25]);
    translate([x,-37,board_thk]) cube([20, 12,25]);
}

module standoff(x,y) {
  translate([x,y,0]) {
    cylinder(standoff_hgt, standoff_rad,standoff_rad);
    cylinder(standoff_hgt+mnt_hgt, mnt_rad, mnt_rad);
  }
}

module arduino() {
    translate([-40,-25,0]) {
        standoff(3,11);             // ok
        standoff(3,board_wid-11);   // 31 ok
        standoff(board_len-3,board_wid-3); // 33 -> 44
        standoff(board_len-3, 3);   //10
    }
}

difference() {
 triangle(base_side,base_thk);
 translate([base_side-trim_side,0,-1]) 
    triangle(trim_side+1,base_thk+2);
 rotate([0,0,120]) 
    translate([base_side-trim_side,,0,-1]) 
    triangle(trim_side+1,base_thk+2);
 rotate([0,0,240]) 
    translate([base_side-trim_side,,0,-1]) 
    triangle(trim_side+1,base_thk+2);
}
translate([0,0,base_thk]) color("red") arduino();