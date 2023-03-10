$fs = .2;
$fa = 2;

plate_wid   = 10;
plate_dpth  = 20;
plate_hgt   = 12;
plate_thk   = 2;
shaft_rad   = 2;
servo_cc    = 42;
shaft_off   = 5;

module bracket() {
  difference() {
    union() {
      cube([plate_wid,   plate_dpth + plate_thk, plate_hgt]);
      translate([0, -plate_thk, 0])
        cube([plate_hgt - plate_thk, 2*plate_thk, plate_hgt]);
    }
    translate([-1,plate_thk, plate_thk])
      cube([plate_wid+2, plate_dpth + plate_thk, plate_hgt]);
    translate([shaft_off,  7, -.1]) cylinder(3,1.75,1.75);
    translate([shaft_off, 17, -.1]) cylinder(3,1.75,1.75);
    translate([shaft_off, plate_thk + 6, 1 + plate_hgt/2]) 
      rotate([90, 0, 0]) cylinder(plate_thk+10, shaft_rad, shaft_rad);
  }
}
 
bracket();