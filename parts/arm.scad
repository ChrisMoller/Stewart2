$fs = .2;
$fa = 2;

arm_thk     =  2.0;
sleeve_len  = 12.0;
rear_wid    =  5.5;
front_wid   =  4.3;
shaft_rad   =  2.0;
shaft_hgt   = 10.0;
shell_thk   =  1.0;
ball_rad    =  4.7;

module pocket() {
  linear_extrude(arm_thk) {
    polygon([[-shaft_rad,  front_wid / 2],
             [-shaft_rad, -front_wid / 2],
             [sleeve_len + 1, -rear_wid / 2],
             [sleeve_len + 1,  rear_wid / 2]]);
  }
}

module arm() {
  linear_extrude(arm_thk + 2 * shell_thk) {
  polygon([[-shaft_rad,   shell_thk + front_wid / 2],
           [-shaft_rad, -(shell_thk + front_wid / 2)],
           [sleeve_len + 1 + shell_thk, -(shell_thk + rear_wid / 2)],
           [sleeve_len + 1 + shell_thk,  (shell_thk + rear_wid / 2)]]);
  }
}

module socket() {
  translate([0, 0, -(arm_thk + shell_thk)])
    difference() {
      arm();
      translate([2, 0, shell_thk]) pocket();
    }
}

socket();
cylinder(shaft_hgt, shaft_rad, shaft_rad);
translate([0, 0, shaft_hgt]) sphere(ball_rad);
