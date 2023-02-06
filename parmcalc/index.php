<!DOCTYPE html>
<html>
<body>

<?php
  $sr = $_GET['sradius']; 
  $sa = $_GET['sangle'];
  $ar = $_GET['aradius']; 
  $aa = $_GET['aangle'];
?>

<img src="pic-01m.jpg">
<img src="pic-02m.jpg"><br>
  
<form action="index.php" method="get">
  Servo radius: <input type="number" step="0.1" id='sradius' name="sradius"
		       value="<?php echo (isset($sr))?$sr:'';?>"
		       ><br>
  Servo angle:  <input type="number" id='aangle'  name="sangle"
		       value="<?php echo (isset($sa))?$sa:'';?>"
		       ><br>
  Anchor radius: <input type="number" step="0.1" id='aradius' name="aradius"
		       value="<?php echo (isset($ar))?$ar:'';?>"
			><br>
  Anchor angle:  <input type="number" id='aangle'  name="aangle"
		       value="<?php echo (isset($aa))?$aa:'';?>"
			><br>
  <input type="submit">
</form>

<?php
  $sradius = $_GET['sradius']; 
  $sangle  = $_GET['sangle'] / 2.0;

  $aradius = $_GET['aradius']; 
  $aangle  = $_GET['aangle'] / 2.0;

  $angs = array(0, 240, 120);

  echo "<br><br>// Servo locations<br><br>";
  $i = 0;
  foreach ($angs as $a) {
    $v = number_format ($sradius * cos(deg2rad($a + $sangle)), 2);
    echo "#define B", $i, "x ", $v, "<br>";

    $v = number_format ($sradius * sin(deg2rad($a + $sangle)), 2);
    echo "#define B", $i++, "y ", $v, "<br>";

    $v = number_format ($sradius * cos(deg2rad($a - $sangle)), 2);
    echo "#define B", $i, "x ", $v, "<br>";

    $v = number_format ($sradius * sin(deg2rad($a -$sangle)), 2);
    echo "#define B", $i++, "y ", $v, "<br>";
  }

  echo "<br><br>// Anchor locations<br><br>";
  $i = 0;
  foreach ($angs as $a) {
    $v = number_format ($aradius * cos(deg2rad($a + $aangle)), 2);
    echo "#define P", $i, "x ", $v, "<br>";

    $v = number_format ($aradius * sin(deg2rad($a + $aangle)), 2);
    echo "#define P", $i++, "y ", $v, "<br>";

    $v = number_format ($aradius * cos(deg2rad($a - $aangle)), 2);
    echo "#define P", $i, "x ", $v, "<br>";

    $v = number_format ($aradius * sin(deg2rad($a -$aangle)), 2);
    echo "#define P", $i++, "y ", $v, "<br>";
  }
?>  

</body>
</html>

