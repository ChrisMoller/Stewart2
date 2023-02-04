/***

This code runs on two different computers simultaneously,
the Arduino and some machine running a browser.  The Arduino
side does three things, it serves a web page to the browser,
which allows the user to set various parameters, it reads
the parameters, and it interprets the parameters to control
the servos.  The broswer side runs a web page with the controls.
The Arduino side is coded in a subset of C++; the browser side
is in a mix of Javascript and HTML.  In the code below,
argument strings to client.print () calls are the HTML and
Javascript being served to the browser.
					     
***/

#include <SPI.h>
#include <SD.h>
#include <WiFiNINA.h>
#include <JOAAT.h>
#include <Servo.h>
#include <BasicLinearAlgebra.h>

//#define LOG_SERVO

/*** platform description ***/

#define ARM_LENGTH	 1.6
#define LEG_LENGTH	 13.4

#if 1
// Servo locations

#define B0x 12.50
#define B0y 4.55
#define B1x 12.50
#define B1y -4.55
#define B2x -2.31
#define B2y -13.10
#define B3x -10.19
#define B3y -8.55
#define B4x -10.19
#define B4y 8.55
#define B5x -2.31
#define B5y 13.10


// Anchor locations

#define P0x 10.33
#define P0y 4.82
#define P1x 10.33
#define P1y -4.82
#define P2x -0.99
#define P2y -11.36
#define P3x -9.34
#define P3y -6.54
#define P4x -9.34
#define P4y 6.54
#define P5x -0.99
#define P5y 11.36
#else
// locations of anchors wrt to platform plane
#define P0x -1.430
#define P0y  0.976
#define P1x -1.432
#define P1y -1.000
#define P2x -0.137
#define P2y -1.743
#define P3x  1.577
#define P3y -0.752
#define P4x  1.582
#define P4y  0.740
#define P5x -0.130
#define P5y  1.726

// locations of servos wrt base
#define B0x -5.40268
#define B0y  2.60979
#define B1x -5.40268
#define B1y -2.60979
#define B2x  0.441194
#define B2y -5.98376
#define B3x  4.96149
#define B3y -3.37396
#define B4x  4.96149
#define B4y  3.37396
#define B5x  0.441194
#define B5y  5.98376
#endif

// structs and classes

typedef struct {
  double x;
  double y;
} pos_s;

typedef struct {
  double x;
  double y;
  double z;
} pos3_s;

class Platform {
  public:
  Platform() {
    anchor[0].x = P0x; anchor[0].z = 0.0; anchor[0].y = P0y;
    anchor[1].x = P1x; anchor[1].z = 0.0; anchor[1].y = P1y;
    anchor[2].x = P2x; anchor[2].z = 0.0; anchor[2].y = P2y;
    anchor[3].x = P3x; anchor[3].z = 0.0; anchor[3].y = P3y;
    anchor[4].x = P4x; anchor[4].z = 0.0; anchor[4].y = P4y;
    anchor[5].x = P5x; anchor[5].z = 0.0; anchor[5].y = P5y;
  }
  pos3_s anchor[6];
};

class Base {
  public:
  Base() {
    pos[0].x = B0x; pos[0].y = B0y;
    pos[1].x = B1x; pos[1].y = B1y;
    pos[2].x = B2x; pos[2].y = B2y;
    pos[3].x = B3x; pos[3].y = B3y;
    pos[4].x = B4x; pos[4].y = B4y;
    pos[5].x = B5x; pos[5].y = B5y;
  }
  pos_s pos[6];
};

#define NUM_SERVOS 6
Servo myServo[NUM_SERVOS];

#define SERVO0_PIN 0
#define SERVO1_PIN 1
#define SERVO2_PIN 2
#define SERVO3_PIN 3
#define SERVO4_PIN 4
#define SERVO5_PIN 5

Platform myPlatform;
Base myBase;

double h0;


// x-axis
static BLA::Matrix<4,4> rotatePitch(double r)
{
  BLA::Matrix<4,4> mtx = {
    1.0,     0.0,    0.0,        0.0,
    0.0,     cos(r), -sin(r),    0.0,
    0.0,     sin(r),  cos(r),    0.0,
    0.0,     0.0,     0.0,       1.0
  };
  return mtx;
}

// y-axis
static BLA::Matrix<4,4> rotateRoll(double r)
{
  BLA::Matrix<4,4> mtx = {
    cos(r),  0.0, sin(r), 0.0,
    0.0,     1.0, 0.0,    0.0,
    -sin(r), 0.0, cos(r), 0.0,
    0.0,     0.0, 0.0,    1.0
  };
  return mtx;
}

// z-axis
static BLA::Matrix<4,4> rotateYaw(double r)
{
  BLA::Matrix<4,4> mtx = {
    cos(r),  -sin(r), 0.0, 0.0,
    sin(r),   cos(r), 0.0, 0.0,
    0.0,      0.0,    1.0, 0.0,
    0.0,      0.0,    0.0, 1.0
  };
  return mtx;
}

static double length(BLA::Matrix<4> v)
{
  return sqrt((v(0) * v(0)) +
	      (v(1) * v(1)) +
	      (v(2) * v(2)));
}

#define R2D(r) ((r / M_PI) * 180.0)
#define D2R(d) ((d / 180.0) * M_PI)

double alpha[6] = {180.0, 0.0, 180.0, 0.0, 180.0, 0.180};
double incr[8];
double pincr;

/******* parameter structure *********/
/* collected information about each parameter, it's name, it's current value
   and a hash value which will be explaned below.*/

/******* parameter labels *******/

#define ETY(v) #v
const char * lbls[] = {
#include "lbls.h"
};
size_t lbl_cnt = sizeof(lbls)/sizeof(char *);

/******* parameter enums *******/

#define ETY(v) PARM_ ## v
enum {
#include "lbls.h"
};

typedef struct {
  double val;
  uint32_t hash;
  int idx;
  const char *name;
} parm_s;

/* A pointer to an unallocated array of the parameters. */
parm_s *parms = nullptr;
int *plut = nullptr;

// functions and subroutines

static void showMatrix (const char *title, BLA::Matrix<4,4> & mx)
{
  Serial.println (title);
  for (int row = 0; row < 4; row++) {
    int col = 0;
    Serial.println (String (mx (row, col++)) + "\t" +
		    String (mx (row, col++)) + "\t" +
		    String (mx (row, col++)) + "\t" +
		    String (mx (row, col++)) + "\t");
  }
}

static void showVector (const char *title, BLA::Matrix<4> & mx)
{
  Serial.print (String (title) + ": ");
  Serial.println (String (mx (0)) + "\t" +
		  String (mx (1)) + "\t" +
		  String (mx (2)) + "\t" +
		  String (mx (3)) + "\t");
}

static void update_alpha()
{
  const double mdx    = 0.825;
  const double mdy    = 1.000;
  const double mdz    = 0.653;
  const double mpitch = 0.657;
  const double mroll  = 0.835;
  const double myaw   = 1.119;

  double dx     = parms[plut[PARM_pdx]].val;
  double dy     = parms[plut[PARM_pdy]].val;
  double dz     = parms[plut[PARM_pdz]].val;
  double pitch  = parms[plut[PARM_ppitch]].val;
  double roll   = parms[plut[PARM_proll]].val;
  double yaw    = parms[plut[PARM_proll]].val;

  BLA::Matrix<4,4> pRot = rotatePitch (D2R (pitch));
  BLA::Matrix<4,4> rRot = rotateRoll (D2R (roll));
  BLA::Matrix<4,4> yRot = rotateYaw (D2R (yaw));
  BLA::Matrix<4,4> cRot = pRot * rRot * yRot;
  double kdz = (dz + h0) - 1.9;
  BLA::Matrix<4> P0 = {dx, dy, kdz, 1.0};


  //double alpha_stage[6];
  bool is_valid = true;

  for (int i = 0; i < NUM_SERVOS; i++) {
    // Eq 3
    
    BLA::Matrix<4> P1 = {myPlatform.anchor[i].x,
                         myPlatform.anchor[i].y,
                         myPlatform.anchor[i].z, 
                         1.0};
                                      
    BLA::Matrix<4> P  = P0 +  (cRot * P1);

    BLA::Matrix<4> B = {myBase.pos[i].x,
                        myBase.pos[i].y, 
                        0.0,
                        1.0};
    BLA::Matrix<4> deltaPB = P - B;
    
    // Eq 9
    double beta = (i & 1) ? (M_PI/6.0) : (-M_PI/6.0);
    //    double beta = -M_PI/6.0;
  
    double l = length (deltaPB);

    double L  =
      (l * l) - ((LEG_LENGTH * LEG_LENGTH) - 
                 (ARM_LENGTH * ARM_LENGTH));

    double M  = 2.0 * ARM_LENGTH * deltaPB(2);
    double N  = 2.0 * ARM_LENGTH *
      (deltaPB(0) * cos (beta) + deltaPB(1) * sin (beta));
    N = -fabs (N);
    double arg = L / sqrt ((M * M) + (N * N));
    double alphaX = asin (arg) - atan2 (N, M);
    

    if (isnan (alphaX)) {
      is_valid = false;
      //      break;
    }
    else {
      alpha[i] = R2D(alphaX);
      //      const double slope = -180.0 / 133.0;
      //      alpha[i] = (slope * (alpha[i] + 43.0)) + 180;
      if (0 == (i & 1)) alpha[i] = 180.0 - alpha[i];
      myServo[i].write( alpha[i]);
    }
  }
}



/********** global variables ***********/

/* An instantiation of a class that generates hashes */
JOAAT joaat;

/* An instantiation of a class that turns the Arduino into a web server */
WiFiServer server(80);

/* WiFi credentials */
#define SECRET_SSID "NETGEAR80"
#define SECRET_PASS "magical574"
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password 

/* holds th local ip address.  not sure i need it... */
IPAddress ip;

bool run  = true;
bool demo = true;

/*****
      One of my rules is to never enter the same information more than once
      --it's too easy for the different versions to get out of sync.  What the
      above does is define the macro ETY two different ways and then include a
      a file containing a list of parameter names for each of those ways.
      Under the first definition, that file expands into a char * vector of
      the names; under the second def it expands into a list of indices into
      that vactor, and both lists are guaranteed to stay in sync.

      A macro, BTW, (like where it says "#define ETY(v) #v" above) is a
      string- manipulation thing.  The compiler works in a lot of different
      stages, but the first one goes through the code looking for things like
      #define statements and then, when it sees a string in the code that
      matches a definition, ETY in this case, it plugs the argument, v in this
      case, into the expansion ("#v" or "PARM_ ## v" in these cases) so
      entries in lbl.h, e,g., ETY(pdx), get expanded into "pdx" or
      plut[PARM_pdx] respectively.  (The actual expansion process is pretty
      complex--I won't go into it here.)  (The preprocessing stage also
      handles things like #if statements, comment removal, and so on--a whole
      lot of stuff.)
 *****/


/******** subroutines and functions *********/


/********* comparison function for qsort (see below) **********/

int cmp_parm (void *s1, void * s2)
{
  uint32_t a = ((parm_s *)s1)->hash;
  uint32_t b = ((parm_s *)s2)->hash;
  return (a == b) ? 0
    : ((a < b) ? -1 : 1);
}


/********* comparison function for bsearch (see below) **********/

int cmp_parm_str (const void *s1, const void *s2)
{
  union {
    const void *v;
    struct {
      uint32_t m;
      uint32_t l;
    }u;
  } a_u;
  a_u.v = s1;
  uint32_t a = a_u.u.m;
  uint32_t b = ((parm_s *)s2)->hash;
  
  return (a == b) ? 0
    : ((a < b) ? -1 : 1);
}


/********* utility function for qsort (see below) **********/

void swap(void* v1, void* v2, int size)
{
  char buffer[size];
  memcpy(buffer, v1, size);
  memcpy(v1, v2, size);
  memcpy(v2, buffer, size);
}


/***

All the control parameters have names like "pdx."  In order to access or
change the value of a parameter, it has to be looked up in the parameter
array, which means matching the string containing the name against the names
in the parameter array.  The "unsophisticated" programmer usually does this
with a series of "if name = this, then do ..., else if name = that..." etc.
Comparing strings is very slow and if you have a lot of names in the array
all these compares take a linearly long time.  Comparing numbers is vastly
faster so the trick is to turn name strings into signature numbers.  This is
called "hashing."

Further, there are faster ways of looking up numbers than by scanning a list,
one of which is called a binary search.  If the list is sorted by hash number
then, if you start with a first compare against the middle of the list, you
can tell immediately whether the target hash number is greater or less than
what you're looking for so you can then skip immediately to either the quarter-
way point on the list, or the three-quarter, and keep doing that recursively
until you find the target.  This is an O(log n) process, i.e., much faster than
a linear search, but it requires the array of targets to be sorted.  qsort,
below is the algo to do the sorting, as seen in action later on.

***/

void qsort(void* v, int size, int left, int right,
      int (*comp)(void*, void*))
{
  void *vt, *v3;
  int i, last, mid = (left + right) / 2;
  if (left >= right)
    return;

  void* vl = (char*)(v + (left * size));
  void* vr = (char*)(v + (mid * size));
  swap(vl, vr, size);
  last = left;
  for (i = left + 1; i <= right; i++) {
    vt = (char*)(v + (i * size));
    if ((*comp)(vl, vt) > 0) {
      ++last;
      v3 = (char*)(v + (last * size));
      swap(vt, v3, size);
    }
  }
  v3 = (char*)(v + (last * size));
  swap(vl, v3, size);
  qsort(v, size, left, last - 1, comp);
  qsort(v, size, last + 1, right, comp);
}	

/***

The other half of the process is the binary search of the sorted list, as done
in bsearch, also seen in use later.

***/

void *bsearch (const void *key, const void *base0,
	       size_t nmemb, size_t size,
	       int (*compar)(const void *, const void *))
{
  const char *base = (const char *) base0;
  int lim, cmp;
  const void *p;

  for (lim = nmemb; lim != 0; lim >>= 1) {
    p = base + (lim >> 1) * size;
    cmp = (*compar)(key, p);
    if (cmp == 0)
      return (void *)p;
    if (cmp > 0) {	/* key > p: move right */
      base = (const char *)p + size;
      lim--;
    } /* else move left */
  }
  return (NULL);
}


/* a utility Javascript function that sets the values of HTML entities in
the browser with the current parameter values known to the Arduino. */

void initter (WiFiClient client, int idx)
{
  client.println ("{");
  String holder = String (parms[idx].name) + " = "
    + String (parms[idx].val, 2) + ";";
  client.println (holder);
  
  holder = String ("if (searchParams.has('") + parms[idx].name  + "'))";
  client.println (holder);

  holder = String (parms[idx].name) + " = searchParams.get('" +
    parms[idx].name + "');";
  client.println (holder);

  holder = String ("document.getElementById('")
    +  parms[idx].name + "').value = "
    + String(parms[idx].val) + ";";
  client.println (holder);
  client.println ("}");
}

/* The long, hairy, process of building the web page */

void buildPage (WiFiClient client)
{
  
	    /***** scripts *******/

/* scripts are Javascript routines that do stuff */


  client.println (F("<script>"));

  /********** window.onload *****/

/* immediately the web page is built, initialise the values of HTML entities */

  client.println (F("window.onload = function () {"));
  client.println (F("  let srch = window.location.search;"));
  client.println (F("  const searchParams =  new URLSearchParams(srch);"));

  for (int i = 0; i < lbl_cnt; i++)
    initter (client, i);

  client.println ("}"); // end window.onload function

  /********** updateParam *****/

/* send data from the web page to the Arguino */
  
  client.println (F("function updateParam(el) {"));

  client.println (F("const XHR = new XMLHttpRequest();"));

  client.println ("  var text = window.location.origin + \"?update=\" + \
el.id + \"=\" + el.value;");
  client.println (F("XHR.open('POST', text);"));
  client.println (F("XHR.setRequestHeader('Content-Type', 'text/plain');"));
  client.println (F("XHR.send();"));

  client.println ("}");


	    /**** function keyHandler(el) ****/

/* The HTML mostly consists of numeric input boxes that can have up/down
arrows clicked, or numbers pounded in followed by an "enter" keypress.  This
handles the enter key. */

  client.println (F("function keyHandler(el) {"));
  client.println (F("  if (event.keyCode=='13'){"));
  client.println (F("    updateParam (el);"));
  client.println (F("    return false;"));
  client.println (F("  }"));
  client.println (F("  return true;"));
  client.println ("}");
	    
  
  client.println ("</script>");

	    /***** end scripts *******/

  /********** styles ***********/

/* styles control what the HTML looks like */

  client.println("<style>");

  client.println("th, td {");
  client.println(F("  padding-top: 0px;"));
  client.println(F("  padding-bottom: 0px;"));
  client.println(F("  padding-left: 10px;"));
  client.println(F("  padding-right: 0px;"));
  client.println("}");
  
  client.println(F("div {"));
  client.println(F("  width: 820px;"));
  client.println(F("  background-color: lightgrey;"));
  client.println(F("  border: 15px lightgrey;"));
  client.println(F("  padding: 15px;"));
  client.println(F("  margin: 20px;"));
  client.println("}");
  
  client.println("</style>");


/* the actual page layout */

	    /*** begin page ****/
	    
  client.println (F("<h1>Stewart</h2>"));
	    
	    
	    /****  forms ****/

/* A couple of truly hairy macros that creae the numeric input boxes.
DO NOT FIDDLE WITH THESE!  You'll wind up tearing your hair out with
frustration and you're to young to be bald. */

#define BUILD_ETY(id,lbl,stp,fm)		      \
  client.println ("<td style=\"text-align:right\">"); \
  client.println (  "<label for=\"" #id "\">" #lbl "</label>"); \
  client.println ("</td>"); \
  client.println ("<td style=\"text-align:right\">"); \
  client.println (  "<input type=\"number\" id=\"" #id"\" step=\"" #stp "\" onwheel=\"updateParam(this);\"  onchange=\"updateParam(this);\">"); \
  client.println ("</td>")
	    
#define BUILD_ETYM(id,lbl,stp,fm)                              \
  client.println ("<td style=\"text-align:right\">"); \
  client.println (  "<label for=\"" #id "\">" #lbl "</label>"); \
  client.println ("</td>"); \
  client.println ("<td style=\"text-align:right\">"); \
  client.println (  "<input type=\"number\" id=\"" #id"\" step=\"" #stp "\" onchange=\"updateParam(this);\" onwheel=\"updateParam(this);\" onkeypress=\"return keyHandler(this);\" min=\"0\">"); \
  client.println ("</td>")
  
	    /**** position form ****/

  client.println (F("<div>"));			// start position form
  client.println (F("<h2>Position</h2>"));

  client.println (F("<form id=\"position\" method=\"get\">"));

  client.println (F("<table>"));

  client.println (F("  <tr>"));		// start pos offset row

  client.println (F("    <td style=\"text-align:right\">"));
  client.println (F("(cm)"));
  client.println (F("    </td>"));

  BUILD_ETY (pdx, dx, 0.1, position);
  BUILD_ETY (pdy, dy, 0.1, position);
  BUILD_ETY (pdz, dz, 0.1, position);

  client.println (F("  </tr>"));		// end pos offset row
  
  client.println (F("  <tr>"));		// start pos attitude row

  client.println (F("    <td style=\"text-align:right\">"));
  client.println (F("(deg)"));
  client.println (F("    </td>"));
	    
  BUILD_ETY (proll,  roll,  1.0, position);
  BUILD_ETY (ppitch, pitch, 1.0, position);
  BUILD_ETY (pyaw,   yaw,   1.0, position);
  
  client.println (F("  </tr>"));		// end pos attitude row

  client.println (F("</table>"));

  client.println (F("</form>"));

  client.println (F("</div>"));		// end position form
	    


	    /**** jitter form ****/

  client.println (F("<div>"));		// start jitter form
  client.println (F("<h2>Jitter</h2>"));
	    
  client.println F(("<form id=\"jitter\" method=\"get\">"));
	    
  client.println (F("<table>"));
  
  client.println (F("  <tr>"));

  client.println (F("    <td style=\"text-align:right\">"));
  client.println (F("(cm)"));
  client.println (F("    </td>"));

  BUILD_ETY (jdx, dx, 0.1, jitter);
  BUILD_ETY (jdy, dy, 0.1, jitter);
  BUILD_ETY (jdz, dz, 0.1, jitter);
  
  client.println (F("  </tr>"));

  client.println (F("  <tr>"));

  client.println (F("    <td style=\"text-align:right\">"));
  client.println (F("(deg)"));
  client.println (F("    </td>"));

  BUILD_ETY (jroll,  roll,  1.0, jitter);
  BUILD_ETY (jpitch, pitch, 1.0, jitter);
  BUILD_ETY (jyaw,   yaw,   1.0, jitter);
  
  client.println (F("  </tr>"));
  
  client.println (F("</table>"));
	 
  client.println (F("</form>"));
  
  client.println (F("</div>"));		// end jitter form
  


	    /**** time form ****/

  client.println (F("<div>"));
  client.println (F("<h2>Time</h2>"));
	    
  client.println (F("<form id=\"time\" method=\"get\">"));
	    
  client.println (F("<table>"));

  client.println (F("  <tr>"));

  client.println (F("    <td style=\"text-align:right\">"));
  client.println (F("(secs)"));
  client.println (F("    </td>"));

  BUILD_ETYM (onset,    onset,    0.1, time);
  BUILD_ETYM (relax,    relax,    0.1, time);
  BUILD_ETYM (interval, interval, 0.1, time);

  client.println (F("  </tr>"));

  client.println (F("</table>"));

  client.println(F("</form>"));

  client.println (F("</div>"));		// end time form



  client.println (F("<script>"));
  client.println (F("function updateScript() {"));

  client.println (F("const XHR = new XMLHttpRequest();"));

  client.println ("  var fn = document.getElementById('fname').value");

  client.println ("  if (0 == fn.length) {");
  client.println ("    alert('It helps if you provide a name.')");
  client.println ("    return;");
  client.println ("  }");

  client.println ("  var parms = \
position.pdx.value + \
',' + position.pdy.value + \
',' + position.pdz.value +   \
',' + position.proll.value +  \
',' + position.ppitch.value + \
',' + position.pyaw.value +   \
',' + jitter.jdx.value +      \
',' + jitter.jdy.value +      \
',' + jitter.jdz.value +      \
',' + jitter.jroll.value +    \
',' + jitter.jpitch.value +   \
',' + jitter.jyaw.value +     \
',' + time.onset.value +      \
',' + time.relax.value +      \
',' + time.interval.value			\
;");
  client.println ("  var text = window.location.origin + \"?script:\" + \
fn + '=' + parms + ';' ");
  client.println (F("XHR.open('POST', text);"));
  client.println (F("XHR.setRequestHeader('Content-Type', 'text/plain');"));
  client.println (F("XHR.send();"));

  client.println (F("var el = document.getElementById(\"scriptSel\");"));
  client.println (F("var option = document.createElement(\"option\");"));
  client.println (F("option.text = fn;"));
  client.println (F("option.value = parms;"));
  client.println (F("option.onclick = \"scriptClick(this);\";"));
  client.println (F("el.add(option);"));

  client.println ("}");		// end updateScript
  client.println (F("</script>"));	

  client.println (F("<div>"));

/****** Save script name ********/

  client.println (F("<label for=\"fname\">Save script</label>"));
  client.println (F("<input type=\text\" id=\"fname\" name=\"fname\"/ \
onkeyup=\"this.value = this.value.toUpperCase();\" \
pattern=\"[\\w]\" maxlength=\"8\" >"));


/****** Save script button ********/

  client.println (F("<input type=\"button\" value=\"Save\" \
onclick=\"updateScript()\">"));

/****** Save script function ********/

  client.println (F("<script>"));
  client.println (F("function scriptClick(el) {"));
  client.println (F("  if (deleteButton.value === \"Select\") {"));
  client.println ("      var val = el.value;");
  client.println ("      var vals = val.split(',');");
  client.println ("      position.pdx.value    = vals[0];");
  client.println ("      position.pdy.value    = vals[1];");
  client.println ("      position.pdz.value    = vals[2];");
  client.println ("      position.proll.value  = vals[3];");
  client.println ("      position.ppitch.value = vals[4];");
  client.println ("      position.pyaw.value   = vals[5];");
  client.println ("      jitter.jdx.value      = vals[6];");
  client.println ("      jitter.jdy.value      = vals[7];");
  client.println ("      jitter.jdz.value      = vals[8];");
  client.println ("      jitter.jroll.value    = vals[9];");
  client.println ("      jitter.jpitch.value   = vals[10];");
  client.println ("      jitter.jyaw.value     = vals[11];");
  client.println ("      time.onset.value      = vals[12];");
  client.println ("      time.relax.value      = vals[13];");
  client.println ("      time.interval.value   = vals[14];");
  client.println (F("    const XHR = new XMLHttpRequest();"));
  client.println ("      var text = window.location.origin + \
\"?loadParms=\" + val;");
  client.println (F("     XHR.open('POST', text);"));
  client.println (F("     XHR.setRequestHeader('Content-Type', \
 'text/plain');"));
  client.println (F("     XHR.send();"));
  client.println (F ("  } else {"));		// deleting
  client.println (F("     const XHR = new XMLHttpRequest();"));
  client.println (F ("    var lbl = el.options[el.selectedIndex].innerHTML;"));
  client.println (F ("    el.remove(el.selectedIndex);"));
  client.println ("       var text = window.location.origin + \
\"?deleteScript=\" + lbl;");
  client.println (F("     XHR.open('POST', text);"));
  client.println (F("     XHR.setRequestHeader('Content-Type', \
 'text/plain');"));
  client.println (F("     XHR.send();"));
  client.println (F ("  }"));
  client.println (F ("}"));
  client.println (F("</script>"));

/****** Save script select box ********/

  client.println (F("<label for=\"scriptSel\">Load script:</label>"));
  client.println (F("<select name=\"scriptSel\" id=\"scriptSel\" \
onclick=\"scriptClick(this);\" \
>"));

  File root = SD.open("/");
  if (root) {
    while (true) {
      File entry =  root.openNextFile();
      if (!entry) break;
      char *fn = entry.name();
      int byteCount = entry.available();
      char *buf = (char *)alloca (byteCount);
      entry.read(buf, byteCount);
      entry.close();
      client.println ("<option value=\"" + String (buf) \
		      + "\" label= \"" + fn + "\"   \
onclick=\"scriptClick(this);\" \
>" + fn + "</option>");
    }
    root.close();
  }

  client.println (F("</select>"));

/****** delete script button style ********/

  client.println (F("<style>"));
  client.println (F(".button {"));
  client.println (F("  text-align: center;"));
  client.println (F("  text-decoration: none;"));
  client.println (F("  color: #000;"));
  client.println (F("  background-color: green;"));
  client.println (F("}"));
  
  client.println (F(".rsbutton {"));
  client.println (F("  text-align: center;"));
  client.println (F("  text-decoration: none;"));
  client.println (F("  color: #000;"));
  client.println (F("  background-color: red;"));
  client.println (F("}"));
  client.println (F("</style>"));

/****** delete script and run/stop function ********/
  client.println (F("<script>"));

  client.println (F("function deleteScript(el) {"));
  client.println (F("  if (deleteButton.value === \"Select\") {"));
  client.println (F("     deleteButton.value = 'Delete';"));
  client.println (F("     document.getElementById(\"deleteButton\").style.backgroundColor = \"red\";"));
  client.println (F("  } else {"));
  client.println (F("     deleteButton.value = 'Select';"));
  client.println (F("     document.getElementById(\"deleteButton\").style.backgroundColor = \"green\";"));
  client.println (F("  }"));
  client.println (F("}"));

  client.println (F("function runStopScript(el) {"));
  client.println (F("  const XHR = new XMLHttpRequest();"));
  client.println (F("  var text"));
  client.println (F("  if (runStopButton.value === \"Stop\") {"));
  client.println (F("     runStopButton.value = 'Run';"));
  client.println (F("     document.getElementById(\"runStopButton\").style.backgroundColor = \"red\";"));
  client.println ("       text = window.location.origin + \"?runState=stop\";");
  client.println (F("  } else {"));
  client.println (F("     runStopButton.value = 'Stop';"));
  client.println (F("     document.getElementById(\"runStopButton\").style.backgroundColor = \"green\";"));
  client.println ("       text = window.location.origin + \"?runState=run\";");
  client.println (F("  }"));
  client.println (F("XHR.open('POST', text);"));
  client.println (F("XHR.setRequestHeader('Content-Type', 'text/plain');"));
  client.println (F("XHR.send();"));
  client.println (F("}"));

  client.println (F("function demoNormScript(el) {"));
  client.println (F("  const XHR = new XMLHttpRequest();"));
  client.println (F("  var text"));
  client.println (F("  if (demoNormButton.value === \"Demo\") {"));
  client.println (F("     demoNormButton.value = 'Norm';"));
  client.println (F("     document.getElementById(\"demoNormButton\").style.backgroundColor = \"red\";"));
  client.println ("       text = window.location.origin + \"?demoState=norm\";");
  client.println (F("  } else {"));
  client.println (F("     demoNormButton.value = 'Demo';"));
  client.println (F("     document.getElementById(\"demoNormButton\").style.backgroundColor = \"green\";"));
  client.println ("       text = window.location.origin + \"?demoState=demo\";");
  client.println (F("  }"));
  client.println (F("XHR.open('POST', text);"));
  client.println (F("XHR.setRequestHeader('Content-Type', 'text/plain');"));
  client.println (F("XHR.send();"));
  client.println (F("}"));

client.println (F("</script>"));

/****** delete script button ********/

  client.println (F("<input id=\"deleteButton\" type=\"button\" \
onclick=\"deleteScript(this);\" class=\"button\" \
value=\"Select\">"));

/****** run/stop button ********/

  client.println (F("<input id=\"runStopButton\" type=\"button\" \
onclick=\"runStopScript(this);\" class=\"rsbutton\" \
value=\"Run\">"));

/****** demo/norm button ********/

  client.println (F("<input id=\"demoNormButton\" type=\"button\" \
onclick=\"demoNormScript(this);\" class=\"rsbutton\" \
value=\"Norm\">"));

  client.println (F("</div>"));
  
	    /***** end of forms ********/
}	// end buildPage

/***

Normal C/C++ code has one entry point, called main(), where execution starts.
Arduino code as two such entry points, setup() and loop().  setup() is run once
to, guess what, set things up.  loop(), guess what again, loops forever.

***/

void setup()
{

  int status = WL_IDLE_STATUS;

  // check for the WiFi module:

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:

  Serial.print("Connecting to ");
  Serial.println(ssid);                   // print the network name (SSID);
  delay(1000);   // wait 1 second
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    Serial.println("Waiting...");
    delay(1000);   // wait 10 seconds for connection:
  }

  ip = WiFi.localIP();
  Serial.println (ip);

  server.begin();    // start the web server on port 80
  
  Serial.println("Connected...");

  if (!SD.begin(4)) {
    Serial.println("SD initialization failed!");
    while (1);
  }

  

/* create the parameter array and compute the hash values */

  parms = (parm_s *)malloc (sizeof(parm_s) * lbl_cnt);
  for (int i = 0; i < lbl_cnt; i++) {
    parms[i].val  = 0.0;
    parms[i].hash = joaat.encode_str (JOAAT_STR (lbls[i]));
    parms[i].idx  = i;
    parms[i].name = lbls[i];
  }

/* sort the array by hash value */

  qsort (parms, sizeof(parm_s), 0, lbl_cnt - 1, cmp_parm);
  plut = (int *)malloc (lbl_cnt * sizeof(int));
  for (int i = 0; i < lbl_cnt; i++) {
    plut[parms[i].idx] = i;
  }

  myServo[0].attach( SERVO0_PIN);
  myServo[1].attach( SERVO1_PIN);
  myServo[2].attach( SERVO2_PIN);
  myServo[3].attach( SERVO3_PIN);
  myServo[4].attach( SERVO4_PIN);
  myServo[5].attach( SERVO5_PIN);

  // compute base height
  h0 = 0.0;					// Eq 10
  for (int i = 0; i < 6; i++) {
    incr[i] = ((double)random(-1000, 1000))/500.0;
    double xp = myPlatform.anchor[i].x;
    double yp = myPlatform.anchor[i].y;
    double xb = myBase.pos[i].x;
    double yb = myBase.pos[i].y;
    h0 += sqrt ( (LEG_LENGTH * LEG_LENGTH) +
		    (ARM_LENGTH * ARM_LENGTH) -
		  ( ((xp - xb) * (xp - xb)) +
		    ((yp - yb) * (yp - yb))));
  }
  h0 /= 6.0;
  h0 *= 1.2;		// ad-hox adjustment

  update_alpha ();
  pincr = ((double)random(1000))/10000.0;
}


enum {
  JITTER_QUIET,
  JITTER_ONSET,
  JITTER_ONSET_CONTINUE,
  JITTER_RELAX
};

#define TO_US(s) ((unsigned int)((s) * 1000000))
int jitter_mode = JITTER_ONSET;

static int cnt = 0;
static void do_demo()
{
#define SETPOS 90
  static int pos[]   = {SETPOS, SETPOS, SETPOS, SETPOS, SETPOS, SETPOS};
  static int incr[] = {1, 3, 2, 4, -2, -4};

  for (int i = 0; i < NUM_SERVOS; i++) {
    
    pos [i] += incr[i];
    if (pos[i] < 0) {
      incr[i] = -incr[i];
      pos[i] = -pos[i];
    }
    else if (pos[i] > 180) {
      incr[i] = -incr[i];
      pos[i] = 360 - pos[i];
    }
    // Serial.println(String(i) + "  " + String(pos[i]));    
    myServo[i].write((i&1) ? pos[i] : 180-pos[i]);
  }
  delay(30);
}

static void do_jitter()
{
  unsigned int sleep_time = TO_US(parms[plut[PARM_interval]].val);
  static double target_dx;
  static double target_dy;
  static double target_dz;
  static double target_ap;
  static double target_ay;
  static double target_ar;
  static double attack_stage;
  static double base_dx;
  static double base_dy;
  static double base_dz;
  static double base_ap;
  static double base_ay;
  static double base_ar;
  
  switch (jitter_mode) {
  case JITTER_QUIET:		// do nothing
    break;
  case JITTER_ONSET:
    base_dx = parms[plut[PARM_pdx]].val;
    base_dy = parms[plut[PARM_pdy]].val;
    base_dz = parms[plut[PARM_pdz]].val;
    base_ap = parms[plut[PARM_ppitch]].val;
    base_ay = parms[plut[PARM_pyaw]].val;
    base_ar = parms[plut[PARM_proll]].val;

    target_dx = random ((double)(-1000, 1000))/ 1000.0;
    target_dy = random ((double)(-1000, 1000))/ 1000.0;
    target_dz = random ((double)(-1000, 1000))/ 1000.0;
    target_ap = random ((double)(-1000, 1000))/ 10000.0;
    target_ay = random ((double)(-1000, 1000))/ 10000.0;
    target_ar = random ((double)(-1000, 1000))/ 10000.0;
    target_dx *= parms[plut[PARM_jdx]].val;
    target_dy *= parms[plut[PARM_jdy]].val;
    target_dz *= parms[plut[PARM_jdz]].val;
    target_ap *= parms[plut[PARM_jpitch]].val;
    target_ar *= parms[plut[PARM_jroll]].val;
    target_ay *= parms[plut[PARM_jyaw]].val;
    attack_stage = 0.0;
    jitter_mode = JITTER_ONSET_CONTINUE;
    //break;					no break
  case JITTER_ONSET_CONTINUE:
    if (attack_stage < 10.0) {
      parms[plut[PARM_pdx]].val    = base_dx + target_dx * attack_stage / 10.0;
      parms[plut[PARM_pdy]].val    = base_dy + target_dy * attack_stage / 10.0; 
      parms[plut[PARM_pdz]].val    = base_dz + target_dz * attack_stage / 10.0; 
      parms[plut[PARM_ppitch]].val = base_ap + target_ap * attack_stage / 10.0;
      parms[plut[PARM_pyaw]].val   = base_ay + target_ay * attack_stage / 10.0; 
      parms[plut[PARM_proll]].val  = base_ar + target_ar * attack_stage / 10.0; 
      attack_stage += 0.2;
    }
    else jitter_mode = JITTER_RELAX;
    break;
  case JITTER_RELAX:
    if (attack_stage > 0.0) {
      parms[plut[PARM_pdx]].val    = target_dx * attack_stage / 10.0;
      parms[plut[PARM_pdy]].val    = target_dy * attack_stage / 10.0; 
      parms[plut[PARM_pdz]].val    = target_dz * attack_stage / 10.0; 
      parms[plut[PARM_ppitch]].val = target_ap * attack_stage / 10.0;
      parms[plut[PARM_pyaw]].val   = target_ay * attack_stage / 10.0; 
      parms[plut[PARM_proll]].val  = target_ar * attack_stage / 10.0; 
      attack_stage -= 0.1;
      sleep_time = TO_US(parms[plut[PARM_relax]].val/10.0);
    }
    else {
      jitter_mode = JITTER_ONSET;
      sleep_time = TO_US(parms[plut[PARM_interval]].val);
    }
    break;
  }
  update_alpha();
  delayMicroseconds(sleep_time);
}
              
void loop() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    // make a String to hold incoming data from the client
    String currentLine = "";
    while (client.connected()) {    // loop while the client's connected
      if (client.available()) {     // if there's bytes to read from the client,
	
        char c = client.read();     // read a byte, then
//	Serial.print (c);
        if (c == '\n') {            // if the byte is a newline character
          if (currentLine.length() == 0) {
	    // if it's blank line, no more input and go ahead and build the
	    // page.
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
	    
	    buildPage (client);

	    client.println ();
            break;
          }		// if empty line
	  else {	// non-empty line
	    if (currentLine.startsWith("POST")) {
	      // if it's incoming data, figure out what kind of data it is...
	      String startString = "update=";
	      int startPos = currentLine.indexOf(startString);
	      if (-1 != startPos) {	// if it's update data...
		// if it's parameter update data...
		startPos += startString.length ();
		String endString = "HTTP";
		int endPos = currentLine.indexOf(endString);
		if (-1 != endPos) {
		  // isolate the actual data
		  endPos--;
		  String text = currentLine.substring (startPos, endPos);
		  // and then split up up into parameter name
		  // and parameter value
		  int endPos =  text.indexOf ("=");
		  String vbl = text.substring (0, endPos);
		  String vals = text.substring (1 + endPos);
		  double val = vals.toDouble ();

		  // compute the hash of the name...
		  uint32_t hash = joaat.encode_str (JOAAT_STR (vbl.c_str ()));
		  // and search for it
		  void *res = bsearch (reinterpret_cast<void *>(hash), parms,
			 lbl_cnt, sizeof(parm_s), cmp_parm_str);
		  if (res != nullptr) {
		    // if you find it, update the value
		    ((parm_s *)res)->val = val;
		    update_alpha ();
		  }
		}
	      }
	      else {		// not update
		String startString = "script:";
		int startPos = currentLine.indexOf(startString);
		if (-1 != startPos) {	// if it's script data...
		  int endPos = currentLine.indexOf('=');
		  if (-1 != endPos) {
		    endPos--;
		    String fn   =
		      currentLine.substring (startPos + startString.length (),
					     endPos + 1);
		    int lastPos = currentLine.indexOf(';');
		    String data = currentLine.substring (endPos + 2, lastPos);
		    if (SD.exists (fn)) SD.remove (fn);
		    File script = SD.open (fn, FILE_WRITE);
		    script.write (data.c_str (), 1 + data.length ());
		    script.close ();
		  }
		}
		else {	// not script=
		  String startString = "deleteScript=";
		  int startPos = currentLine.indexOf(startString);
		  if (-1 != startPos) {		// if it's a deleteScript req
		    int endPos = currentLine.indexOf(" HTTP");
		    if (-1 != endPos) {
		      endPos--;
		      String fn   =
			currentLine.substring (startPos +
					       startString.length (),
					       endPos + 1);
		      SD.remove (fn);
		    }
		  }
		  else {		// not delete script
		    String startString = "runState=";
		    int startPos = currentLine.indexOf(startString);
		    if (-1 != startPos) {	// if it's a runstop req
		      int stopit = currentLine.indexOf("stop");
		      run = (-1 == stopit) ? true : false;
		    }
		    else {
		      String startString = "demoState=";
          Serial.println(currentLine);
		      int startPos = currentLine.indexOf(startString);
		      if (-1 != startPos) {	// if it's a dem req
			      int demoit = currentLine.indexOf("norm");
			      demo = (-1 == demoit) ? true : false;
		      }
		    }
		  }
		}
	      }			// end not update
	    }
	  }
	  currentLine = "";
        }			// if '\n'
	else if (c != '\r') {
          currentLine += c;      // add it to the end of the currentLine
        }
      }				// if data available
    }				// if connected
    client.stop();
  }				// if client

  if (run) {
    if (demo) do_demo ();
    else do_jitter ();
  }
}				// end of loop


