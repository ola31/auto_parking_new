#include "ros/ros.h"
#include "std_msgs/String.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

#define MAX_RANGE 10

#define LEFT_ANG 90
#define RIGHT_ANG 270

#define DEG2RAD 0.0174533
#define RAD2DEG 180.0/3.141592

//#define SIDE_DEG 45
int SIDE_DEG = 58; //(int)(RAD2DEG*atan(1.2/0.75)); = 57.99

#define TRIANGLE 1.0//0.31 //0.34

#define PRESENT_PAST_RATIO 0.7

#define MAX_LINEAR_VEL 0.7 //2.0
#define MAX_ANGULAR_Z 2.0

#define Robot_Width 0.55

#define LOOP_RATE 30

#define PARKING_AREA_TIME 3

#define PARKING_AREA_TRIAGNLE (0.6)*(1.2)*0.5


/*********************
 * S = 0.5 * (r*r) *theta
 * ********************/

int phase = 0;

float laserscan_arr[180]={0};

float pre_linear_x = 0.0;
float pre_angular_z = 0.0;
float linear_x=0;
float angular_z=0;

float Kp = 15;
float phase2_Kp = 0.005;
float phase2_Kp_angular = 10.0;

float sum_all_scan = 0;
float stop_threshold = 70;

int wait =0;
int wait_threshold = (float)PARKING_AREA_TIME/((float)1.0/(float)LOOP_RATE);

int phase2_count = 0;
int phase4_count =0;
int phase5_count=0;

float yd_laserscan_arr[720]={0};

//float arrarrarr[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.6440000534057617, 0.0, 0.0, 2.246999979019165, 2.25, 2.253000020980835, 2.256999969482422, 0.0, 2.256999969482422, 2.256999969482422, 2.257999897003174, 2.259999990463257, 2.263000011444092, 0.0, 2.5429999828338623, 2.378999948501587, 2.311000108718872, 2.243000030517578, 2.187000036239624, 0.0, 2.178999900817871, 2.187999963760376, 2.196000099182129, 2.2039999961853027, 2.2119998931884766, 2.2239999771118164, 0.0, 2.2339999675750732, 2.24399995803833, 2.253999948501587, 0.0, 1.7630000114440918, 1.7079999446868896, 1.7070000171661377, 0.0, 1.7070000171661377, 1.715999960899353, 1.7259999513626099, 1.7359999418258667, 1.746000051498413, 0.0, 1.7569999694824219, 1.7699999809265137, 0.0, 0.0, 1.0299999713897705, 0.0, 1.024999976158142, 0.0, 0.0, 1.878999948501587, 1.9670000076293945, 2.053999900817871, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3980000019073486, 0.0, 1.4320000410079956, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1720000505447388, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.215999960899353, 0.0, 0.0, 0.0, 1.2050000429153442, 0.0, 0.0, 0.0, 1.1950000524520874, 1.1929999589920044, 1.190999984741211, 1.1929999589920044, 0.0, 1.190000057220459, 1.187999963760376, 1.187000036239624, 1.1859999895095825, 1.1799999475479126, 1.1829999685287476, 0.0, 1.180999994277954, 1.1790000200271606, 1.1779999732971191, 1.1790000200271606, 1.1770000457763672, 0.0, 1.1740000247955322, 1.1749999523162842, 1.1749999523162842, 1.1759999990463257, 1.1770000457763672, 0.0, 1.1770000457763672, 1.1759999990463257, 1.1759999990463257, 1.1779999732971191, 1.1779999732971191, 0.0, 1.1770000457763672, 1.1779999732971191, 1.1790000200271606, 1.1820000410079956, 1.180999994277954, 1.184999942779541, 0.0, 1.187999963760376, 1.190999984741211, 1.187999963760376, 1.187999963760376, 0.0, 0.0, 0.8489999771118164, 0.8370000123977661, 0.8399999737739563, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.35100001096725464, 0.0, 0.3529999852180481, 0.3569999933242798, 0.36000001430511475, 0.36399999260902405, 0.0, 0.367000013589859, 0.3709999918937683, 0.37400001287460327, 0.37700000405311584, 0.0, 0.31200000643730164, 0.0, 0.0, 0.0, 0.0, 0.0, 0.30300000309944153, 0.0, 0.0, 0.0, 0.31299999356269836, 0.31700000166893005, 0.32199999690055847, 0.0, 0.32600000500679016, 0.3310000002384186, 0.335999995470047, 0.0, 0.3409999907016754, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.515999972820282, 0.5099999904632568, 0.0, 0.0, 0.0, 0.503000020980835, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4970000088214874, 0.0, 0.453000009059906, 0.45500001311302185, 0.4569999873638153, 0.45100000500679016, 0.0, 0.0, 0.4819999933242798, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5230000019073486, 0.5260000228881836, 0.0, 0.527999997138977, 0.0, 0.5669999718666077, 0.0, 0.0, 0.0, 0.0, 0.5490000247955322, 0.5479999780654907, 0.0, 0.0, 0.0, 0.5730000138282776, 0.5740000009536743, 0.5740000009536743, 0.5730000138282776, 0.0, 0.0, 0.5230000019073486, 0.0, 0.5189999938011169, 0.49300000071525574, 0.48899999260902405, 0.48500001430511475, 0.4869999885559082, 0.4869999885559082, 0.49000000953674316, 0.0, 0.4909999966621399, 0.49300000071525574, 0.4830000102519989, 0.0, 0.0, 0.0, 0.0, 0.4429999887943268, 0.0, 0.0, 0.4339999854564667, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3540000021457672, 0.0, 0.0, 0.3619999885559082, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.33799999952316284, 0.33500000834465027, 0.3330000042915344, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3109999895095825, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.36399999260902405, 0.36000001430511475, 0.3569999933242798, 0.3540000021457672, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6589999794960022, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6240000128746033, 0.625, 0.0, 0.625, 0.6230000257492065, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7459999918937683, 0.7450000047683716, 0.7459999918937683, 0.0, 0.0, 0.0, 0.0, 0.0, 2.2639999389648438, 2.24399995803833, 2.2899999618530273, 2.1579999923706055, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6009999513626099, 0.0, 1.5820000171661377, 1.6019999980926514, 1.6230000257492065, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.2890000343322754, 0.0, 0.0, 0.0, 0.0, 1.50600004196167, 0.0, 0.0, 0.0, 0.0, 3.369999885559082, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9420000314712524, 0.0, 0.0, 0.0, 3.5160000324249268, 0.0, 3.5290000438690186, 3.2980000972747803, 0.0, 0.0, 1.871000051498413, 1.8289999961853027, 1.8179999589920044, 1.812999963760376, 0.0, 0.0, 0.0, 3.757999897003174, 0.0, 0.0, 0.0, 1.6799999475479126, 1.6749999523162842, 0.0, 1.6799999475479126, 0.0, 0.0, 0.0, 0.0, 2.4210000038146973, 0.0, 0.0, 0.0, 0.0, 2.3289999961853027, 2.3440001010894775, 0.0, 0.0, 0.0, 1.8140000104904175, 1.7940000295639038, 0.0, 1.7949999570846558, 1.815999984741211, 1.8359999656677246, 1.8550000190734863, 1.9980000257492065, 0.0, 2.005000114440918, 0.0, 0.0, 0.0, 3.986999988555908, 3.9739999771118164, 3.9649999141693115, 3.9649999141693115, 3.9519999027252197, 0.0, 3.938999891281128, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.819999933242798, 0.0, 0.0, 0.0, 3.371000051498413, 3.388000011444092, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.7309999465942383, 0.0, 0.0, 0.0, 0.0, 3.7160000801086426, 3.7100000381469727, 0.0, 0.0, 0.0, 0.0, 0.0, 3.683000087738037, 3.674999952316284, 3.6670000553131104, 3.680999994277954, 3.6740000247955322, 3.678999900817871, 0.0, 3.678999900817871, 3.677999973297119, 3.6649999618530273, 3.6579999923706055, 3.6440000534057617, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7889999747276306, 0.7889999747276306, 0.0, 0.7860000133514404, 0.0, 0.0, 0.0, 0.0, 2.4189999103546143, 2.440999984741211, 0.0, 2.437000036239624, 0.0, 0.0, 1.5429999828338623, 0.0, 2.381999969482422, 0.0, 2.4590001106262207, 2.4779999256134033, 2.496999979019165, 2.5450000762939453, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.569999933242798, 0.0};
//float arrarrarr[]={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.631999969482422, 0.0, 0.0, 2.247999906539917, 2.253000020980835, 2.252000093460083, 0.0, 2.253999948501587, 2.255000114440918, 2.256999969482422, 2.257999897003174, 2.259999990463257, 0.0, 2.4000000953674316, 2.5399999618530273, 2.36899995803833, 2.2929999828338623, 2.2330000400543213, 0.0, 2.2060000896453857, 2.178999900817871, 2.188999891281128, 2.197999954223633, 2.2060000896453857, 0.0, 2.2139999866485596, 2.2219998836517334, 2.2360000610351562, 2.247999906539917, 0.0, 0.0, 1.7410000562667847, 0.0, 1.7020000219345093, 1.6979999542236328, 1.7100000381469727, 1.7200000286102295, 1.7300000190734863, 0.0, 1.7419999837875366, 1.75, 1.7610000371932983, 0.0, 0.0, 1.0379999876022339, 0.0, 1.027999997138977, 0.0, 0.0, 0.0, 1.8860000371932983, 1.906000018119812, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2599999904632568, 1.2410000562667847, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.2170000076293945, 1.215000033378601, 0.0, 1.2130000591278076, 1.2109999656677246, 1.2029999494552612, 1.2039999961853027, 1.2000000476837158, 1.1959999799728394, 0.0, 1.1950000524520874, 1.1959999799728394, 1.194000005722046, 1.190999984741211, 1.187999963760376, 0.0, 1.187999963760376, 1.187000036239624, 1.184999942779541, 1.1820000410079956, 1.1790000200271606, 1.1820000410079956, 1.1779999732971191, 0.0, 1.1779999732971191, 1.1779999732971191, 1.1790000200271606, 1.1770000457763672, 1.1749999523162842, 0.0, 1.1749999523162842, 1.1759999990463257, 1.1770000457763672, 1.1759999990463257, 1.1790000200271606, 0.0, 1.1790000200271606, 1.1770000457763672, 1.1759999990463257, 1.1770000457763672, 1.1759999990463257, 1.1799999475479126, 0.0, 1.180999994277954, 1.1829999685287476, 1.1829999685287476, 1.184000015258789, 1.190000057220459, 0.0, 0.0, 0.0, 0.8560000061988831, 0.0, 0.8420000076293945, 0.8429999947547913, 0.8429999947547913, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3540000021457672, 0.35600000619888306, 0.0, 0.3580000102519989, 0.3630000054836273, 0.3659999966621399, 0.3700000047683716, 0.0, 0.37299999594688416, 0.37599998712539673, 0.0, 0.32199999690055847, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3009999990463257, 0.30399999022483826, 0.30799999833106995, 0.0, 0.3109999895095825, 0.31700000166893005, 0.32199999690055847, 0.0, 0.3240000009536743, 0.33000001311302185, 0.335999995470047, 0.0, 0.3400000035762787, 0.3440000116825104, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4429999887943268, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4830000102519989, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5109999775886536, 0.0, 0.5059999823570251, 0.0, 0.5059999823570251, 0.5040000081062317, 0.503000020980835, 0.0, 0.0, 0.0, 0.5009999871253967, 0.4819999933242798, 0.46299999952316284, 0.460999995470047, 0.4560000002384186, 0.45100000500679016, 0.0, 0.0, 0.0, 0.0, 0.6489999890327454, 0.0, 0.0, 0.0, 0.5139999985694885, 0.5170000195503235, 0.0, 0.5260000228881836, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.550000011920929, 0.0, 0.0, 0.0, 0.5770000219345093, 0.5730000138282776, 0.5740000009536743, 0.5730000138282776, 0.5720000267028809, 0.0, 0.5830000042915344, 0.0, 0.5230000019073486, 0.5080000162124634, 0.49300000071525574, 0.49000000953674316, 0.4860000014305115, 0.4869999885559082, 0.4880000054836273, 0.0, 0.49000000953674316, 0.492000013589859, 0.49300000071525574, 0.0, 0.0, 0.0, 0.44699999690055847, 0.0, 0.0, 0.0, 0.43799999356269836, 0.43299999833106995, 0.4230000078678131, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3630000054836273, 0.35199999809265137, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3440000116825104, 0.33500000834465027, 0.3319999873638153, 0.32899999618530273, 0.32499998807907104, 0.31700000166893005, 0.31700000166893005, 0.0, 0.0, 0.3059999942779541, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.36399999260902405, 0.3610000014305115, 0.3569999933242798, 0.3540000021457672, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6620000004768372, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5950000286102295, 0.0, 0.0, 0.0, 0.0, 0.5960000157356262, 0.5929999947547913, 0.0, 0.0, 0.0, 0.0, 0.6299999952316284, 0.625, 0.6240000128746033, 0.621999979019165, 0.621999979019165, 0.0, 0.6230000257492065, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.7459999918937683, 0.746999979019165, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.247999906539917, 2.2920000553131104, 0.0, 2.1610000133514404, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.6410000324249268, 1.61899995803833, 0.0, 0.0, 0.0, 3.243000030517578, 0.0, 3.260999917984009, 3.2660000324249268, 3.2750000953674316, 0.0, 0.0, 0.0, 0.0, 1.5210000276565552, 0.0, 0.0, 0.0, 0.0, 0.0, 3.3529999256134033, 0.0, 0.0, 0.0, 1.5440000295639038, 1.5429999828338623, 0.0, 0.0, 0.0, 1.9459999799728394, 0.0, 0.0, 3.4739999771118164, 3.503999948501587, 3.5160000324249268, 3.5280001163482666, 3.2909998893737793, 3.578000068664551, 0.0, 0.0, 1.7910000085830688, 0.0, 1.774999976158142, 1.6859999895095825, 1.6579999923706055, 1.6519999504089355, 1.6449999809265137, 1.649999976158142, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.4230000972747803, 0.0, 0.0, 0.0, 2.3550000190734863, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.9570000171661377, 1.909999966621399, 1.7580000162124634, 1.7259999513626099, 0.0, 1.7410000562667847, 1.7549999952316284, 1.8079999685287476, 1.9420000314712524, 0.0, 0.0, 0.0, 3.9719998836517334, 0.0, 3.9690001010894775, 3.938999891281128, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.364000082015991, 3.4119999408721924, 0.0, 0.0, 0.0, 0.0, 0.0, 3.7300000190734863, 0.0, 3.7290000915527344, 3.7190001010894775, 3.7190001010894775, 3.7190001010894775, 0.0, 3.7100000381469727, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.683000087738037, 3.678999900817871, 3.6740000247955322, 3.6740000247955322, 3.6740000247955322, 0.0, 3.6740000247955322, 3.6670000553131104, 3.6610000133514404, 3.6600000858306885, 3.6389999389648438, 0.0, 0.0, 0.0, 0.0, 0.7889999747276306, 0.7870000004768372, 0.7860000133514404, 0.0, 0.7850000262260437, 0.0, 0.0, 0.0, 2.492000102996826, 2.4630000591278076, 2.434000015258789, 0.0, 1.562000036239624, 0.0, 0.0, 0.0, 2.381999969482422, 0.0, 2.4769999980926514, 0.0, 2.559000015258789, 0.0, 0.0, 0.0, 0.0, 0.0, 2.4809999465942383, 0.0, 0.0, 2.575000047683716, 0.0, 0.0};
//float arrarrarr[]={0.0, 0.0, 3.302999973297119, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.6480000019073486, 0.0, 2.250999927520752, 2.25600004196167, 2.260999917984009, 2.263000011444092, 2.2639999389648438, 2.5409998893737793, 2.36899995803833, 2.23799991607666, 2.187000036239624, 2.191999912261963, 2.2090001106262207, 2.2269999980926514, 2.2360000610351562, 0.0, 1.7059999704360962, 1.7089999914169312, 1.7120000123977661, 1.7330000400543213, 1.75600004196167, 1.7680000066757202, 0.0, 1.0290000438690186, 1.0210000276565552, 1.8960000276565552, 0.0, 0.0, 0.0, 0.0, 1.3839999437332153, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.1759999990463257, 0.0, 1.2350000143051147, 0.0, 0.0, 0.0, 1.2170000076293945, 1.2059999704360962, 0.0, 1.2000000476837158, 1.1970000267028809, 1.194000005722046, 1.1929999589920044, 1.187999963760376, 1.184999942779541, 1.184000015258789, 1.184999942779541, 1.1799999475479126, 1.1759999990463257, 1.1759999990463257, 1.1720000505447388, 1.1740000247955322, 1.1770000457763672, 1.1770000457763672, 1.1759999990463257, 1.1790000200271606, 1.1799999475479126, 1.184000015258789, 1.1829999685287476, 1.1859999895095825, 1.1920000314712524, 0.0, 0.8349999785423279, 0.8389999866485596, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3490000069141388, 0.3529999852180481, 0.35899999737739563, 0.3630000054836273, 0.36899998784065247, 0.37400001287460327, 0.31700000166893005, 0.0, 0.0, 0.30300000309944153, 0.3070000112056732, 0.31299999356269836, 0.31700000166893005, 0.32600000500679016, 0.33000001311302185, 0.33899998664855957, 0.34599998593330383, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.36899998784065247, 0.0, 0.0, 0.0, 0.0, 0.5120000243186951, 0.0, 0.0, 0.0, 0.0, 0.5130000114440918, 0.5, 0.5049999952316284, 0.5230000019073486, 0.515999972820282, 0.5059999823570251, 0.46299999952316284, 0.4560000002384186, 0.44699999690055847, 0.0, 0.0, 0.0, 0.5090000033378601, 0.5230000019073486, 0.546999990940094, 0.5640000104904175, 0.0, 0.5490000247955322, 0.5450000166893005, 0.5839999914169312, 0.5740000009536743, 0.5740000009536743, 0.5759999752044678, 0.5490000247955322, 0.5040000081062317, 0.4860000014305115, 0.4860000014305115, 0.49000000953674316, 0.492000013589859, 0.47099998593330383, 0.4490000009536743, 0.4440000057220459, 0.4429999887943268, 0.42800000309944153, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.5220000743865967, 2.443000078201294, 0.0, 1.5470000505447388, 2.38700008392334, 2.5169999599456787, 2.5420000553131104, 2.552999973297119, 0.0, 0.0, 0.0, 0.0, 0.0};
void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  int i=0;
  float left_triangle=0;
  float right_triangle=0;
/*
  for(i=0;i<180;i++){
    if(isinf(msg->ranges[i+LEFT_ANG])){
      laserscan_arr[i] = MAX_RANGE;
    }
    else laserscan_arr[i] = msg->ranges[i+LEFT_ANG];
  }

  for(i=0;i<SIDE_DEG;i++){
    left_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }
  for(i=180-SIDE_DEG-1;i<180;i++){
    right_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }
*/
yd_laserscan_arr[0]=msg->ranges[0];
  for(i=1;i<719;i++){
    if(msg->ranges[i]==0){
        yd_laserscan_arr[i]=yd_laserscan_arr[i-1];
    }
    else{
      yd_laserscan_arr[i]=msg->ranges[i];
    }
  }

  for(i=0;i<90;i++){
    if(isinf(msg->ranges[i*2])){
      laserscan_arr[89-i] = MAX_RANGE;
    }
    else laserscan_arr[89-i]=yd_laserscan_arr[i*2];
  }
  for(i=269;i<359;i++){
    if(isinf(msg->ranges[i*2])){
      laserscan_arr[89+(359-i)] = MAX_RANGE;
    }
    laserscan_arr[89+(359-i)]=yd_laserscan_arr[i*2];
  }

  for(i=0;i<SIDE_DEG;i++){
    left_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }
  for(i=180-SIDE_DEG-1;i<180;i++){
    right_triangle += (laserscan_arr[i]*laserscan_arr[i]);
  }
/*
  printf("[");
  for(i=0;i<180;i++){
    printf("%d : %f\n",i,laserscan_arr[i]);
  }
  printf("]");
*/
  ROS_INFO("1 deg : %f",laserscan_arr[1]);
  left_triangle *= (0.5*DEG2RAD);
  right_triangle *=(0.5*DEG2RAD);

  //ROS_INFO("left_triangle : %f, right_triangle : %f",left_triangle,right_triangle);

 // float triangle_error = left_triangle-right_triangle;
  float triangle_error = TRIANGLE-right_triangle;
  ROS_INFO("triangle : %f",right_triangle);

  float left_square = left_triangle - TRIANGLE;

  if(phase == 0){
    //ROS_INFO("size : %d",sizeof(arrarrarr)/sizeof(float));
/*
    if((1.2*1.2 - left_square) < 0.35){
      phase++;
    }
*/
    if((laserscan_arr[2]+laserscan_arr[3]+laserscan_arr[4])/3 > 1.2){
      phase2_count ++;
      if(phase2_count >=5){
        phase++;
      }
    }
    linear_x = MAX_LINEAR_VEL*(((1.25*1.25) - left_square)/(1.25*1.25)); //Kp*0.015*((1.25*1.25) - left_square); //+ MAX_LINEAR_VEL*PRESENT_PAST_RATIO + pre_linear_x*(1 - PRESENT_PAST_RATIO) - left_square * Kp*0.01;
    angular_z = MAX_ANGULAR_Z*(triangle_error/TRIANGLE);//PRESENT_PAST_RATIO * triangle_error * Kp + pre_angular_z * (1 - PRESENT_PAST_RATIO);
    if(angular_z > MAX_ANGULAR_Z){
      angular_z = MAX_ANGULAR_Z;
    }
    if(angular_z < -1*MAX_ANGULAR_Z){
      angular_z = -1*MAX_ANGULAR_Z;
    }
    angular_z = PRESENT_PAST_RATIO*angular_z + (1-PRESENT_PAST_RATIO)*pre_angular_z;
    pre_angular_z = angular_z;
    ROS_INFO("phase0");
    ROS_WARN("angle 0 : %f angle 90 : %f angle 180 : %f ",laserscan_arr[0],laserscan_arr[90],laserscan_arr[179]);
  }
  if(phase == 1){



    float r_vel,l_vel;
    l_vel = 0.2;//0.4;                                                                  //tuning 1-1
    r_vel = l_vel * ((((1.5 - Robot_Width)*0.5)+Robot_Width)/((1.5-Robot_Width)*0.5));
    float ease_curve = -0.1; //0.07;                                                    //tuning 1-2

    //ROS_INFO("r_vel = %f",r_vel);
    float ang_vel = (r_vel-l_vel)/Robot_Width - ease_curve;
    float lin_vel = (r_vel+l_vel)/2;

    linear_x = lin_vel;
    angular_z = ang_vel;

    if(left_triangle < TRIANGLE*0.5){                                                   //tuning 1-3
      phase ++;
    }
  }
  if(phase == 2){
    ROS_INFO("phase 2");
    for(i=0;i<180;i++){
      sum_all_scan += laserscan_arr[i];
    }
    //ROS_INFO("sum_all_scan : %f",sum_all_scan);
    if(sum_all_scan < stop_threshold){
      phase++;
    }

    float triangle_error_rl = left_triangle-right_triangle;
    linear_x = 0.4*phase2_Kp * sum_all_scan;                                            //tuning 2-1
    angular_z = 0.1*phase2_Kp_angular * triangle_error_rl;                              //tuning 2-2

    sum_all_scan = 0.0;

  }
  if(phase == 3){
    ROS_INFO("phase3");
    linear_x = 0.0;
    angular_z = 0.0;
    if((wait++) > wait_threshold){
      phase ++;
    }

  }
  if(phase == 4){
    //ROS_INFO("wait_theashold : %d",wait_threshold);
    ROS_INFO("phase 4");
    pre_angular_z = 0.0;
/*
    if(left_triangle > PARKING_AREA_TRIAGNLE){
      phase ++;
    }
    */

/*
    for(i=0;i<180;i++){
      sum_all_scan += laserscan_arr[i];
    }
*/
    //ROS_INFO("sum_all_scan : %f",sum_all_scan);

    float triangle_error_rl = left_triangle-right_triangle;
    linear_x = -0.4;                                                                    //tuning 3-1
    angular_z = -1 * 0.5*phase2_Kp_angular * triangle_error_rl;                         //tuning 3-2
    angular_z = PRESENT_PAST_RATIO*angular_z + (1-PRESENT_PAST_RATIO)*pre_angular_z;

    sum_all_scan = 0.0;

    /*if(left_triangle > PARKING_AREA_TRIAGNLE){
      phase ++;
    }*/
    if(laserscan_arr[90]>0.7){                                                          //tuning 4-1
      phase4_count++;
      if(phase4_count>5){
        phase++;
      }
    }
  }
  if(phase == 5){

    ROS_INFO("phase 5");

    float r_vel,l_vel;
    l_vel = -0.2; //-0.4;                                                                //tuning 5-1
    r_vel = l_vel * ((((1.5 - Robot_Width)*0.5)+Robot_Width)/((1.5-Robot_Width)*0.5));
    float ease_curve = 0.0;//0.08;                                                       //tuning 5-2

    //ROS_INFO("r_vel = %f",r_vel);
    float ang_vel = (r_vel-l_vel)/Robot_Width - ease_curve;
    float lin_vel = (r_vel+l_vel)/2;

    linear_x = lin_vel;
    angular_z = ang_vel;

    //ang_vel *=-1;
    //lin_vel *=-1;

    if(laserscan_arr[81]>(MAX_RANGE-2) ||
       laserscan_arr[82]>(MAX_RANGE-2) ||
       laserscan_arr[83]>(MAX_RANGE-2) ||
       laserscan_arr[84]>(MAX_RANGE-2) ||
       laserscan_arr[85]>(MAX_RANGE-2) ||
       laserscan_arr[86]>(MAX_RANGE-2) ||
       laserscan_arr[88]>(MAX_RANGE-2) ||
       laserscan_arr[89]>(MAX_RANGE-2) ||
       laserscan_arr[90]>(MAX_RANGE-2) ||
       laserscan_arr[91]>(MAX_RANGE-2) ||
       laserscan_arr[92]>(MAX_RANGE-2) ||
       laserscan_arr[93]>(MAX_RANGE-2) ||
       laserscan_arr[94]>(MAX_RANGE-2) ||
       laserscan_arr[95]>(MAX_RANGE-2) ||
       laserscan_arr[96]>(MAX_RANGE-2) ||
       laserscan_arr[97]>(MAX_RANGE-2) ||
       laserscan_arr[98]>(MAX_RANGE-2) ||
       laserscan_arr[99]>(MAX_RANGE-2)
       ){
        phase5_count++;
        ROS_WARN("isinf");
        if(phase5_count >= 5){
          phase++;
          pre_angular_z = 0.0;
        }
    }
  }
  if(phase == 6){
    ROS_INFO("phase6");
    float right_error = 0.6*1.414- laserscan_arr[135];                                                                        //tuning 5-0

    if(laserscan_arr[170]>(MAX_RANGE-2)){
      phase++;
    }
    else if(laserscan_arr[120]>(MAX_RANGE-2)){
      linear_x = 0.2;
      right_error = 0.6 - laserscan_arr[179];
      angular_z = PRESENT_PAST_RATIO * right_error * Kp*0.5 + pre_angular_z * (1 - PRESENT_PAST_RATIO);
      pre_angular_z = angular_z;

    }
    else if(laserscan_arr[160]>(MAX_RANGE-2)){
      linear_x = 0.1;
      right_error = 0.6 - laserscan_arr[179];
      angular_z = PRESENT_PAST_RATIO * right_error * Kp*0.5 + pre_angular_z * (1 - PRESENT_PAST_RATIO);
      pre_angular_z = angular_z;
    }
    else{
      float right_error = 0.6*1.414- laserscan_arr[135];                                                                        //tuning 5-0
      linear_x =  0.4;//+ MAX_LINEAR_VEL*PRESENT_PAST_RATIO + pre_linear_x*(1 - PRESENT_PAST_RATIO) - left_square * Kp*0.01;    //tuning 5-1
      //angular_z = PRESENT_PAST_RATIO * right_error * Kp*0.5 + pre_angular_z * (1 - PRESENT_PAST_RATIO);                         //tuning 5-2
      angular_z = MAX_LINEAR_VEL*(right_error/0.6*1.414);
      angular_z = PRESENT_PAST_RATIO * angular_z + (1-PRESENT_PAST_RATIO)*pre_angular_z;
      pre_angular_z = angular_z;
    }
  }
  if(phase == 7){
    linear_x = 0.0;
    angular_z = 0.0;

  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto_parking_new_node");
  ros::NodeHandle nh;

  ros::Publisher cmdvel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/scan", 1000, scan_Callback);

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    //ROS_INFO("%d",SIDE_DEG);
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;

    cmdvel_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
