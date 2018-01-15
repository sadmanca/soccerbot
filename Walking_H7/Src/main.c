/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_hal.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
/* Motor driver. */
#include "../../Dynamixel_AX-12A_Driver/src/Dynamixel_AX-12A.h"
#include "../../Dynamixel_AX-12A_Driver/src/Dynamixel_AX-12A.c"
#include <stdio.h>

/* Motor Angles. */
#include "../../../../control/soccer-control/angles.h"

#include "MPU6050.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern const double MOTORANGLES[DIM][SIZE];
//const double MOTORANGLES[12][1001] = {
//		{//1
//				0.044595,0.033301,0.022335,0.011681,0.001322,-0.0087571,-0.018571,-0.028134,-0.03746,-0.046561,-0.05545,-0.064137,-0.072634,-0.08095,-0.089097,-0.097082,-0.10491,-0.1126,-0.12015,-0.12757,-0.13487,-0.14205,-0.14913,-0.15609,-0.16296,-0.16973,-0.17642,-0.18302,-0.18953,-0.19597,-0.20234,-0.20863,-0.21486,-0.22103,-0.22713,-0.23318,-0.23916,-0.2451,-0.25098,-0.25682,-0.2626,-0.26802,-0.27311,-0.27787,-0.28234,-0.28653,-0.29045,-0.29414,-0.29759,-0.30083,-0.30387,-0.30673,-0.3094,-0.31191,-0.31427,-0.31649,-0.31856,-0.32051,-0.32234,-0.32406,-0.32568,-0.32719,-0.32861,-0.32995,-0.33121,-0.33239,-0.33349,-0.33453,-0.33551,-0.33643,-0.33729,-0.3381,-0.33886,-0.33958,-0.34025,-0.34088,-0.34147,-0.34203,-0.34255,-0.34304,-0.34351,-0.34394,-0.34435,-0.34473,-0.34509,-0.34543,-0.34575,-0.34605,-0.34633,-0.34659,-0.33166,-0.31717,-0.30312,-0.28948,-0.27624,-0.26338,-0.2509,-0.23876,-0.22697,-0.2155,-0.20433,-0.19346,-0.18287,-0.17255,-0.16248,-0.15265,-0.14304,-0.13366,-0.12448,-0.11549,-0.10669,-0.098061,-0.089599,-0.081294,-0.073138,-0.065122,-0.057238,-0.049481,-0.041843,-0.034317,-0.026899,-0.019581,-0.012359,-0.0052278,0.0018175,0.0087814,0.015668,0.022482,0.029225,0.035903,0.042518,0.049074,0.055573,0.062017,0.068411,0.074755,0.081052,0.087304,0.093514,0.099682,0.10581,0.11157,0.11697,0.12204,0.1268,0.13127,0.13547,0.13941,0.14311,0.14659,0.14985,0.15329,0.15728,0.16179,0.16681,0.17233,0.17835,0.18485,0.19182,0.19923,0.20707,0.21528,0.22382,0.23262,0.24159,0.25064,0.25964,0.26845,0.27692,0.28488,0.29216,0.29859,0.30402,0.30831,0.31135,0.31307,0.31344,0.31249,0.31026,0.30685,0.30239,0.29702,0.29092,0.28424,0.27717,0.26985,0.26243,0.25505,0.24782,0.24084,0.21473,0.19066,0.16853,0.14823,0.12961,0.11252,0.096813,0.082326,0.0689,0.056376
//		},
//		{//2
//	0.38545,0.39119,0.39712,0.40322,0.40949,0.41591,0.42247,0.42916,0.43597,0.44289,0.44991,0.45701,0.4642,0.47146,0.47878,0.48615,0.49357,0.50102,0.5085,0.51601,0.52353,0.53106,0.53858,0.54611,0.55362,0.56112,0.56859,0.57604,0.58345,0.59082,0.59815,0.60543,0.61266,0.61983,0.62694,0.63398,0.64096,0.64785,0.65468,0.66142,0.66807,0.67422,0.67991,0.68516,0.69002,0.69452,0.69869,0.70256,0.70614,0.70946,0.71255,0.71542,0.71808,0.72056,0.72286,0.725,0.727,0.72886,0.73059,0.73221,0.73371,0.73512,0.73643,0.73766,0.7388,0.73987,0.74087,0.7418,0.74268,0.7435,0.74426,0.74498,0.74565,0.74627,0.74686,0.74741,0.74793,0.74841,0.74886,0.74929,0.74969,0.75006,0.75041,0.75074,0.75105,0.75134,0.75161,0.75186,0.7521,0.75232,0.75932,0.76618,0.77294,0.77963,0.78626,0.79285,0.79941,0.80595,0.81248,0.819,0.82551,0.83202,0.83853,0.84504,0.85153,0.85802,0.8645,0.87096,0.8774,0.88381,0.8902,0.89656,0.90288,0.90915,0.91538,0.92156,0.92768,0.93374,0.93973,0.94566,0.95151,0.95727,0.96296,0.96856,0.97407,0.97948,0.98479,0.99,0.9951,1.0001,1.005,1.0097,1.0144,1.0189,1.0233,1.0276,1.0317,1.0357,1.0396,1.0433,1.0469,1.0501,1.053,1.0556,1.058,1.0601,1.062,1.0637,1.0653,1.0667,1.068,1.0722,1.0821,1.0966,1.1149,1.1361,1.1593,1.1838,1.2087,1.2334,1.2572,1.2796,1.2999,1.3178,1.3325,1.3438,1.3511,1.354,1.3521,1.3453,1.3333,1.316,1.2936,1.2663,1.2345,1.1987,1.1596,1.1178,1.074,1.029,0.98328,0.93749,0.892,0.84716,0.80318,0.76022,0.71841,0.6778,0.63846,0.60045,0.56616,0.53385,0.50368,0.4759,0.45083,0.42887,0.41051,0.39633,0.38697,0.38311
//		},
//		{//3
//				-1.4042,-1.4083,-1.4122,-1.416,-1.4197,-1.4232,-1.4266,-1.4299,-1.4331,-1.4361,-1.439,-1.4418,-1.4445,-1.447,-1.4494,-1.4517,-1.4538,-1.4558,-1.4576,-1.4594,-1.4609,-1.4624,-1.4636,-1.4648,-1.4657,-1.4665,-1.4672,-1.4677,-1.468,-1.4682,-1.4682,-1.468,-1.4677,-1.4672,-1.4665,-1.4657,-1.4646,-1.4634,-1.462,-1.4605,-1.4587,-1.4569,-1.455,-1.453,-1.4511,-1.4492,-1.4472,-1.4453,-1.4435,-1.4417,-1.4399,-1.4382,-1.4365,-1.4349,-1.4333,-1.4318,-1.4304,-1.429,-1.4277,-1.4265,-1.4253,-1.4241,-1.4231,-1.422,-1.421,-1.4201,-1.4192,-1.4184,-1.4176,-1.4168,-1.4161,-1.4154,-1.4148,-1.4142,-1.4136,-1.4131,-1.4126,-1.4121,-1.4116,-1.4112,-1.4108,-1.4104,-1.41,-1.4097,-1.4094,-1.4091,-1.4088,-1.4085,-1.4083,-1.408,-1.4204,-1.4316,-1.4419,-1.4513,-1.4599,-1.4677,-1.4748,-1.4813,-1.4871,-1.4925,-1.4973,-1.5016,-1.5055,-1.5089,-1.5119,-1.5146,-1.5168,-1.5188,-1.5204,-1.5216,-1.5226,-1.5232,-1.5236,-1.5237,-1.5235,-1.523,-1.5222,-1.5212,-1.52,-1.5185,-1.5168,-1.5148,-1.5125,-1.5101,-1.5074,-1.5045,-1.5013,-1.4979,-1.4943,-1.4905,-1.4864,-1.4822,-1.4777,-1.473,-1.468,-1.4629,-1.4575,-1.4519,-1.4461,-1.4401,-1.4339,-1.4278,-1.4219,-1.4163,-1.4108,-1.4056,-1.4005,-1.3957,-1.3911,-1.3866,-1.3824,-1.3844,-1.3975,-1.4203,-1.4511,-1.4885,-1.531,-1.5773,-1.6263,-1.6769,-1.7282,-1.7796,-1.8302,-1.8796,-1.9272,-1.9725,-2.0151,-2.0545,-2.0904,-2.1225,-2.1503,-2.1736,-2.1921,-2.2055,-2.2137,-2.2166,-2.2142,-2.2063,-2.1932,-2.1749,-2.1518,-2.124,-2.0918,-2.0556,-2.0157,-1.9725,-1.9264,-1.8779,-1.8273,-1.7753,-1.7291,-1.6821,-1.6351,-1.5893,-1.5456,-1.5052,-1.4696,-1.4402,-1.4184,-1.4059
//		},
//		{//4
//				1.0187,1.0171,1.0151,1.0128,1.0102,1.0073,1.0042,1.0008,0.99711,0.99322,0.98912,0.98479,0.98027,0.97554,0.97062,0.96551,0.96023,0.95477,0.94915,0.94336,0.93741,0.93131,0.92505,0.91865,0.91211,0.90543,0.89862,0.89167,0.88459,0.87739,0.87006,0.86262,0.85505,0.84738,0.83959,0.83169,0.82368,0.81557,0.80735,0.79904,0.79063,0.78264,0.77506,0.76788,0.76108,0.75464,0.74855,0.74279,0.73734,0.7322,0.72734,0.72275,0.71842,0.71433,0.71048,0.70684,0.70342,0.70018,0.69714,0.69427,0.69157,0.68902,0.68662,0.68436,0.68223,0.68023,0.67834,0.67657,0.6749,0.67332,0.67184,0.67045,0.66914,0.66791,0.66675,0.66565,0.66463,0.66366,0.66275,0.6619,0.66109,0.66034,0.65963,0.65896,0.65833,0.65774,0.65718,0.65666,0.65617,0.6557,0.66105,0.66545,0.66897,0.67167,0.6736,0.67482,0.67537,0.67531,0.67465,0.67346,0.67175,0.66956,0.66692,0.66386,0.66039,0.65655,0.65235,0.64781,0.64296,0.63781,0.63237,0.62667,0.62071,0.6145,0.60807,0.60142,0.59456,0.5875,0.58026,0.57284,0.56524,0.55749,0.54958,0.54152,0.53332,0.52498,0.51652,0.50793,0.49922,0.4904,0.48147,0.47243,0.46329,0.45406,0.44473,0.43532,0.42581,0.41622,0.40655,0.3968,0.38698,0.37769,0.36893,0.36065,0.35284,0.34547,0.33851,0.33196,0.32577,0.31994,0.31445,0.31213,0.31542,0.32366,0.33621,0.35241,0.3717,0.39357,0.41763,0.44354,0.47106,0.50002,0.5303,0.56186,0.59466,0.62871,0.66401,0.70056,0.73832,0.7772,0.81704,0.85757,0.89845,0.93919,0.97923,1.0179,1.0546,1.0886,1.1192,1.146,1.1685,1.1865,1.1998,1.2084,1.2125,1.2123,1.208,1.2001,1.1888,1.1748,1.1629,1.1482,1.1315,1.1134,1.0947,1.0764,1.0591,1.0438,1.0314,1.0228
//		},
//		{//5
//				-0.044595,-0.033301,-0.022335,-0.011681,-0.001322,0.0087571,0.018571,0.028134,0.03746,0.046561,0.05545,0.064137,0.072634,0.08095,0.089097,0.097082,0.10491,0.1126,0.12015,0.12757,0.13487,0.14205,0.14913,0.15609,0.16296,0.16973,0.17642,0.18302,0.18953,0.19597,0.20234,0.20863,0.21486,0.22103,0.22713,0.23318,0.23916,0.2451,0.25098,0.25682,0.2626,0.26802,0.27311,0.27787,0.28234,0.28653,0.29045,0.29414,0.29759,0.30083,0.30387,0.30673,0.3094,0.31191,0.31427,0.31649,0.31856,0.32051,0.32234,0.32406,0.32568,0.32719,0.32861,0.32995,0.33121,0.33239,0.33349,0.33453,0.33551,0.33643,0.33729,0.3381,0.33886,0.33958,0.34025,0.34088,0.34147,0.34203,0.34255,0.34304,0.34351,0.34394,0.34435,0.34473,0.34509,0.34543,0.34575,0.34605,0.34633,0.34659,0.33166,0.31717,0.30312,0.28948,0.27624,0.26338,0.2509,0.23876,0.22697,0.2155,0.20433,0.19346,0.18287,0.17255,0.16248,0.15265,0.14304,0.13366,0.12448,0.11549,0.10669,0.098061,0.089599,0.081294,0.073138,0.065122,0.057238,0.049481,0.041843,0.034317,0.026899,0.019581,0.012359,0.0052278,-0.0018175,-0.0087814,-0.015668,-0.022482,-0.029225,-0.035903,-0.042518,-0.049074,-0.055573,-0.062017,-0.068411,-0.074755,-0.081052,-0.087304,-0.093514,-0.099682,-0.10581,-0.11157,-0.11697,-0.12204,-0.1268,-0.13127,-0.13547,-0.13941,-0.14311,-0.14659,-0.14985,-0.15329,-0.15728,-0.16179,-0.16681,-0.17233,-0.17835,-0.18485,-0.19182,-0.19923,-0.20707,-0.21528,-0.22382,-0.23262,-0.24159,-0.25064,-0.25964,-0.26845,-0.27692,-0.28488,-0.29216,-0.29859,-0.30402,-0.30831,-0.31135,-0.31307,-0.31344,-0.31249,-0.31026,-0.30685,-0.30239,-0.29702,-0.29092,-0.28424,-0.27717,-0.26985,-0.26243,-0.25505,-0.24782,-0.24084,-0.21473,-0.19066,-0.16853,-0.14823,-0.12961,-0.11252,-0.096813,-0.082326,-0.0689,-0.056376
//		},
//		{//6
//				0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
//		},
//		{//7
//				0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
//		},
//		{//8
//				-0.20433,-0.19346,-0.18287,-0.17255,-0.16248,-0.15265,-0.14304,-0.13366,-0.12448,-0.11549,-0.10669,-0.098061,-0.089599,-0.081294,-0.073138,-0.065122,-0.057238,-0.049481,-0.041843,-0.034317,-0.026899,-0.019581,-0.012359,-0.0052278,0.0018175,0.0087814,0.015668,0.022482,0.029225,0.035903,0.042518,0.049074,0.055573,0.062017,0.068411,0.074755,0.081052,0.087304,0.093514,0.099682,0.10581,0.11157,0.11697,0.12204,0.1268,0.13127,0.13547,0.13941,0.14311,0.14659,0.14985,0.15329,0.15728,0.16179,0.16681,0.17233,0.17835,0.18485,0.19182,0.19923,0.20707,0.21528,0.22382,0.23262,0.24159,0.25064,0.25964,0.26845,0.27692,0.28488,0.29216,0.29859,0.30402,0.30831,0.31135,0.31307,0.31344,0.31249,0.31026,0.30685,0.30239,0.29702,0.29092,0.28424,0.27717,0.26985,0.26243,0.25505,0.24782,0.24084,0.21473,0.19066,0.16853,0.14823,0.12961,0.11252,0.096813,0.082326,0.0689,0.056376,0.044595,0.033301,0.022335,0.011681,0.001322,-0.0087571,-0.018571,-0.028134,-0.03746,-0.046561,-0.05545,-0.064137,-0.072634,-0.08095,-0.089097,-0.097082,-0.10491,-0.1126,-0.12015,-0.12757,-0.13487,-0.14205,-0.14913,-0.15609,-0.16296,-0.16973,-0.17642,-0.18302,-0.18953,-0.19597,-0.20234,-0.20863,-0.21486,-0.22103,-0.22713,-0.23318,-0.23916,-0.2451,-0.25098,-0.25682,-0.2626,-0.26802,-0.27311,-0.27787,-0.28234,-0.28653,-0.29045,-0.29414,-0.29759,-0.30083,-0.30387,-0.30673,-0.3094,-0.31191,-0.31427,-0.31649,-0.31856,-0.32051,-0.32234,-0.32406,-0.32568,-0.32719,-0.32861,-0.32995,-0.33121,-0.33239,-0.33349,-0.33453,-0.33551,-0.33643,-0.33729,-0.3381,-0.33886,-0.33958,-0.34025,-0.34088,-0.34147,-0.34203,-0.34255,-0.34304,-0.34351,-0.34394,-0.34435,-0.34473,-0.34509,-0.34543,-0.34575,-0.34605,-0.34633,-0.34659,-0.33166,-0.31717,-0.30312,-0.28948,-0.27624,-0.26338,-0.2509,-0.23876,-0.22697,-0.2155
//		},
//		{//9
//				0.67175,0.66956,0.66692,0.66386,0.66039,0.65655,0.65235,0.64781,0.64296,0.63781,0.63237,0.62667,0.62071,0.6145,0.60807,0.60142,0.59456,0.5875,0.58026,0.57284,0.56524,0.55749,0.54958,0.54152,0.53332,0.52498,0.51652,0.50793,0.49922,0.4904,0.48147,0.47243,0.46329,0.45406,0.44473,0.43532,0.42581,0.41622,0.40655,0.3968,0.38698,0.37769,0.36893,0.36065,0.35284,0.34547,0.33851,0.33196,0.32577,0.31994,0.31445,0.31213,0.31542,0.32366,0.33621,0.35241,0.3717,0.39357,0.41763,0.44354,0.47106,0.50002,0.5303,0.56186,0.59466,0.62871,0.66401,0.70056,0.73832,0.7772,0.81704,0.85757,0.89845,0.93919,0.97923,1.0179,1.0546,1.0886,1.1192,1.146,1.1685,1.1865,1.1998,1.2084,1.2125,1.2123,1.208,1.2001,1.1888,1.1748,1.1629,1.1482,1.1315,1.1134,1.0947,1.0764,1.0591,1.0438,1.0314,1.0228,1.0187,1.0171,1.0151,1.0128,1.0102,1.0073,1.0042,1.0008,0.99711,0.99322,0.98912,0.98479,0.98027,0.97554,0.97062,0.96551,0.96023,0.95477,0.94915,0.94336,0.93741,0.93131,0.92505,0.91865,0.91211,0.90543,0.89862,0.89167,0.88459,0.87739,0.87006,0.86262,0.85505,0.84738,0.83959,0.83169,0.82368,0.81557,0.80735,0.79904,0.79063,0.78264,0.77506,0.76788,0.76108,0.75464,0.74855,0.74279,0.73734,0.7322,0.72734,0.72275,0.71842,0.71433,0.71048,0.70684,0.70342,0.70018,0.69714,0.69427,0.69157,0.68902,0.68662,0.68436,0.68223,0.68023,0.67834,0.67657,0.6749,0.67332,0.67184,0.67045,0.66914,0.66791,0.66675,0.66565,0.66463,0.66366,0.66275,0.6619,0.66109,0.66034,0.65963,0.65896,0.65833,0.65774,0.65718,0.65666,0.65617,0.6557,0.66105,0.66545,0.66897,0.67167,0.6736,0.67482,0.67537,0.67531,0.67465,0.67346
//		},
//		{//10
//				-1.4973,-1.5016,-1.5055,-1.5089,-1.5119,-1.5146,-1.5168,-1.5188,-1.5204,-1.5216,-1.5226,-1.5232,-1.5236,-1.5237,-1.5235,-1.523,-1.5222,-1.5212,-1.52,-1.5185,-1.5168,-1.5148,-1.5125,-1.5101,-1.5074,-1.5045,-1.5013,-1.4979,-1.4943,-1.4905,-1.4864,-1.4822,-1.4777,-1.473,-1.468,-1.4629,-1.4575,-1.4519,-1.4461,-1.4401,-1.4339,-1.4278,-1.4219,-1.4163,-1.4108,-1.4056,-1.4005,-1.3957,-1.3911,-1.3866,-1.3824,-1.3844,-1.3975,-1.4203,-1.4511,-1.4885,-1.531,-1.5773,-1.6263,-1.6769,-1.7282,-1.7796,-1.8302,-1.8796,-1.9272,-1.9725,-2.0151,-2.0545,-2.0904,-2.1225,-2.1503,-2.1736,-2.1921,-2.2055,-2.2137,-2.2166,-2.2142,-2.2063,-2.1932,-2.1749,-2.1518,-2.124,-2.0918,-2.0556,-2.0157,-1.9725,-1.9264,-1.8779,-1.8273,-1.7753,-1.7291,-1.6821,-1.6351,-1.5893,-1.5456,-1.5052,-1.4696,-1.4402,-1.4184,-1.4059,-1.4042,-1.4083,-1.4122,-1.416,-1.4197,-1.4232,-1.4266,-1.4299,-1.4331,-1.4361,-1.439,-1.4418,-1.4445,-1.447,-1.4494,-1.4517,-1.4538,-1.4558,-1.4576,-1.4594,-1.4609,-1.4624,-1.4636,-1.4648,-1.4657,-1.4665,-1.4672,-1.4677,-1.468,-1.4682,-1.4682,-1.468,-1.4677,-1.4672,-1.4665,-1.4657,-1.4646,-1.4634,-1.462,-1.4605,-1.4587,-1.4569,-1.455,-1.453,-1.4511,-1.4492,-1.4472,-1.4453,-1.4435,-1.4417,-1.4399,-1.4382,-1.4365,-1.4349,-1.4333,-1.4318,-1.4304,-1.429,-1.4277,-1.4265,-1.4253,-1.4241,-1.4231,-1.422,-1.421,-1.4201,-1.4192,-1.4184,-1.4176,-1.4168,-1.4161,-1.4154,-1.4148,-1.4142,-1.4136,-1.4131,-1.4126,-1.4121,-1.4116,-1.4112,-1.4108,-1.4104,-1.41,-1.4097,-1.4094,-1.4091,-1.4088,-1.4085,-1.4083,-1.408,-1.4204,-1.4316,-1.4419,-1.4513,-1.4599,-1.4677,-1.4748,-1.4813,-1.4871,-1.4925
//		},
//		{//11
//				0.82551,0.83202,0.83853,0.84504,0.85153,0.85802,0.8645,0.87096,0.8774,0.88381,0.8902,0.89656,0.90288,0.90915,0.91538,0.92156,0.92768,0.93374,0.93973,0.94566,0.95151,0.95727,0.96296,0.96856,0.97407,0.97948,0.98479,0.99,0.9951,1.0001,1.005,1.0097,1.0144,1.0189,1.0233,1.0276,1.0317,1.0357,1.0396,1.0433,1.0469,1.0501,1.053,1.0556,1.058,1.0601,1.062,1.0637,1.0653,1.0667,1.068,1.0722,1.0821,1.0966,1.1149,1.1361,1.1593,1.1838,1.2087,1.2334,1.2572,1.2796,1.2999,1.3178,1.3325,1.3438,1.3511,1.354,1.3521,1.3453,1.3333,1.316,1.2936,1.2663,1.2345,1.1987,1.1596,1.1178,1.074,1.029,0.98328,0.93749,0.892,0.84716,0.80318,0.76022,0.71841,0.6778,0.63846,0.60045,0.56616,0.53385,0.50368,0.4759,0.45083,0.42887,0.41051,0.39633,0.38697,0.38311,0.38545,0.39119,0.39712,0.40322,0.40949,0.41591,0.42247,0.42916,0.43597,0.44289,0.44991,0.45701,0.4642,0.47146,0.47878,0.48615,0.49357,0.50102,0.5085,0.51601,0.52353,0.53106,0.53858,0.54611,0.55362,0.56112,0.56859,0.57604,0.58345,0.59082,0.59815,0.60543,0.61266,0.61983,0.62694,0.63398,0.64096,0.64785,0.65468,0.66142,0.66807,0.67422,0.67991,0.68516,0.69002,0.69452,0.69869,0.70256,0.70614,0.70946,0.71255,0.71542,0.71808,0.72056,0.72286,0.725,0.727,0.72886,0.73059,0.73221,0.73371,0.73512,0.73643,0.73766,0.7388,0.73987,0.74087,0.7418,0.74268,0.7435,0.74426,0.74498,0.74565,0.74627,0.74686,0.74741,0.74793,0.74841,0.74886,0.74929,0.74969,0.75006,0.75041,0.75074,0.75105,0.75134,0.75161,0.75186,0.7521,0.75232,0.75932,0.76618,0.77294,0.77963,0.78626,0.79285,0.79941,0.80595,0.81248,0.819
//		},
//		{//12
//				0.20433,0.19346,0.18287,0.17255,0.16248,0.15265,0.14304,0.13366,0.12448,0.11549,0.10669,0.098061,0.089599,0.081294,0.073138,0.065122,0.057238,0.049481,0.041843,0.034317,0.026899,0.019581,0.012359,0.0052278,-0.0018175,-0.0087814,-0.015668,-0.022482,-0.029225,-0.035903,-0.042518,-0.049074,-0.055573,-0.062017,-0.068411,-0.074755,-0.081052,-0.087304,-0.093514,-0.099682,-0.10581,-0.11157,-0.11697,-0.12204,-0.1268,-0.13127,-0.13547,-0.13941,-0.14311,-0.14659,-0.14985,-0.15329,-0.15728,-0.16179,-0.16681,-0.17233,-0.17835,-0.18485,-0.19182,-0.19923,-0.20707,-0.21528,-0.22382,-0.23262,-0.24159,-0.25064,-0.25964,-0.26845,-0.27692,-0.28488,-0.29216,-0.29859,-0.30402,-0.30831,-0.31135,-0.31307,-0.31344,-0.31249,-0.31026,-0.30685,-0.30239,-0.29702,-0.29092,-0.28424,-0.27717,-0.26985,-0.26243,-0.25505,-0.24782,-0.24084,-0.21473,-0.19066,-0.16853,-0.14823,-0.12961,-0.11252,-0.096813,-0.082326,-0.0689,-0.056376,-0.044595,-0.033301,-0.022335,-0.011681,-0.001322,0.0087571,0.018571,0.028134,0.03746,0.046561,0.05545,0.064137,0.072634,0.08095,0.089097,0.097082,0.10491,0.1126,0.12015,0.12757,0.13487,0.14205,0.14913,0.15609,0.16296,0.16973,0.17642,0.18302,0.18953,0.19597,0.20234,0.20863,0.21486,0.22103,0.22713,0.23318,0.23916,0.2451,0.25098,0.25682,0.2626,0.26802,0.27311,0.27787,0.28234,0.28653,0.29045,0.29414,0.29759,0.30083,0.30387,0.30673,0.3094,0.31191,0.31427,0.31649,0.31856,0.32051,0.32234,0.32406,0.32568,0.32719,0.32861,0.32995,0.33121,0.33239,0.33349,0.33453,0.33551,0.33643,0.33729,0.3381,0.33886,0.33958,0.34025,0.34088,0.34147,0.34203,0.34255,0.34304,0.34351,0.34394,0.34435,0.34473,0.34509,0.34543,0.34575,0.34605,0.34633,0.34659,0.33166,0.31717,0.30312,0.28948,0.27624,0.26338,0.2509,0.23876,0.22697,0.2155
//		}
//};

const enum motorNames {MOTOR1, MOTOR2, MOTOR3, MOTOR4, MOTOR5,
					   MOTOR6, MOTOR7, MOTOR8, MOTOR9, MOTOR10,
					   MOTOR11, MOTOR12, MOTOR13, MOTOR14, MOTOR15,
					   MOTOR16, MOTOR17, MOTOR18
};

const double PI = 3.141592654;

Dynamixel_HandleTypeDef Motor1;
Dynamixel_HandleTypeDef Motor2;
Dynamixel_HandleTypeDef Motor3;
Dynamixel_HandleTypeDef Motor4;
Dynamixel_HandleTypeDef Motor5;
Dynamixel_HandleTypeDef Motor6;
Dynamixel_HandleTypeDef Motor7;
Dynamixel_HandleTypeDef Motor8;
Dynamixel_HandleTypeDef Motor9;
Dynamixel_HandleTypeDef Motor10;
Dynamixel_HandleTypeDef Motor11;
Dynamixel_HandleTypeDef Motor12;
Dynamixel_HandleTypeDef Motor13;
Dynamixel_HandleTypeDef Motor14;
Dynamixel_HandleTypeDef Motor15;
Dynamixel_HandleTypeDef Motor16;
Dynamixel_HandleTypeDef Motor17;
Dynamixel_HandleTypeDef Motor18;

MPU6050_HandleTypeDef IMUdata;

int8_t output_buffer[6];
int8_t output_buffer_2[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void integrate(double * collector, double value) {
	*collector += value * 0.01;
}

double derivate(double last, double current) {
	return (current - last) / 0.01;
}

double PID(double value, double * collector, double * last, double kp, double ki, double kd) {
	integrate(collector, value);
	double output = kp * value + ki * (*collector) + kd * derivate(*last, value);
	*last = value;
	return output;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void StartIMUtask()
{

  /* USER CODE BEGIN StartIMUtask */
    uint8_t output_buffer[6];
    uint8_t output_buffer_2[6];
    MPU6050_HandleTypeDef IMUdata;
    IMUdata._I2C_Handle = &hi2c1;
    uint16_t test=5;
    MPU6050_RESET_SENSOR_REG(IMUdata);
    MPU6050_init(&IMUdata);

  /* Infinite loop */
  for(;;)
  {
	  MPU6050_READ_DATA(&IMUdata, MPU6050_RA_ACCEL_XOUT_H,output_buffer);
	  			  	uint16_t Xa = abs((int16_t)(output_buffer[0]<<8|output_buffer[1]));
	  			  	uint16_t Ya = abs((int16_t)(output_buffer[2]<<8|output_buffer[3]));
	  			  	uint16_t Za = abs((int16_t)(output_buffer[4]<<8|output_buffer[5]));

	  			  	//Now read the gyroscope values:

	  				MPU6050_READ_DATA(&IMUdata, MPU6050_RA_GYRO_XOUT_H,output_buffer_2);
	  				uint16_t Xg = abs((int16_t)(output_buffer_2[0]<<8|output_buffer_2[1]));
	  				uint16_t Yg = abs((int16_t)(output_buffer_2[2]<<8|output_buffer_2[3]));
	  				uint16_t Zg = abs((int16_t)(output_buffer_2[4]<<8|output_buffer_2[5]));


	  				//Now store in struct:
	  				IMUdata._X_ACCEL = Xa;
	  				IMUdata._Y_ACCEL = Ya;
	  				IMUdata._Z_ACCEL = Za;

	  				IMUdata._X_GYRO = Xg;
	  				IMUdata._Y_GYRO = Yg;
	  				IMUdata._Z_GYRO = Zg;

	  				//TEST: Read a specific register

	  				//Now enqueue the struct
	  				//NOTE: determine how large the queue should be

	  				//xQueueSend( IMUqueueHandle, &IMUdata, 1000);
	  				HAL_UART_Transmit(&huart3, &Xg, 2, 10);
	  				//HAL_UART_Transmit(&huart3, &test, 2, 10);
	  				HAL_Delay(1000);
  }
  /* USER CODE END StartIMUtask */
}
void getIMU(){
	/* USER CODE BEGIN StartIMUtask */

	    uint16_t test=5;
		  MPU6050_READ_DATA(&IMUdata, MPU6050_RA_ACCEL_XOUT_H,output_buffer);
		  			  	int16_t Xa = (int16_t)((output_buffer[0]<<8|output_buffer[1]));
		  			  	int16_t Ya = (int16_t)((output_buffer[2]<<8|output_buffer[3]));
		  			  	int16_t Za = (int16_t)((output_buffer[4]<<8|output_buffer[5]));

		  			  	//Now read the gyroscope values:

		  				MPU6050_READ_DATA(&IMUdata, MPU6050_RA_GYRO_XOUT_H,output_buffer_2);
		  				int16_t Xg = (int16_t)((output_buffer_2[0]<<8|output_buffer_2[1]));
		  				int16_t Yg = (int16_t)((output_buffer_2[2]<<8|output_buffer_2[3]));
		  				int16_t Zg = (int16_t)((output_buffer_2[4]<<8|output_buffer_2[5]));


		  				//Now store in struct:
		  				IMUdata._X_ACCEL = Xa;
		  				IMUdata._Y_ACCEL = Ya;
		  				IMUdata._Z_ACCEL = Za;

		  				IMUdata._X_GYRO = Xg;
		  				IMUdata._Y_GYRO = Yg;
		  				IMUdata._Z_GYRO = Zg;

		  				//TEST: Read a specific register

		  				//Now enqueue the struct
		  				//NOTE: determine how large the queue should be

		  				//xQueueSend( IMUqueueHandle, &IMUdata, 1000);
//		  				HAL_UART_Transmit(&huart3, &Xa, 2, 10);
		  				//HAL_UART_Transmit(&huart3, &test, 2, 10);
//		  				HAL_Delay(100);
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART7_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
//  char a = 0;
//  for(int i = 0; i < 20; i++){
//	  HAL_UART_Transmit(&huart3, &a, 1, 10);
//	  HAL_Delay(500);
//  }
//  a = 0xAA;
//  HAL_UART_Transmit(&huart3, &a, 1, 10);
//  HAL_Delay(100);
  // StartIMUtask();
//  while(1){
//	  HAL_UART_Transmit(&huart3, "cv", 2, 10);
//	  HAL_Delay(100);
//  }
  HAL_Delay(1000);
  IMUdata._I2C_Handle = &hi2c1;
  MPU6050_RESET_SENSOR_REG(&IMUdata);
  HAL_Delay(1000);
  MPU6050_init(&IMUdata);
  HAL_Delay(1000);
//  while(1){
//	  HAL_Delay(100);
//	  getIMU();
//	  char zz[2];
//	  zz[1] = IMUdata._X_ACCEL & 0xFF;
//	  zz[0] = IMUdata._X_ACCEL >> 8;
//	  HAL_UART_Transmit(&huart3, (uint8_t *)zz, 2, 10);
//  }
  Dynamixel_Init(&Motor1, 1, &huart2, GPIOD, GPIO_PIN_7);
  Dynamixel_Init(&Motor2, 2, &huart2, GPIOD, GPIO_PIN_7);
  Dynamixel_Init(&Motor3, 3, &huart2, GPIOD, GPIO_PIN_7);
  Dynamixel_Init(&Motor4, 4, &huart1, GPIOB, GPIO_PIN_10);
  Dynamixel_Init(&Motor5, 5, &huart1, GPIOB, GPIO_PIN_10);
  Dynamixel_Init(&Motor6, 6, &huart1, GPIOB, GPIO_PIN_10);
  Dynamixel_Init(&Motor7, 7, &huart7, GPIOA, GPIO_PIN_15);
  Dynamixel_Init(&Motor8, 8, &huart7, GPIOA, GPIO_PIN_15);
  Dynamixel_Init(&Motor9, 9, &huart7, GPIOA, GPIO_PIN_15);
  Dynamixel_Init(&Motor10, 10, &huart5, GPIOB, GPIO_PIN_0);
  Dynamixel_Init(&Motor11, 11, &huart5, GPIOB, GPIO_PIN_0);
  Dynamixel_Init(&Motor12, 12, &huart5, GPIOB, GPIO_PIN_0);
  Dynamixel_Init(&Motor13, 13, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor14, 14, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor15, 15, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor16, 16, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor17, 17, &huart4, GPIOA, GPIO_PIN_0);
  Dynamixel_Init(&Motor18, 18, &huart4, GPIOA, GPIO_PIN_0);

  Dynamixel_HandleTypeDef* arrDynamixel[18];
  arrDynamixel[0] = &Motor1;
  arrDynamixel[1] = &Motor2;
  arrDynamixel[2] = &Motor3;
  arrDynamixel[3] = &Motor4;
  arrDynamixel[4] = &Motor5;
  arrDynamixel[5] = &Motor6;
  arrDynamixel[6] = &Motor7;
  arrDynamixel[7] = &Motor8;
  arrDynamixel[8] = &Motor9;
  arrDynamixel[9] = &Motor10;
  arrDynamixel[10] = &Motor11;
  arrDynamixel[11] = &Motor12;
  arrDynamixel[12] = &Motor13;
  arrDynamixel[13] = &Motor14;
  arrDynamixel[14] = &Motor15;
  arrDynamixel[15] = &Motor16;
  arrDynamixel[16] = &Motor17;
  arrDynamixel[17] = &Motor18;

  for(int i = 0; i < 18; i++){
	  Dynamixel_SetGoalVelocity(arrDynamixel[i], 100);
	  Dynamixel_SetCWComplianceSlope(arrDynamixel[i], 4);
	  Dynamixel_SetCCWComplianceSlope(arrDynamixel[i], 4);
  }

//  Dynamixel_SetCWComplianceSlope(arrDynamixel[MOTOR8], 1);
//  Dynamixel_SetCCWComplianceSlope(arrDynamixel[MOTOR8], 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  double intg_ax = 0.0, intg_ay = 0.0, intg_az = 0.0;
	  double intg_gx = 0.0, intg_gy = 0.0, intg_gz = 0.0;
	  double intg_qx = 0.0, intg_qy = 0.0, intg_qz = 0.0;
	  double last_ax = 0.0, last_ay = 0.0, last_az = 0.0;
	  double last_qx = 0.0, last_qy = 0.0, last_qz = 0.0;
	  double kp_ax = -10.0, kp_ay = -8.0, kp_az = 0.0, kp_qx = -500.0, kp_qy = 0.0, kp_qz = 0.0;
	  double ki_ax = -1.0, ki_ay = -4.0, ki_az = 0.0, ki_qx = -500.0, ki_qy = 0.0, ki_qz = 0.0;
	  double kd_ax = -0.01, kd_ay = -0.00, kd_az = 0.0, kd_qx = -1.0, kd_qy = 0.0, kd_qz = 0.0;
	  double motor_perturb[12];

	  const double MAX_PERTURB = 15;
	  for(int j = 0; j < SIZE; j ++){
		  getIMU();

		  integrate(&intg_gx, ((double)IMUdata._X_GYRO)/32768.0);

		  double PID_ax = 0;//PID(((double)IMUdata._X_ACCEL)/32768.0, &intg_ax, &last_ax, kp_ax, ki_ax, kd_ax);
		  double PID_ay = 0;//PID(((double)IMUdata._Y_ACCEL)/32768.0, &intg_ay, &last_ay, kp_ay, ki_ay, kd_ay);
		  double PID_qx = PID(intg_gx, &intg_qx, &last_qx, kp_qx, ki_qx, kd_qx);

		  motor_perturb[MOTOR1]  =  ((PID_ax < MAX_PERTURB ? PID_ax : MAX_PERTURB) > -MAX_PERTURB ? PID_ax : -MAX_PERTURB);
		  motor_perturb[MOTOR2]  = -((PID_ay - PID_qx < MAX_PERTURB ? PID_ay - PID_qx : MAX_PERTURB) > -MAX_PERTURB ? PID_ay - PID_qx : -MAX_PERTURB);
		  motor_perturb[MOTOR3]  = 0;
		  motor_perturb[MOTOR4]  = -((PID_ay - PID_qx < MAX_PERTURB ? PID_ay - PID_qx : MAX_PERTURB) > -MAX_PERTURB ? PID_ay - PID_qx : -MAX_PERTURB);
		  motor_perturb[MOTOR5]  =  ((PID_ax < MAX_PERTURB ? PID_ax : MAX_PERTURB) > -MAX_PERTURB ? PID_ax : -MAX_PERTURB);
		  motor_perturb[MOTOR6]  = 0;
		  motor_perturb[MOTOR7]  = 0;
		  motor_perturb[MOTOR8]  =  ((PID_ax < MAX_PERTURB ? PID_ax : MAX_PERTURB) > -MAX_PERTURB ? PID_ax : -MAX_PERTURB);
		  motor_perturb[MOTOR9]  =  ((PID_ay - PID_qx < MAX_PERTURB ? PID_ay - PID_qx : MAX_PERTURB) > -MAX_PERTURB ? PID_ay - PID_qx : -MAX_PERTURB);
		  motor_perturb[MOTOR10] = 0;
		  motor_perturb[MOTOR11] =  ((PID_ay - PID_qx < MAX_PERTURB ? PID_ay - PID_qx : MAX_PERTURB) > -MAX_PERTURB ? PID_ay - PID_qx : -MAX_PERTURB);
		  motor_perturb[MOTOR12] =  ((PID_ax < MAX_PERTURB ? PID_ax : MAX_PERTURB) > -MAX_PERTURB ? PID_ax : -MAX_PERTURB);

		  for(int i = MOTOR1; i <= MOTOR12; i++){ // NB: i begins at 0 (i.e. Motor1 corresponds to i = 0)
			  switch(i){
				  case MOTOR1:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 - 1 + motor_perturb[MOTOR1]);
								  break;
				  case MOTOR2:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 + 3 + motor_perturb[MOTOR2]);
								  break;
				  case MOTOR3:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 + 1 + motor_perturb[MOTOR3]);
								  break;
				  case MOTOR4:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 + 2 + motor_perturb[MOTOR4]);
								  break;
				  case MOTOR5:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 - 0 + motor_perturb[MOTOR5]);
								  break;
				  case MOTOR6:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 + 0 + motor_perturb[MOTOR6]);
								  break;
				  case MOTOR7:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 + 0 + motor_perturb[MOTOR7]);
								  break;
				  case MOTOR8:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 - 3 + motor_perturb[MOTOR8]);
								  break;
				  case MOTOR9:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 - 0 + motor_perturb[MOTOR9]);
								  break;
				  case MOTOR10:	  Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 + 4 + motor_perturb[MOTOR10]);
								  break;
				  case MOTOR11:   Dynamixel_SetGoalPosition(arrDynamixel[i], MOTORANGLES[i][j]*180/PI + 150 + 1 + motor_perturb[MOTOR11]);
								  break;
				  case MOTOR12:	  Dynamixel_SetGoalPosition(arrDynamixel[i], -1*MOTORANGLES[i][j]*180/PI + 150 + 3 + motor_perturb[MOTOR12]);
								  break;
			  }
		  }
		  HAL_Delay(10); // Default from Lukas: 10
	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Supply configuration update enable 
    */
  MODIFY_REG(PWR->CR3, PWR_CR3_SCUEN, 0);

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while ((PWR->D3CR & (PWR_D3CR_VOSRDY)) != PWR_D3CR_VOSRDY) 
  {
    
  }
    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.I2c123ClockSelection = RCC_I2C123CLKSOURCE_D2PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(SystemCoreClock/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
