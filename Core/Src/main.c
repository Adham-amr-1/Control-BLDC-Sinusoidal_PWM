/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Macros For 6 Timer Channels to be more readable
#define PHASE_A TIM_CHANNEL_1
#define PHASE_B TIM_CHANNEL_2
#define PHASE_C TIM_CHANNEL_3
// Macros for Throttle Range
#define MIN_THROTTLE 0
#define MAX_THROTTLE 3560
// Macros For PWM Signal Range
#define MAX_PWM 1439 // 72MHz / 1440 = 50KHz
#define MIN_PWM 0
// Macro For Dead Time Period
#define DEADTIME_DELAY 20 // Real Deadtime in microsecond
// Macro For indicating that Hall in invalid state to avoid any damage
#define INVALID_STATE 255
// Size of Sine Look Up Table
#define SINE_TABLE_SIZE 1024
// Macro for Filter Size of ADC
#define ADC_FILTER_SIZE   254  // Size of the moving average window

#define MAX_INTERPOL_COUNTER 65535

// Static Global Variables for ADC Filter
static uint16_t adcSamples[ADC_FILTER_SIZE];
static uint8_t  adcIndex = 0;
// De ma3mola 3andohm in test we will know if it matter
#define MAX(a, b) ((a)>(b)?(a):(b)) // MAX(0,-2) 0>-2? 0:0
#define MIN(a, b) ((a)<(b)?(a):(b))

#define SECTOR_ANGLE_DEG   60.0f

// LookUp Table to generate Sin Wave
// Global sine table
uint16_t sine_LUT[SINE_TABLE_SIZE]= {
		720, 724, 728, 733, 737, 742, 746, 750, 755, 759, 764, 768, 772, 777, 781,
		786, 790, 794, 799, 803, 808, 812, 816, 821, 825, 829, 834, 838, 843, 847,
		851, 856, 860, 864, 869, 873, 877, 881, 886, 890, 894, 899, 903, 907, 911,
		916, 920, 924, 928, 933, 937, 941, 945, 949, 954, 958, 962, 966, 970, 974,
		978, 983, 987, 991, 995, 999, 1003, 1007, 1011, 1015, 1019, 1023, 1027, 1031,
		1035, 1039, 1043, 1047, 1051, 1055, 1059, 1063, 1066, 1070, 1074, 1078, 1082,
		1086, 1089, 1093, 1097, 1101, 1104, 1108, 1112, 1116, 1119, 1123, 1127, 1130,
		1134, 1137, 1141, 1145, 1148, 1152, 1155, 1159, 1162, 1166, 1169, 1173, 1176,
		1179, 1183, 1186, 1189, 1193, 1196, 1199, 1203, 1206, 1209, 1212, 1216, 1219,
		1222, 1225, 1228, 1231, 1234, 1238, 1241, 1244, 1247, 1250, 1253, 1256, 1259,
		1261, 1264, 1267, 1270, 1273, 1276, 1278, 1281, 1284, 1287, 1289, 1292, 1295,
		1297, 1300, 1303, 1305, 1308, 1310, 1313, 1315, 1318, 1320, 1323, 1325, 1327,
		1330, 1332, 1334, 1337, 1339, 1341, 1343, 1346, 1348, 1350, 1352, 1354, 1356,
		1358, 1360, 1362, 1364, 1366, 1368, 1370, 1372, 1374, 1375, 1377, 1379, 1381,
		1383, 1384, 1386, 1388, 1389, 1391, 1392, 1394, 1395, 1397, 1398, 1400, 1401,
		1403, 1404, 1405, 1407, 1408, 1409, 1411, 1412, 1413, 1414, 1415, 1416, 1417,
		1418, 1420, 1421, 1422, 1422, 1423, 1424, 1425, 1426, 1427, 1428, 1428, 1429,
		1430, 1431, 1431, 1432, 1432, 1433, 1434, 1434, 1435, 1435, 1436, 1436, 1436,
		1437, 1437, 1437, 1438, 1438, 1438, 1438, 1439, 1439, 1439, 1439, 1439, 1439,
		1439, 1439, 1439, 1439, 1439, 1439, 1439, 1438, 1438, 1438, 1438, 1437, 1437,
		1437, 1436, 1436, 1436, 1435, 1435, 1434, 1434, 1433, 1432, 1432, 1431, 1431,
		1430, 1429, 1428, 1428, 1427, 1426, 1425, 1424, 1423, 1422, 1422, 1421, 1420,
		1418, 1417, 1416, 1415, 1414, 1413, 1412, 1411, 1409, 1408, 1407, 1405, 1404,
		1403, 1401, 1400, 1398, 1397, 1395, 1394, 1392, 1391, 1389, 1388, 1386, 1384,
		1383, 1381, 1379, 1377, 1375, 1374, 1372, 1370, 1368, 1366, 1364, 1362, 1360,
		1358, 1356, 1354, 1352, 1350, 1348, 1346, 1343, 1341, 1339, 1337, 1334, 1332,
		1330, 1327, 1325, 1323, 1320, 1318, 1315, 1313, 1310, 1308, 1305, 1303, 1300,
		1297, 1295, 1292, 1289, 1287, 1284, 1281, 1278, 1276, 1273, 1270, 1267, 1264,
		1261, 1259, 1256, 1253, 1250, 1247, 1244, 1241, 1238, 1234, 1231, 1228, 1225,
		1222, 1219, 1216, 1212, 1209, 1206, 1203, 1199, 1196, 1193, 1189, 1186, 1183,
		1179, 1176, 1173, 1169, 1166, 1162, 1159, 1155, 1152, 1148, 1145, 1141, 1137,
		1134, 1130, 1127, 1123, 1119, 1116, 1112, 1108, 1104, 1101, 1097, 1093, 1089,
		1086, 1082, 1078, 1074, 1070, 1066, 1063, 1059, 1055, 1051, 1047, 1043, 1039,
		1035, 1031, 1027, 1023, 1019, 1015, 1011, 1007, 1003, 999, 995, 991, 987, 983,
		978, 974, 970, 966, 962, 958, 954, 949, 945, 941, 937, 933, 928, 924, 920, 916,
		911, 907, 903, 899, 894, 890, 886, 881, 877, 873, 869, 864, 860, 856, 851, 847,
		843, 838, 834, 829, 825, 821, 816, 812, 808, 803, 799, 794, 790, 786, 781, 777,
		772, 768, 764, 759, 755, 750, 746, 742, 737, 733, 728, 724, 720, 715, 711, 706,
		702, 697, 693, 689, 684, 680, 675, 671, 667, 662, 658, 653, 649, 645, 640, 636,
		631, 627, 623, 618, 614, 610, 605, 601, 596, 592, 588, 583, 579, 575, 570, 566,
		562, 558, 553, 549, 545, 540, 536, 532, 528, 523, 519, 515, 511, 506, 502, 498,
		494, 490, 485, 481, 477, 473, 469, 465, 461, 456, 452, 448, 444, 440, 436, 432,
		428, 424, 420, 416, 412, 408, 404, 400, 396, 392, 388, 384, 380, 376, 373, 369,
		365, 361, 357, 353, 350, 346, 342, 338, 335, 331, 327, 323, 320, 316, 312, 309,
		305, 302, 298, 294, 291, 287, 284, 280, 277, 273, 270, 266, 263, 260, 256, 253,
		250, 246, 243, 240, 236, 233, 230, 227, 223, 220, 217, 214, 211, 208, 205, 201,
		198, 195, 192, 189, 186, 183, 180, 178, 175, 172, 169, 166, 163, 161, 158, 155,
		152, 150, 147, 144, 142, 139, 136, 134, 131, 129, 126, 124, 121, 119, 116, 114,
		112, 109, 107, 105, 102, 100, 98, 96, 93, 91, 89, 87, 85, 83, 81, 79, 77, 75,
		73, 71, 69, 67, 65, 64, 62, 60, 58, 56, 55, 53, 51, 50, 48, 47, 45, 44, 42, 41,
		39, 38, 36, 35, 34, 32, 31, 30, 28, 27, 26, 25, 24, 23, 22, 21, 19, 18, 17, 17,
		16, 15, 14, 13, 12, 11, 11, 10, 9, 8, 8, 7, 7, 6, 5, 5, 4, 4, 3, 3, 3, 2, 2, 2,
		1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3,
		4, 4, 5, 5, 6, 7, 7, 8, 8, 9, 10, 11, 11, 12, 13, 14, 15, 16, 17, 17, 18, 19, 21,
		22, 23, 24, 25, 26, 27, 28, 30, 31, 32, 34, 35, 36, 38, 39, 41, 42, 44, 45, 47,
		48, 50, 51, 53, 55, 56, 58, 60, 62, 64, 65, 67, 69, 71, 73, 75, 77, 79, 81, 83,
		85, 87, 89, 91, 93, 96, 98, 100, 102, 105, 107, 109, 112, 114, 116, 119, 121,
		124, 126, 129, 131, 134, 136, 139, 142, 144, 147, 150, 152, 155, 158, 161, 163,
		166, 169, 172, 175, 178, 180, 183, 186, 189, 192, 195, 198, 201, 205, 208, 211,
		214, 217, 220, 223, 227, 230, 233, 236, 240, 243, 246, 250, 253, 256, 260, 263,
		266, 270, 273, 277, 280, 284, 287, 291, 294, 298, 302, 305, 309, 312, 316, 320,
		323, 327, 331, 335, 338, 342, 346, 350, 353, 357, 361, 365, 369, 373, 376, 380,
		384, 388, 392, 396, 400, 404, 408, 412, 416, 420, 424, 428, 432, 436, 440, 444,
		448, 452, 456, 461, 465, 469, 473, 477, 481, 485, 490, 494, 498, 502, 506, 511,
		515, 519, 523, 528, 532, 536, 540, 545, 549, 553, 558, 562, 566, 570, 575, 579,
		583, 588, 592, 596, 601, 605, 610, 614, 618, 623, 627, 631, 636, 640, 645, 649,
		653, 658, 662, 667, 671, 675, 680, 684, 689, 693, 697, 702, 706, 711, 715
};


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Global Variables For Debugging
uint8_t  HALL_GLOBAL = 0;
uint16_t ADC_BEFORE_SAMPLING = 0;
uint16_t ADC_AFTER_SAMPLING = 0;
uint16_t ADC_MAPPING = 0;
// Global Variable For Motor Position
uint8_t  Recent_state = 0;
uint8_t  Prev_state = INVALID_STATE;
// Debugging Theta
uint16_t THETA;
uint16_t THETA_A;
uint16_t THETA_B;
uint16_t THETA_C;

float FRAC;

static const uint16_t phase_shift = 120; // 120° in 1024-point table (1024/3 ≈ 341)

// Global variables for hall interpolation
uint8_t current_hall = 0;

uint8_t last_hall = 0;

uint32_t now = 0;
uint32_t last_time = 0;

uint32_t delta_time = 1000; // avoid division by zero

float base_angle = 0.0f;
float prev_base_angle = 0.0f;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Read_Hall_Sensors(void)
{
	// Read Hall Sensors FeedBack Signal
	uint8_t h3= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	uint8_t h2= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	uint8_t h1= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
	// Shift Values to be in one variable for Checking
	uint8_t hall_value = h3<<2 | h2<<1 | h1<<0;
	// Check Values of Hall Sensor Reading ( For Debugging )
	HALL_GLOBAL = hall_value;
	return hall_value;
}

void Disable_Switches()
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	HAL_TIM_PWM_Stop(&htim1, PHASE_A);  // A_HIGH A8
	HAL_TIM_PWM_Stop(&htim1, PHASE_B);  // B_HIGH A9
	HAL_TIM_PWM_Stop(&htim1, PHASE_C);  // C_HIGH A10

	HAL_TIMEx_PWMN_Stop(&htim1, PHASE_A); // A_LOW A7
	HAL_TIMEx_PWMN_Stop(&htim1, PHASE_B); // B_LOW B0
	HAL_TIMEx_PWMN_Stop(&htim1, PHASE_C); // C_LOW B1
}

uint16_t Throttle_mapping(uint16_t Value, uint16_t ADC_Min, uint16_t ADC_Max, uint16_t Min_PWM, uint16_t Max_PWM)
{
	if(Value > ADC_Max)
	{
		Value = ADC_Max;
	}
	else if (Value < ADC_Min)
	{
		Value = ADC_Min;
	}
	return (uint16_t)((Value - ADC_Min) * (Max_PWM - Min_PWM) / (ADC_Max - ADC_Min) + Min_PWM);
}

void Step1()
{
	HAL_TIM_PWM_Start(&htim1, PHASE_A); 	  // A_HIGH A8

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_B);    // B_LOW B0
	HAL_TIMEx_PWMN_Start(&htim1, PHASE_C);    // C_LOW B1
}

void Step2()
{
	HAL_TIM_PWM_Start(&htim1, PHASE_A);       // A_HIGH A0
	HAL_TIM_PWM_Start(&htim1, PHASE_B);       // B_HIGH A1

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_C);    // C_LOW B1
}

void Step3()
{
	HAL_TIM_PWM_Start(&htim1, PHASE_B);       // B_HIGH A1

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_A);    // A_LOW A7
	HAL_TIMEx_PWMN_Start(&htim1, PHASE_C);    // C_LOW B1
}

void Step4()
{
	HAL_TIM_PWM_Start(&htim1, PHASE_B);      // B_HIGH A1
	HAL_TIM_PWM_Start(&htim1, PHASE_C);      // C_HIGH A2

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_A);   // A_LOW A7
}

void Step5()
{
	HAL_TIM_PWM_Start(&htim1, PHASE_C);        // C_HIGH A2

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_B);     // B_LOW B0
	HAL_TIMEx_PWMN_Start(&htim1, PHASE_A);     // A_LOW A7
}

void Step6()
{
	HAL_TIM_PWM_Start(&htim1, PHASE_A);        // A_HIGH A0
	HAL_TIM_PWM_Start(&htim1, PHASE_C);        // C_HIGH A2

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_B);     // B_LOW A6
}

// Assumes 72 MHz system clock
void deadTimeDelay(uint32_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0); // Reset the timer counter
	while (__HAL_TIM_GET_COUNTER(&htim2) < us); // Wait until the counter reaches the delay value
}

void safeDisableOutputs(uint16_t DeadTime)
{
	Disable_Switches();
//	HAL_Delay(500);
	deadTimeDelay(DeadTime);
}

// Hall to angle mapping
// Hall to base electrical angle (deg)
static float HallToAngle(uint8_t hall)
{
    switch (hall)
    {
        case 0b101: return 0.0f;
        case 0b100: return 60.0f;
        case 0b110: return 120.0f;
        case 0b010: return 180.0f;
        case 0b011: return 240.0f;
        case 0b001: return 300.0f;
        default:    return -1.0f; // invalid
    }
}

// Poll hall sensors and update sector timing
static void PollHallSensors(void)
{
    uint8_t new_hall = Read_Hall_Sensors();
    if (new_hall == current_hall)
    {
    	return; // no change
    }

    float ang = HallToAngle(new_hall);

    if (ang < 0.0f)
    {
    	return; // invalid state; ignore
    }

    now = __HAL_TIM_GET_COUNTER(&htim3);

    delta_time = (now >= last_time) ? (now - last_time) : (0xFFFFFFFF - last_time + now + 1);

    if (delta_time == 0)
    {
    	delta_time = 1; // protect
    }
    last_time = now;

    last_hall = current_hall;
    current_hall = new_hall;

    prev_base_angle = base_angle;
    base_angle = ang; // new sector base angle (0..300 step 60)
}

// Interpolated angle calculation
static float GetInterpolatedAngle(void)
{
    now = __HAL_TIM_GET_COUNTER(&htim3);

    uint32_t elapsed = (now >= last_time) ? (now - last_time) : (0xFFFFFFFF - last_time + now + 1);

    float fraction = (float)elapsed / (float)delta_time;

    if (fraction > 1.0f)
    {
    	fraction = 1.0f;
    }
    if (fraction < 0.0f)
    {
        	fraction = 0.0f;
    }
    FRAC = fraction;

    // Interpolate only across 60 electrical degrees (Hall sector width)
    float angle = base_angle + (fraction * SECTOR_ANGLE_DEG);
//    if (angle >= 360.0f) angle -= 360.0f; //Testing Something
    THETA = (uint16_t)angle;
    return angle;
}

void Control_BLDC(uint16_t ADC_TO_PWM , uint8_t hall)
{

	// Update Hall timing
	PollHallSensors();

	deadTimeDelay(1);

	float angle_deg = GetInterpolatedAngle();


	// Convert degrees -> LUT index
	uint16_t base_idx = (uint16_t)((angle_deg * (float)SINE_TABLE_SIZE) / 360.0f) % SINE_TABLE_SIZE;


	uint16_t idxA = base_idx;
	uint16_t idxB = (base_idx + phase_shift) % SINE_TABLE_SIZE;
	uint16_t idxC = (base_idx + 2*phase_shift) % SINE_TABLE_SIZE;
	THETA_A = idxA;
	THETA_B = idxB;
	THETA_C = idxC;

	// Scale LUT (0..MAX_PWM) by pwm_mag (0..MAX_PWM) => 0..MAX_PWM
	uint16_t dutyA = (uint16_t)(((uint32_t)sine_LUT[idxA] * ADC_TO_PWM) / MAX_PWM);
	uint16_t dutyB = (uint16_t)(((uint32_t)sine_LUT[idxB] * ADC_TO_PWM) / MAX_PWM);
	uint16_t dutyC = (uint16_t)(((uint32_t)sine_LUT[idxC] * ADC_TO_PWM) / MAX_PWM);

	dutyA = MIN(MAX_PWM, MAX(MIN_PWM, dutyA));
	dutyB = MIN(MAX_PWM, MAX(MIN_PWM, dutyB));
	dutyC = MIN(MAX_PWM, MAX(MIN_PWM, dutyC));

	TIM1->CCR1 = dutyA; // A
	TIM1->CCR2 = dutyB; // B
	TIM1->CCR3 = dutyC; // C

	  switch (hall)
	  {
	    case 0b101:  // Hall = 5 → Step1
	      Step1();
	      break;
	    case 0b100:  // Step2
	      Step2();
	      break;
	    case 0b110:  // Step3
	      Step3();
	      break;
	    case 0b010:  // Step4
	      Step4();
	      break;
	    case 0b011:  // Step5
	      Step5();
	      break;
	    case 0b001:  // Step6
	      Step6();
	      break;
	    default:
	      Disable_Switches();
	      break;
	  }
//	   Disable Switches for Dead Time to make sure that High & Low of each phase won't open together ( For Safety )
//	  safeDisableOutputs(DEADTIME_DELAY);
}

uint16_t ADC_Sampling(uint16_t ADC_Before)
{
	// Insert new sample into moving-average buffer
	adcSamples[adcIndex] = ADC_Before;
	adcIndex = (adcIndex + 1) % ADC_FILTER_SIZE;

	// Compute average of the buffer
	uint32_t sum = 0;
	for(uint16_t i = 0; i < ADC_FILTER_SIZE; i++) {
		sum += adcSamples[i];
	}
	return (uint16_t)(sum / ADC_FILTER_SIZE);
}

uint16_t StartADC(uint32_t TimeOut)
{
	// Start and Convert Analog Throttle Signal To Digital To be handled
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, TimeOut);
	return HAL_ADC_GetValue(&hadc1);
}

void CheckState(uint8_t HALL_State)
{
	  if( Recent_state != Prev_state || Recent_state == INVALID_STATE )
	  {
		  safeDisableOutputs(DEADTIME_DELAY); // Unified handling for state change or invalid state
		  if(Recent_state != INVALID_STATE)
		  {
			  Prev_state = HALL_State; // Only update old_state if valid
		  }
	  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  uint16_t ADCValue;
  uint16_t AdcMapp;
  uint16_t ADC_Sampled;
  uint8_t hall;

  HAL_TIM_Base_Start(&htim3); // timing for interpolation
  HAL_TIM_Base_Start(&htim2); // us delay timer

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Start and read ADC (once)
	  ADCValue = StartADC(100);
	  // Check Values of ADC Before Sampling ( For Debugging )
	  ADC_BEFORE_SAMPLING = ADCValue;
	  //Sample / Filter The ADC Values to check the best one and reduce the noise
	  ADC_Sampled = ADC_Sampling(ADCValue);
	  // Check That Max & Min Values of sampled within Range of Throttle
	  if(ADC_Sampled < MIN_THROTTLE)
	  {
		  ADC_Sampled = MIN_THROTTLE;
	  }
	  if (ADC_Sampled > MAX_THROTTLE)
	  {
		  ADC_Sampled = MAX_THROTTLE;
	  }
	  // Check Values of ADC After Sampling ( For Debugging )
	  ADC_AFTER_SAMPLING = ADC_Sampled;
	  // Mapping the Throttle Value from Throttle Rang to Used PWM Range To Control Duty Cycle of PWM
	  AdcMapp = Throttle_mapping(ADC_Sampled, MIN_THROTTLE, MAX_THROTTLE, MIN_PWM, MAX_PWM);
	  // Check Values of Throttle Mapping ( For Debugging )
	  ADC_MAPPING = AdcMapp;
	  // Read Hall Sensor State ( Hall Sensor Feedback )
	  hall = Read_Hall_Sensors();
	  Recent_state = hall;
	  // Check Hall State
	  CheckState(hall);
	  /*
	   * ControlLing BLDC Motor By using two parameters ( Arguments )
	   * AdcMapp -> Control the speed of Motor and the switching of each MOSFETS
	   * hall -> FeedBack of Hall indicate the rotor position to take the right action ( Step ) to control motor
	   *
	   */
	  Control_BLDC(AdcMapp, hall);

	  //HAL_Delay(50);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1439;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
