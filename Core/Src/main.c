/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define PCA9685_MODE1         0x0
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x6
#define PCA9685_MODE1_SLEEP_BIT      4
#define PCA9685_MODE1_AI_BIT         5
#define PCA9685_MODE1_RESTART_BIT    7
double joy_x = 128.0;
double joy_y = 128.0;
double xr = 0.0;
double yr = 0.0;
int spf = 87;
int spr = 77;
int l1 = 50;
int l2 = 20;
int l3 = 100;
int l4 = 100;
//double _servo_offsets[12] = { 175, 90, 90, 10, 95, 98, 170, 93, 90, 1, 107, 90 };
double _servo_offsets[12] = { 180, 105, 83, 6, 77, 87, 182, 90, 87, -4, 90, 85 };
//double _servo_offsets[12] = { 173, 95, 84, 2, 79.4, 94, 166, 103, 85, -10, 91, 83 };
double _val_list[12];
double _thetas[4][3];

void PCA9685_SetBit(I2C_HandleTypeDef* hi2c, uint8_t address, uint8_t Register, uint8_t Bit, uint8_t Value)
{
    uint8_t readValue;
    HAL_I2C_Mem_Read(hi2c, address, Register, 1, &readValue, 1, 10);
    if (Value == 0) readValue &= ~(1 << Bit);
    else readValue |= (1 << Bit);
    HAL_I2C_Mem_Write(hi2c, address, Register, 1, &readValue, 1, 10);
    //HAL_Delay(1);
}

void PCA9685_SetPWMFrequency(I2C_HandleTypeDef* hi2c, uint8_t address, uint16_t frequency)
{
    uint8_t prescale;
    if (frequency >= 1526) prescale = 0x03;
    else if (frequency <= 24) prescale = 0xFF;
    else prescale = 25000000 / (4096 * frequency);

    PCA9685_SetBit(hi2c, address, PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
    HAL_I2C_Mem_Write(hi2c, address, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
    PCA9685_SetBit(hi2c, address, PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
    PCA9685_SetBit(hi2c, address, PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

void PCA9685_Init(I2C_HandleTypeDef* hi2c, uint8_t address, uint16_t frequency)
{
    PCA9685_SetPWMFrequency(hi2c, address, frequency);
    PCA9685_SetBit(hi2c, address, PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

void PCA9685_SetPWM(I2C_HandleTypeDef* hi2c, uint8_t address, uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
    uint8_t registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
    uint8_t pwm[4];
    pwm[0] = OnTime & 0xFF;
    pwm[1] = OnTime >> 8;
    pwm[2] = OffTime & 0xFF;
    pwm[3] = OffTime >> 8;
    HAL_I2C_Mem_Write(hi2c, address, registerAddress, 1, pwm, 4, 10);
}

void PCA9685_SetServoAngle(I2C_HandleTypeDef* hi2c, uint8_t address, uint8_t Channel, float Angle)
{
    float Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
    PCA9685_SetPWM(hi2c, address, Channel, 0, (uint16_t)Value);
}

void mat_mult(double result[4][4], double mat1[4][4], double mat2[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i][j] = 0.0;
            for (int k = 0; k < 4; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void mat_mult_vec(double result[4], double mat[4][4], double vec[4]) {
    for (int i = 0; i < 4; i++) {
        result[i] = 0.0;
        for (int j = 0; j < 4; j++) {
            result[i] += mat[i][j] * vec[j];
        }
    }
}

void calcLeg(double t, double x, double y, double z, double result[4]) {
    double Sl = -30; //(-)10 Straight, (+)10 Back
    double Sw = 0; //(-)5 Left, (+)5 Right
    double Sa = 0; //(-)3 Right Turn, (+)3 Left Turn
    double Sh = 40; // 100
    double t0 = 000;
    double t1 = 1200;
    double t2 = 500;
    double t3 = 100;

    double startLp[4] = { x - Sl / 2.0, y, z - Sw, 1.0 };
    double endY = 0;
    double endLp[4] = { x + Sl / 2.0, y + endY, z + Sw, 1.0 };

    if (t < t0) {
        for (int i = 0; i < 4; i++) {
            result[i] = startLp[i];
        }
    }
    else if (t < t0 + t1) {
        double td = t - t0;
        double tp = 1.0 / (t1 / td);  // Python과 동일하게 계산

        double diffLp[4];
        for (int i = 0; i < 4; i++) {
            diffLp[i] = endLp[i] - startLp[i];
        }

        double curLp[4];
        for (int i = 0; i < 4; i++) {
            curLp[i] = startLp[i] + diffLp[i] * tp;
        }

        double psi = -((M_PI / 180.0 * Sa) / 2.0) + (M_PI / 180.0 * Sa) * tp;

        double Ry[4][4] = {
            {cos(psi), 0.0, sin(psi), 0.0},
            {0.0, 1.0, 0.0, 0.0},
            {-sin(psi), 0.0, cos(psi), 0.0},
            {0.0, 0.0, 0.0, 1.0}
        };

        double rotatedLp[4];
        for (int i = 0; i < 4; i++) {
            rotatedLp[i] = 0.0;
            for (int j = 0; j < 4; j++) {
                rotatedLp[i] += Ry[i][j] * curLp[j];
            }
        }

        for (int i = 0; i < 4; i++) {
            result[i] = rotatedLp[i];
        }
    }
    else if (t < t0 + t1 + t2) {
        for (int i = 0; i < 4; i++) {
            result[i] = endLp[i];
        }
    }
    else if (t < t0 + t1 + t2 + t3) {
        double td = t - (t0 + t1 + t2);
        double tp = 1.0 / (t3 / td);

        double diffLp[4];
        for (int i = 0; i < 4; i++) {
            diffLp[i] = startLp[i] - endLp[i];
        }

        double curLp[4];
        for (int i = 0; i < 4; i++) {
            curLp[i] = endLp[i] + diffLp[i] * tp;
        }

        curLp[1] += Sh * sin(M_PI * tp);

        for (int i = 0; i < 4; i++) {
            result[i] = curLp[i];
        }
    }
}

double normalize_mod(double value, double modulus) {
    double result = fmod(value, modulus);
    if (result < 0) {
        result += modulus;
    }
    return result;
}

void positions(double t, double result[4][4]) {
    double spf = 87;
    double spr = 77;

    double t0 = 000;
    double t1 = 1200;
    double t2 = 500;
    double t3 = 100;

    double Tt = (t0 + t1 + t2 + t3);
    double Tt2 = Tt / 2;
    double rd = 0; // rear delta - unused - maybe stupid
    //double Tt4 = Tt / 4;
    double td = normalize_mod(t, Tt);   //front left
    double t2_mod = normalize_mod(t  + Tt2, Tt);   //front right
    double rtd = normalize_mod(t + rd, Tt);     //rear left
    double rt2 = normalize_mod(t + Tt2 + rd, Tt);   //rear right
    //double td = normalize_mod(t + Tt4 * 3, Tt);
    //double t2_mod = normalize_mod(t + Tt4 * 1, Tt);
    //double rtd = normalize_mod(t + Tt4 * 2, Tt);
    //double rt2 = normalize_mod(t, Tt);

    double Fx = 120;
    double Rx = -50;
    double Fy = -100;
    double Ry = -100;

    calcLeg(td, Fx, Fy, spf, result[0]);
    calcLeg(t2_mod, Fx, Fy, -spf, result[1]);
    calcLeg(rt2, Rx, Ry, spr, result[2]);
    calcLeg(rtd, Rx, Ry, -spr, result[3]);
    //HAL_Delay(10);

}

void legIK(double point[4], double result[3]) {
    double x = point[0];
    double y = point[1];
    double z = point[2];

    double F;
    double G;
    double H;
    double theta1;
    double theta2;
    double theta3;
    double D;

    F = sqrt(fmax(0.0, pow(x, 2) + pow(y, 2) - pow(l1, 2)));

    G = F - l2;
    H = sqrt(pow(G, 2) + pow(z, 2));

    double epsilon = 1e-6;
    theta1 = -atan2(y + epsilon, x + epsilon) - atan2(F, -l1 + epsilon);

    D = (pow(H, 2) - pow(l3, 2) - pow(l4, 2)) / (2 * l3 * l4);

    if (D < -1.0 || D > 1.0) {
        theta3 = 0;
    }
    else {
        theta3 = acos(D);
    }

    theta2 = atan2(z, G) - atan2(l4 * sin(theta3), l3 + l4 * cos(theta3));

    result[0] = theta1;
    result[1] = theta2;
    result[2] = theta3;
}

void bodyIK(double omega, double phi, double psi, double xm, double ym, double zm, double result[4][4][4]) {
    double Rx[4][4] = {
        {1, 0, 0, 0},
        {0, cos(omega), -sin(omega), 0},
        {0, sin(omega), cos(omega), 0},
        {0, 0, 0, 1}
    };

    double Ry[4][4] = {
        {cos(phi), 0, sin(phi), 0},
        {0, 1, 0, 0},
        {-sin(phi), 0, cos(phi), 0},
        {0, 0, 0, 1}
    };

    double Rz[4][4] = {
        {cos(psi), -sin(psi), 0, 0},
        {sin(psi), cos(psi), 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    // Rxyz = Rx * (Ry * Rz)
    double RyRz[4][4];
    double Rxyz[4][4];
    mat_mult(RyRz, Ry, Rz);
    mat_mult(Rxyz, Rx, RyRz);

    double T[4][4] = {
        {1, 0, 0, xm},
        {0, 1, 0, ym},
        {0, 0, 1, zm},
        {0, 0, 0, 1}
    };

    double Tm[4][4];
    mat_mult(Tm, T, Rxyz);

    double sHp = sin(M_PI / 2);
    double cHp = cos(M_PI / 2);
    double L = 140.0;
    double W = 75.0;

    double leg1[4][4] = {
        {cHp, 0, sHp, L / 2},
        {0, 1, 0, 0},
        {-sHp, 0, cHp, W / 2},
        {0, 0, 0, 1}
    };
    double leg2[4][4] = {
        {cHp, 0, sHp, L / 2},
        {0, 1, 0, 0},
        {-sHp, 0, cHp, -W / 2},
        {0, 0, 0, 1}
    };
    double leg3[4][4] = {
        {cHp, 0, sHp, -L / 2},
        {0, 1, 0, 0},
        {-sHp, 0, cHp, W / 2},
        {0, 0, 0, 1}
    };
    double leg4[4][4] = {
        {cHp, 0, sHp, -L / 2},
        {0, 1, 0, 0},
        {-sHp, 0, cHp, -W / 2},
        {0, 0, 0, 1}
    };

    mat_mult(result[0], Tm, leg1);
    mat_mult(result[1], Tm, leg2);
    mat_mult(result[2], Tm, leg3);
    mat_mult(result[3], Tm, leg4);
}

void calcIK(double Lp[4][4], double angles[3], double center[3], double result[4][3]) {
    //double omega = angles[0];
    //double phi = angles[1];
    //double psi = angles[2];
    //double xm = center[0];
    //double ym = center[1];
    //double zm = center[2];
    double Ix[4][4] = {
        {-1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };

    // Precomputed inverses
    double inv_Tlf[4][4] = {
            { 6.123234e-17, -0.000000e+00, -1.000000e+00,  3.750000e+01 },
            { 0.000000e+00,  1.000000e+00,  0.000000e+00, -5.000000e+01 }, //height  == Tlf[1][3], all change
            { 1.000000e+00,  0.000000e+00,  6.123234e-17, -1.200000e+02 },
            { 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00 }
        };

        double inv_Trf[4][4] = {
            { 6.123234e-17, -0.000000e+00, -1.000000e+00, -3.750000e+01 },
            { 0.000000e+00,  1.000000e+00,  0.000000e+00, -5.000000e+01 },
            { 1.000000e+00,  0.000000e+00,  6.123234e-17, -1.200000e+02 },
            { 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00 }
        };

        double inv_Tlb[4][4] = {
            { 6.123234e-17, -0.000000e+00, -1.000000e+00,  3.750000e+01 },
            { 0.000000e+00,  1.000000e+00,  0.000000e+00, -5.000000e+01 },
            { 1.000000e+00,  0.000000e+00,  6.123234e-17,  2.000000e+01 },
            { 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00 }
        };

        double inv_Trb[4][4] = {
            { 6.123234e-17, -0.000000e+00, -1.000000e+00, -3.750000e+01 },
            { 0.000000e+00,  1.000000e+00,  0.000000e+00, -5.000000e+01 },
            { 1.000000e+00,  0.000000e+00,  6.123234e-17,  2.000000e+01 },
            { 0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00 }
        };

    double Lp_tmp[4];
    double Lp2_tmp[4];
    double ik_result[3];
    // Tlf
    mat_mult_vec(Lp_tmp, inv_Tlf, Lp[0]);

    legIK(Lp_tmp, ik_result);
    for (int i = 0; i < 3; i++) {
        result[0][i] = ik_result[i];
    }

    // Trf
    mat_mult_vec(Lp2_tmp, inv_Trf, Lp[1]);
    mat_mult_vec(Lp_tmp, Ix, Lp2_tmp);
    legIK(Lp_tmp, ik_result);
    for (int i = 0; i < 3; i++) {
        result[1][i] = ik_result[i];
    }

    // Tlb
    mat_mult_vec(Lp_tmp, inv_Tlb, Lp[2]);
    legIK(Lp_tmp, ik_result);
    for (int i = 0; i < 3; i++) {
        result[2][i] = ik_result[i];
    }

    // Trb
    mat_mult_vec(Lp2_tmp, inv_Trb, Lp[3]);
    mat_mult_vec(Lp_tmp, Ix, Lp2_tmp);  // Ix
    legIK(Lp_tmp, ik_result);
    for (int i = 0; i < 3; i++) {
        result[3][i] = ik_result[i];
    }
}

void getDegreeAngles(double La[4][3], double thetas[4][3]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++) {
            La[i][j] *= 180.0 / M_PI;
            thetas[i][j] = (double)La[i][j];
        }
    }
}

void angleToServo(double La[4][3]) {
    getDegreeAngles((double(*)[3])La, _thetas);

    // FL Lower
    _val_list[0] = _servo_offsets[0] - _thetas[0][2];
    // FL Upper
    _val_list[1] = _servo_offsets[1] - _thetas[0][1];
    // FL Shoulder
    _val_list[2] = _servo_offsets[2] + _thetas[0][0];

    // FR Lower
    _val_list[3] = _servo_offsets[3] + _thetas[1][2];
    // FR Upper
    _val_list[4] = _servo_offsets[4] + _thetas[1][1];
    // FR Shoulder
    _val_list[5] = _servo_offsets[5] - _thetas[1][0];

    // BL Lower
    _val_list[6] = _servo_offsets[6] - _thetas[2][2];
    // BL Upper
    _val_list[7] = _servo_offsets[7] - _thetas[2][1];
    // BL Shoulder
    _val_list[8] = _servo_offsets[8] - _thetas[2][0];

    // BR Lower
    _val_list[9] = _servo_offsets[9] + _thetas[3][2];
    // BR Upper
    _val_list[10] = _servo_offsets[10] + _thetas[3][1];
    // BR Shoulder
    _val_list[11] = _servo_offsets[11] + _thetas[3][0];
}

void servoRotate(double thetas[4][3]) {
    angleToServo(thetas);

    for (int x = 0; x < 12; x++) {
        //_val_list[x] = (int)((_val_list[x] - 26.36) * (1980.0 / 1500.0));

        if (_val_list[x] > 180) {
            _val_list[x] = 179;
            continue;
        }
        else if (_val_list[x] <= 0) {
            _val_list[x] = 1;
            continue;
        }
        if (x < 6){
        	PCA9685_SetServoAngle(&hi2c1, 0x82, x, _val_list[x]);
        }
        else {
        	PCA9685_SetServoAngle(&hi2c1, 0x80, x-6, _val_list[x]);
        }
    }
}
void positions(double d, double result[4][4]);
void calcIK(double Lp[4][4], double angles[3], double center[3], double result[4][3]);
void servoRotate(double thetas[4][3]);
void angleToServo(double La[4][3]);
void mat_mult(double result[4][4], double mat1[4][4], double mat2[4][4]);
void mat_mult_vec(double result[4], double mat[4][4], double vec[4]);
void mat_inverse(double mat[4][4], double inv[4][4]);
void legIK(double point[4], double result[3]);
void getDegreeAngles(double La[4][3], double thetas[4][3]);
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
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_Init(&hi2c1, 0x80, 50);
  PCA9685_Init(&hi2c1, 0x82, 50);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  double elapsed_time_ms = (double)HAL_GetTick();

	  double Lp[4][4];
	  positions(elapsed_time_ms, Lp);

	  double rot[3] = { 0, 0, 0 };
	  double pos[3] = { 50, 80, 0 };

	  double angles[4][3];
	  calcIK(Lp, rot, pos, angles);

	  servoRotate((double(*)[3])angles);
	  //stop pose
	  //PCA9685_SetServoAngle(&hi2c1, 0x82, 0, 102);
	  //PCA9685_SetServoAngle(&hi2c1, 0x82, 1, 126);
	  //PCA9685_SetServoAngle(&hi2c1, 0x82, 2, 90);

	  //PCA9685_SetServoAngle(&hi2c1, 0x82, 3, 83);
	  //PCA9685_SetServoAngle(&hi2c1, 0x82, 4, 90);
	  //PCA9685_SetServoAngle(&hi2c1, 0x82, 5, 90);

	  //PCA9685_SetServoAngle(&hi2c1, 0x80, 0, 102);
	  //PCA9685_SetServoAngle(&hi2c1, 0x80, 1, 126);
	  //PCA9685_SetServoAngle(&hi2c1, 0x80, 2, 87);

	  //PCA9685_SetServoAngle(&hi2c1, 0x80, 3, 78);
	  //PCA9685_SetServoAngle(&hi2c1, 0x80, 4, 59);
	  //PCA9685_SetServoAngle(&hi2c1, 0x80, 5, 87);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
