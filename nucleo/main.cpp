// main.cpp
//{{{  includes
#include <algorithm>
#include <string>
#include <ctype.h>

#include "cmsis_os.h"

#include "../common/stm32l4xx_nucleo_144.h"
#include "../common/heap.h"
#include "../common/cRtc.h"
#include "cTouch.h"

#include "cLcd.h"

using namespace std;
//}}}
const string kHello = "smallLcd " + string(__TIME__) + " " + string(__DATE__);

//{{{  vars
cLcd* gLcd = nullptr;
cRtc* gRtc = nullptr;
cTouch* gTouch = nullptr;

float mRadius = 160.f;
cPointF mCentre = cPointF (160.f, 240.f);

enum eMove { eNotPressed, ePressed, eMoveCentre, eMoveRadius};
eMove mMove = eNotPressed;
//}}}

static I2C_HandleTypeDef I2cHandle;
extern "C" { void I2C4_EV_IRQHandler() { HAL_I2C_EV_IRQHandler (&I2cHandle); } }
extern "C" { void I2C4_ER_IRQHandler() { HAL_I2C_ER_IRQHandler (&I2cHandle); } }

#define WHO_AM_I_MPU9250  0x75  // Should return 0x71
#define MPU9250_ADDRESS   0x68<<1  // Device address when ADO = 0
//{{{
void mpu2950init() {

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_I2C4_CLK_ENABLE();

  // gpio init
  GPIO_InitTypeDef  GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C4;
  HAL_GPIO_Init (GPIOF, &GPIO_InitStruct);

  // i2c init
  I2cHandle.Instance = I2C4;
  I2cHandle.Init.Timing = 0x20601138;
  I2cHandle.Init.OwnAddress1 = 0;
  I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.OwnAddress2 = 0;
  I2cHandle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init (&I2cHandle) != HAL_OK)
    printf ("HAL_I2C_Init error\n");

  HAL_NVIC_SetPriority (I2C4_ER_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ (I2C4_ER_IRQn);
  HAL_NVIC_SetPriority (I2C4_EV_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ (I2C4_EV_IRQn);

  HAL_I2CEx_ConfigAnalogFilter (&I2cHandle, I2C_ANALOGFILTER_ENABLE);

  uint8_t value = WHO_AM_I_MPU9250;
  if (HAL_I2C_Master_Transmit (&I2cHandle, MPU9250_ADDRESS, &value, 1, 10000) != HAL_OK)
    printf ("m2950init id tx error\n");
  if (HAL_I2C_Master_Receive (&I2cHandle, MPU9250_ADDRESS, &value, 1, 10000) != HAL_OK)
    printf ("m2950init id rx error\n");
  else
    printf ("whoami la %x\n", value);
  }
//}}}
#include "MPU9250.h"

//{{{
void clockConfig120Mhz() {
// System Clock source = PLL (MSI)
// SYSCLK(Hz)          = 120000000
// HCLK(Hz)            = 120000000
// AHB Prescaler       = 1
// APB1 Prescaler      = 1
// APB2 Prescaler      = 1
// MSI Frequency(Hz)   = 4000000
// PLL_M               = 1
// PLL_N               = 60
// PLL_Q               = 2
// PLL_R               = 2
// PLL_P               = 7
// Flash Latency(WS)   = 5

  // Enable voltage range 1 boost mode for frequency above 80 Mhz
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling (PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  __HAL_RCC_PWR_CLK_DISABLE();

  // Enable MSI Oscillator and activate PLL with MSI as source
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  if (HAL_RCC_OscConfig (&RCC_OscInitStruct) != HAL_OK)
    while (1);

  // avoid undershoot due to maximum frequency, select PLL system clock AHB prescaler divider 2 as first step
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                 RCC_CLOCKTYPE_PCLK1  | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    while (1);

  // AHB prescaler divider at 1 as second step
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  if (HAL_RCC_ClockConfig (&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    while (1);
  }
//}}}
//{{{
void uiThread (void* arg) {

  gLcd->tftInit();
  gLcd->display (100);

  while (true) {
    if (gLcd->isChanged() || (gLcd->getPresentTime() >= 1000)) {
      gLcd->start();
      gLcd->clear (kBlack);

      gLcd->setShowInfo (BSP_PB_GetState (BUTTON_KEY) == 0);
      gLcd->drawInfo();

      //float vRef = (3.f * vRefIntValueCalibrated) / vRefIntValue;
      //float v5v = v5vValue * ((vRef * (39.f + 27.f) / 39.f) / 4096.f);
      //float vBat = vBatValue * ((vRef * 3.f) / 4096.f);
      //gLcd->text (kWhite, 20, "vBat  " + dec (vBatValue) + " " +
      //                                  dec (int (vBat),1,' ') + "." +
      //                                  dec (int (vBat * 100) % 100, 2,'0'), cRect (0, 40, 320, 60));
      //gLcd->text (kWhite, 20, "vRef " + dec (vRefIntValue) + " " +
      //                                 dec (int (vRef),1,' ') + "." +
      //                                 dec (int (vRef * 100) % 100, 2,'0') + " " +
      //                                 dec (v5vValue) + " " +
      //                                 dec (int (v5v),1,' ') + "." +
      //                                 dec (int (v5v * 100) % 100, 2,'0'), cRect (0, 20, 320, 40));

      if (gTouch->getPressed()) {
        //{{{  touch radius and string
        gLcd->aEllipse (gTouch->getPos(), cPointF (16.f,16.f), 32);
        gLcd->aRender (kYellow, false);

        gLcd->text (kWhite, 22,
                    dec (gTouch->getValueX(),4,' ') + "," + dec (gTouch->getValueY(), 4, ' ') + " " +
                    dec (int(gTouch->getPos().x)) + "." + dec (int(gTouch->getPos().x * 10) % 10, 1,'0') + "," +
                    dec (int(gTouch->getPos().y)) + "." + dec (int(gTouch->getPos().y * 10) % 10, 1,'0'),
                    cRect (0, 20, 320, 42));
        }
        //}}}

      //{{{  clock
      float hourA;
      float minuteA;
      float secondA;
      float subSecondA;
      gRtc->getClockAngles (hourA, minuteA, secondA, subSecondA);

      auto r = mRadius;
      auto c = mCentre;
      gLcd->aEllipse (c, cPointF(r-4.f, r), 64);
      gLcd->aRender (mMove == eMoveCentre ? kGrey : sRgba (128,128,128, 192), false);

      gLcd->aEllipseOutline (c, cPointF(r, r), 4.f, gTouch->getPressed() ? 16 : 64);
      gLcd->aRender (mMove == eMoveRadius ? kWhite : sRgba (180,180,0, 255), false);

      float handWidth = r > 60.f ? r / 20.f : 3.f;
      float hourR = r * 0.75f;
      gLcd->aPointedLine (c, c + cPointF (hourR * sin (hourA), hourR * cos (hourA)), handWidth);
      float minuteR = r * 0.9f;
      gLcd->aPointedLine (c, c + cPointF (minuteR * sin (minuteA), minuteR * cos (minuteA)), handWidth);
      gLcd->aRender (kWhite);

      float secondR = r * 0.95f;
      gLcd->aPointedLine (c, c + cPointF (secondR * sin (secondA), secondR * cos (secondA)), handWidth);
      gLcd->aRender (sRgba (255,0,0, 180));

      //float subSecondR = r * 0.95f;
      //gLcd->aPointedLine (c, c + cPointF (minuteR * sin (subSecondA), minuteR * cos (subSecondA)), 3.f);
      //gLcd->aRender (sRgba (255,255,0, 128));
      //}}}
      gLcd->text (kWhite, 30, gRtc->getClockTimeDateString(), cRect (0, 426, 320, 480));

      gLcd->present();
      }

    vTaskDelay (10);
    }

  }
//}}}
//{{{
void appThread (void* arg) {

  MPU9250 mpu9250;

  mpu2950init();
  mpu9250.resetMPU9250(); // Reset registers to default in preparation for device calibration
  mpu9250.calibrateMPU9250 (gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
  printf ("x gyro bias = %f\n", gyroBias[0]);
  printf ("y gyro bias = %f\n", gyroBias[1]);
  printf  ("z gyro bias = %f\n", gyroBias[2]);
  printf ("x accel bias = %f\n", accelBias[0]);
  printf ("y accel bias = %f\n", accelBias[1]);
  printf ("z accel bias = %f\n", accelBias[2]);
  vTaskDelay (2000);

  mpu9250.initMPU9250();
  printf("MPU9250 initialized for active data mode....\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

  mpu9250.initAK8963 (magCalibration);
  printf ("AK8963 initialized for active data mode....\n"); // Initialize device for active mode read of magnetometer
  printf ("Accelerometer full-scale range = %f  g\n", 2.0f*(float)(1<<Ascale));
  printf ("Gyroscope full-scale range = %f  deg/s\n", 250.0f*(float)(1<<Gscale));
  if(Mscale == 0)
    printf ("Magnetometer resolution = 14  bits\n");
  if(Mscale == 1)
    printf ("Magnetometer resolution = 16  bits\n");
  if(Mmode == 2)
    printf ("Magnetometer ODR = 8 Hz\n");
  if(Mmode == 6)
    printf ("Magnetometer ODR = 100 Hz\n");
  vTaskDelay (2000);


  mpu9250.getAres(); // Get accelerometer sensitivity
  mpu9250.getGres(); // Get gyro sensitivity
  mpu9250.getMres(); // Get magnetometer sensitivity
  printf ("Accelerometer sensitivity is %f LSB/g \n", 1.0f/aRes);
  printf ("Gyroscope sensitivity is %f LSB/deg/s \n", 1.0f/gRes);
  printf ("Magnetometer sensitivity is %f LSB/G \n", 1.0f/mRes);
  magbias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
  magbias[1] = +120.;  // User environmental x-axis correction in milliGauss
  magbias[2] = +125.;  // User environmental x-axis correction in milliGauss

  while (true) {
    vTaskDelay (100);
    // If intPin goes high, all data registers have new data
    //if(mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {  // On interrupt, check if data ready interrupt

    mpu9250.readAccelData(accelCount);  // Read the x/y/z adc values
    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - accelBias[1];
    az = (float)accelCount[2]*aRes - accelBias[2];

    mpu9250.readGyroData(gyroCount);  // Read the x/y/z adc values
    // Calculate the gyro value into actual degrees per second
    gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
    gy = (float)gyroCount[1]*gRes - gyroBias[1];
    gz = (float)gyroCount[2]*gRes - gyroBias[2];

    mpu9250.readMagData(magCount);  // Read the x/y/z adc values
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental corrections
    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];
    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];
    //}

    // Pass gyro rate as rad/s
    mpu9250.MadgwickQuaternionUpdate (ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
    // mpu9250.MahonyQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, my, mx, mz);

    // Serial print and/or display at 0.5 s rate independent of data rates
    //delt_t = t.read_ms() - count;
    //if (delt_t > 500) { // update LCD once per half-second independent of read rate

    printf ("ax = %f ay = %f az = %f mg\n", 1000*ax, 1000*ay, 1000*az);
    printf ("gx = %f gy = %f gz = %f deg/s\n", gx, gy, gz);
    printf ("mx = %f my = %f mz = %f mG\n", mx, my, mz);

    tempCount = mpu9250.readTempData();  // Read the adc values
    temperature = ((float)tempCount) / 333.87f + 21.0f; // Temperature in degrees Centigrade
    printf (" temperature = %f  C\n", temperature);

    printf ("q0 = %f q1 = %f q2 = %f q3 = %f\n", q[0],q[1],q[2],q[3]);

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / PI;

    yaw   *= 180.0f / PI;
    yaw   -= 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    roll  *= 180.0f / PI;
    printf("Yaw, Pitch, Roll: %f %f %f\n", yaw, pitch, roll);
    }

  cPointF offset;
  int brightness = 100;

  while (true) {
    gTouch->start();
    gTouch->wait();

    if (gTouch->getState() == cTouch::ePress) {
      if (gTouch->getPressed()) {
        if (mMove == eNotPressed) {
          brightness = 100;
          gLcd->display (brightness);
          if ((mCentre - gTouch->getPos()).magnitude() < mRadius-4.f) {
            mMove = eMoveCentre;
            offset = gTouch->getPos() - mCentre;
            }
          else if ((mCentre - gTouch->getPos()).magnitude() < mRadius+24.f)
            mMove = eMoveRadius;
          else
            mMove = ePressed;
          }

        if (mMove == eMoveCentre)
          mCentre = gTouch->getPos() - offset;
        else if (mMove == eMoveRadius)
          mRadius = (mCentre - gTouch->getPos()).magnitude();

        gLcd->change();
        vTaskDelay (1);
        }
      else {
        mMove = eNotPressed;
        if (brightness > 0)
          gLcd->display (--brightness);
        vTaskDelay (50);
        }
      }
    }
  }
//}}}

int main() {
  HAL_Init();
  clockConfig120Mhz();
  printf ("%s\n", kHello.c_str());

  BSP_LED_Init (LED_RED);
  BSP_PB_Init (BUTTON_KEY, BUTTON_MODE_GPIO);

  gRtc = new cRtc();
  gRtc->init();

  gLcd = new cLcd();
  gLcd->init (kHello);

  gTouch = new cTouch (gLcd->getSize());
  gTouch->init();

  TaskHandle_t uiHandle;
  xTaskCreate ((TaskFunction_t)uiThread, "ui", 4096, 0, 4, &uiHandle);
  TaskHandle_t appHandle;
  xTaskCreate ((TaskFunction_t)appThread, "app", 1024, 0, 4, &appHandle);
  vTaskStartScheduler();

  return 0;
  }
