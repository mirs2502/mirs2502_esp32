#ifndef CONFIG_H_
#define CONFIG_H_

//エンコーダ用
#define PIN_ENC_A_L 4
#define PIN_ENC_B_L 5
#define PIN_ENC_A_R 13
#define PIN_ENC_B_R 14

//足回り用
#define PIN_DIR_R   25
#define PIN_PWM_R   26
#define PIN_DIR_L   32
#define PIN_PWM_L   33

//バッテリー用
#define PIN_BATT_1  27
#define PIN_BATT_2  28

//絶対に設定しろ!!!!!!!
#define ROS_DOMAIN_ID 90

#define WATCHDOG_TIMEOUT 500

//足回り速度制御用
//足回り速度制御用
double RKP = 40.0;
double RKI = 10.0;
double RKD = 10.0;
double LKP = 40.0;
double LKI = 10.0;
double LKD = 10.0;

//車体パラメータ
#define COUNTS_PER_REV    4096.0
double WHEEL_RADIUS = 0.0391;  //ホイール径 (再補正済み)
double WHEEL_BASE = 0.424;  //車輪間幅

// PWM設定
const int32_t r_Channel = 0;        // PWMチャンネル
const int32_t l_Channel = 1;
const int32_t pwmFrequency = 5000; // PWM周波数 (5kHz)
const int32_t pwmResolution = 8;   // PWM分解能 (8ビット = 0-255)

// 清掃用モーター
// 清掃用モーター
#define PIN_CLEAN_PWM_1 18
#define PIN_CLEAN_DIR_1 19

#define PIN_CLEAN_PWM_2a 23
#define PIN_CLEAN_DIR_2a 22

#define PIN_CLEAN_PWM_2b 16
#define PIN_CLEAN_DIR_2b 15

const int32_t clean_Channel_1 = 2;
const int32_t clean_Channel_2a = 3;
const int32_t clean_Channel_2b = 4;

#endif // CONFIG_H_
