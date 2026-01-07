#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <mirs_msgs/srv/parameter_update.h>
#include <mirs_msgs/srv/simple_command.h>
#include <mirs_msgs/msg/basic_param.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <mirs_msgs/action/trigger.h>
#include <mirs_msgs/srv/basic_command.h> // Added for motor control
#include <pthread.h>
#include "config.h"

//... (既存の #include ... はそのまま) ...
#include "config.h"

// --- ここから追加 ---
// 別ファイルで定義されている関数のプロトタイプ宣言
void ros_setup();
void encoder_open();
void encoder_reset();
void vel_ctrl_set();
void vlt_setup();
void vlt_watch();
void PID_control();
void calculate_vel();
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void cmd_vel_Callback(const void * msgin);
void param_Callback(const void * msgin);
void update_service_callback(const void * req, void * res);
void reset_service_callback(const void * req, void * res);
void motor_ctrl_callback(const void * req, void * res); // Added callback
static void IRAM_ATTR enc_change_l();
static void IRAM_ATTR enc_change_r();
// --- ここまで追加 ---


//topic通信で使用するメッセージ宣言
std_msgs__msg__Int32MultiArray enc_msg;         //エンコーダー情報
//... (以下、変更なし) ...

//topic通信で使用するメッセージ宣言

std_msgs__msg__Float64MultiArray vlt_msg;       //電圧情報
std_msgs__msg__Float64MultiArray curr_vel_msg;  //速度情報
geometry_msgs__msg__Twist cmd_vel_msg;          //速度指令値
mirs_msgs__msg__BasicParam param_msg;           //パラメーターメッセージ

//service通信で使用するメッセージ宣言
mirs_msgs__srv__ParameterUpdate_Response update_res;
mirs_msgs__srv__ParameterUpdate_Request update_req;
mirs_msgs__srv__SimpleCommand_Response reset_res;
mirs_msgs__srv__SimpleCommand_Request reset_req;
mirs_msgs__srv__BasicCommand_Response motor_ctrl_res; // Added response
mirs_msgs__srv__BasicCommand_Request motor_ctrl_req;   // Added request

//publisher,subscriber,serviceの宣言
rcl_publisher_t enc_pub;
rcl_publisher_t vlt_pub;
rcl_publisher_t curr_vel_pub;
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t param_sub;
rcl_service_t update_srv;
rcl_service_t reset_srv;
rcl_service_t motor_ctrl_srv; // Added service

//ノードに関わる宣言
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

/* 処理で使用するグローバル変数 */

//エンコーダーカウント
//int32_t count_l = 0;
//int32_t count_r = 0;

//int32_t prev_count_l = 0;
//int32_t prev_count_r = 0;

//エンコーダーカウント (ISRで変更されるため volatile が必須)
// ISR (IRAM) からアクセスされるため volatile と DRAM_ATTR が必須
DRAM_ATTR volatile int32_t count_l = 0;
DRAM_ATTR volatile int32_t count_r = 0;

DRAM_ATTR volatile int32_t prev_count_l = 0;
DRAM_ATTR volatile int32_t prev_count_r = 0;

//速度制御用の変数
double r_vel_cmd;
double l_vel_cmd;
double r_vel;
double l_vel;

float linear_x;   //  直進速度
float angular_z;  //  回転速度

float r_err_sum = 0;
float l_err_sum = 0;

float prev_r_err = 0;
float prev_l_err = 0;

//電圧
double vlt_1 = 0;
double vlt_2 = 0;

//WatchDog用
uint32_t lastCalledAt;

// Motor Control Pin
// #define MOTOR_PIN 25 // Example pin, adjust as needed
// #define MOTOR_PWM_CHANNEL 4 // Use a different channel than motors

// Control Mode (0: PID, 1: PWM)
int control_mode = 0;

void setup() {
  vel_ctrl_set();
  ros_setup();

  encoder_open();
  vlt_setup();
  
  delay(500);
}

void loop() {
  delay(10);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}

