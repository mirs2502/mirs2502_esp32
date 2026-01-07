// 他のファイルで定義されたグローバル変数を参照することを宣言
extern DRAM_ATTR volatile int32_t count_l;
extern DRAM_ATTR volatile int32_t count_r;


//ノードのタイマーコールバック関数
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //PID計算
    PID_control();
    //エンコーダーデータを格納
    enc_msg.data.data[0] = count_l;
    enc_msg.data.data[1] = count_r;
    //電圧観測
    vlt_watch(); 
    // watchdog
    if( (millis() - lastCalledAt) > WATCHDOG_TIMEOUT){
      r_vel_cmd = 0;
      l_vel_cmd = 0;
    }

    curr_vel_msg.data.data[0] = l_vel;
    curr_vel_msg.data.data[1] = r_vel;
    vlt_msg.data.data[0] = vlt_1;
    vlt_msg.data.data[1] = vlt_2;
    rcl_publish(&enc_pub, &enc_msg, NULL);
    rcl_publish(&vlt_pub, &vlt_msg, NULL);
    rcl_publish(&curr_vel_pub, &curr_vel_msg, NULL);
  }
}

// cmd_velメッセージのコールバック関数
void cmd_vel_Callback(const void * msgin) {
  const geometry_msgs__msg__Twist * vel_msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Switch back to PID mode
  control_mode = 0;

  // linear.x と angular.z のデータを取得
  linear_x = vel_msg->linear.x;
  angular_z = vel_msg->angular.z;

  //  目標速度計算
  r_vel_cmd = linear_x + WHEEL_BASE / 2 * angular_z;
  l_vel_cmd = linear_x - WHEEL_BASE / 2 * angular_z;

  // WatchDog用 最後に呼び出された時間を格納
  lastCalledAt = millis();
}

//TODO: 消してサービスに移行
//パラメーター更新のコールバック関数
void param_Callback(const void * msgin){
  const mirs_msgs__msg__BasicParam * param_msg = (const mirs_msgs__msg__BasicParam *)msgin;

  // linear.x と angular.z のデータを取得
  WHEEL_RADIUS = param_msg->wheel_radius;
  WHEEL_BASE = param_msg->wheel_base;
  RKP = param_msg->rkp;
  RKI = param_msg->rki;
  RKD = param_msg->rkd;
  LKP = param_msg->lkp;
  LKI = param_msg->lki;
  LKD = param_msg->lkd;
}

// モーター制御サービスのコールバック関数
// モーター制御サービスのコールバック関数
// モーター制御サービスのコールバック関数
void motor_ctrl_callback(const void * req, void * res){
  // control_mode は変更しない (PID制御と独立して動かすため)
  
  mirs_msgs__srv__BasicCommand_Request * req_in = (mirs_msgs__srv__BasicCommand_Request *) req;
  mirs_msgs__srv__BasicCommand_Response * res_in = (mirs_msgs__srv__BasicCommand_Response *) res;

  // param1をPWM値として使用
  // param1をMotor ID, param2をPWM値として使用
  int motor_id = (int)req_in->param1;
  int pwm_val = (int)req_in->param2;
  
  // PWM値の制限 (8bit resolution: 0-255)
  if(pwm_val > 255) pwm_val = 255;
  if(pwm_val < -255) pwm_val = -255;

  // 清掃用モーター制御
  // 安全対策: 足回り用の変数やcontrol_modeには一切触れない

  if (motor_id == 1) {
    // Motor 1 Control
    if(pwm_val >= 0){
      digitalWrite(PIN_CLEAN_DIR_1, LOW); 
      ledcWrite(clean_Channel_1, uint8_t(pwm_val));
    }else{
      digitalWrite(PIN_CLEAN_DIR_1, HIGH); 
      ledcWrite(clean_Channel_1, uint8_t(-pwm_val));
    }
  } else if (motor_id == 2) {
    // Motor 2 Control (Synchronized 2a & 2b)
    if(pwm_val >= 0){
      // Forward
      digitalWrite(PIN_CLEAN_DIR_2a, LOW);
      ledcWrite(clean_Channel_2a, uint8_t(pwm_val));
      
      digitalWrite(PIN_CLEAN_DIR_2b, LOW);
      ledcWrite(clean_Channel_2b, uint8_t(pwm_val));
    }else{
      // Reverse
      digitalWrite(PIN_CLEAN_DIR_2a, HIGH);
      ledcWrite(clean_Channel_2a, uint8_t(-pwm_val));

      digitalWrite(PIN_CLEAN_DIR_2b, HIGH);
      ledcWrite(clean_Channel_2b, uint8_t(-pwm_val));
    }
  }

  res_in->success = true;
}