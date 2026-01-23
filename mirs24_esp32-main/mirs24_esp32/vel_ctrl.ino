// 他のファイルで定義されたグローバル変数を参照することを宣言
extern DRAM_ATTR volatile int32_t count_l;
extern DRAM_ATTR volatile int32_t count_r;
extern DRAM_ATTR volatile int32_t prev_count_l;
extern DRAM_ATTR volatile int32_t prev_count_r;

extern int control_mode;

unsigned long prev_calc_time = 0;

void calculate_vel(){
  unsigned long current_time = millis();
  
  if (prev_calc_time == 0) {
    prev_calc_time = current_time;
    prev_count_l = count_l;
    prev_count_r = count_r;
    return;
  }

  double dt = (double)(current_time - prev_calc_time) / 1000.0;
  prev_calc_time = current_time;

  if (dt <= 0.0) return;

  //エンコーダーの変化量を計算
  int32_t delta_left = count_l - prev_count_l;
  int32_t delta_right = count_r - prev_count_r;

  prev_count_l = count_l;
  prev_count_r = count_r;

  // 車輪の回転角度（ラジアン）を計算
  double delta_left_rad = (delta_left / COUNTS_PER_REV) * 2.0 * PI;
  double delta_right_rad = (delta_right / COUNTS_PER_REV) * 2.0 * PI;

  // それぞれの車輪の移動距離を計算
  double left_distance = delta_left_rad * WHEEL_RADIUS;
  double right_distance = delta_right_rad * WHEEL_RADIUS;

  // 現在の速度を計算 (m/s)
  l_vel = left_distance / dt;
  r_vel = right_distance / dt;
}

void PID_control(){
  calculate_vel();

  // If in PWM mode, skip PID control to avoid overwriting PWM values
  if (control_mode == 1) {
    return;
  }

  // Reset PID terms if command is 0 (Stop)
  if (r_vel_cmd == 0) {
    r_err_sum = 0;
    prev_r_err = 0;
    r_vel = 0; // Optional: Force zero velocity if stopped
  }
  if (l_vel_cmd == 0) {
    l_err_sum = 0;
    prev_l_err = 0;
    l_vel = 0;
  }

  float r_err = r_vel_cmd - r_vel;
  float l_err = l_vel_cmd - l_vel;

  r_err_sum += r_err;
  l_err_sum += l_err;

  // Anti-Windup (Limit Integral Term)
  // Limit sum to contribute at most ~50% of max PWM (e.g., 128)
  // 128 / RKI (30) ~= 4.0
  if (r_err_sum > 4.0) r_err_sum = 4.0;
  if (r_err_sum < -4.0) r_err_sum = -4.0;
  if (l_err_sum > 4.0) l_err_sum = 4.0;
  if (l_err_sum < -4.0) l_err_sum = -4.0;

  // PID計算を実行
  double r_pwm = RKP * r_err + RKI * r_err_sum + RKD *  (r_err - prev_r_err);
  double l_pwm = LKP * l_err + LKI * l_err_sum + LKD *  (l_err - prev_l_err);

  prev_r_err = r_err;
  prev_l_err = l_err;

  // Minimum PWM (Deadband compensation)
  // 60 is a provisional value. Tune this if the robot doesn't move at low speeds.
  double min_pwm = 30.0;
  if (r_pwm > 0 && r_pwm < min_pwm) r_pwm = min_pwm;
  if (r_pwm < 0 && r_pwm > -min_pwm) r_pwm = -min_pwm;
  if (l_pwm > 0 && l_pwm < min_pwm) l_pwm = min_pwm;
  if (l_pwm < 0 && l_pwm > -min_pwm) l_pwm = -min_pwm;

  if(r_pwm > 255){
    r_pwm = 255;
  }else if(r_pwm < -255){
    r_pwm = -255;
  }
  if(l_pwm > 255){
    l_pwm = 255;
  }else if(l_pwm < -255){
    l_pwm = -255;
  }

  //  出力値の前処理
  if(r_pwm >= 0){
    digitalWrite(PIN_DIR_R, LOW);
  }else{
    r_pwm *= -1;
    digitalWrite(PIN_DIR_R,HIGH);
  }
  if(l_pwm >= 0){
    digitalWrite(PIN_DIR_L, HIGH);
  }else{
    l_pwm *= -1;
    digitalWrite(PIN_DIR_L,LOW);
  }

  // --- Stall Detection (Failsafe) ---
  static unsigned long stall_start_time_r = 0;
  static unsigned long stall_start_time_l = 0;
  const int STALL_PWM_THRESHOLD = 50;
  const unsigned long STALL_TIME_THRESHOLD = 1000; // 1 second

  // Right Motor Stall Check
  if (r_pwm > STALL_PWM_THRESHOLD && abs(r_vel) < 0.001) {
    if (stall_start_time_r == 0) {
      stall_start_time_r = millis();
    } else if (millis() - stall_start_time_r > STALL_TIME_THRESHOLD) {
      r_pwm = 0; // Force stop
    }
  } else {
    stall_start_time_r = 0;
  }

  // Left Motor Stall Check
  if (l_pwm > STALL_PWM_THRESHOLD && abs(l_vel) < 0.001) {
    if (stall_start_time_l == 0) {
      stall_start_time_l = millis();
    } else if (millis() - stall_start_time_l > STALL_TIME_THRESHOLD) {
      l_pwm = 0; // Force stop
    }
  } else {
    stall_start_time_l = 0;
  }
  // ----------------------------------

  //  確実な停止 (PWM 0出力)
  if(r_vel_cmd == 0){
    r_pwm = 0;
  }
  if(l_vel_cmd == 0){
    l_pwm = 0; 
  }

  //  pwm出力
  ledcWrite(r_Channel, uint8_t(r_pwm));
  ledcWrite(l_Channel, uint8_t(l_pwm));
}

void vel_ctrl_set() {
  // ledcのPWM設定
  pinMode(PIN_DIR_R, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);
  ledcSetup(r_Channel, pwmFrequency, pwmResolution);
  ledcSetup(l_Channel, pwmFrequency, pwmResolution);
  ledcAttachPin(PIN_PWM_R, r_Channel);
  ledcAttachPin(PIN_PWM_L, l_Channel);

  // 清掃用モーター設定
  // 清掃用モーター設定
  // Motor 1
  pinMode(PIN_CLEAN_DIR_1, OUTPUT);
  ledcSetup(clean_Channel_1, pwmFrequency, pwmResolution);
  ledcAttachPin(PIN_CLEAN_PWM_1, clean_Channel_1);

  // Motor 2a
  pinMode(PIN_CLEAN_DIR_2a, OUTPUT);
  ledcSetup(clean_Channel_2a, pwmFrequency, pwmResolution);
  ledcAttachPin(PIN_CLEAN_PWM_2a, clean_Channel_2a);

  // Motor 2b
  pinMode(PIN_CLEAN_DIR_2b, OUTPUT);
  ledcSetup(clean_Channel_2b, pwmFrequency, pwmResolution);
  ledcAttachPin(PIN_CLEAN_PWM_2b, clean_Channel_2b);

  curr_vel_msg.data.size = 2; // メッセージ配列のサイズを2に設定
  curr_vel_msg.data.data = (double *)malloc(curr_vel_msg.data.size * sizeof(double)); // 配列のメモリを確保
  curr_vel_msg.data.data[0] = 0;
  curr_vel_msg.data.data[1] = 0;

  // 初期化時に確実に停止させる
  ledcWrite(r_Channel, 0);
  ledcWrite(l_Channel, 0);
}
