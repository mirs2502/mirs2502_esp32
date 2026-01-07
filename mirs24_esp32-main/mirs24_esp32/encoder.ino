// 他のファイルで定義されたグローバル変数を参照することを宣言
extern DRAM_ATTR volatile int32_t count_l;
extern DRAM_ATTR volatile int32_t count_r;

void encoder_reset() {
  count_l = 0;
  count_r = 0;
}

static void IRAM_ATTR enc_change_l() {
  int32_t a_curr = digitalRead(PIN_ENC_A_L);
  int32_t b_curr = digitalRead(PIN_ENC_B_L);

  if(a_curr == b_curr){
    count_l++;
  }else{
    count_l--;
  }
}

static void IRAM_ATTR enc_change_r() {
  int32_t a_curr = digitalRead(PIN_ENC_A_R);
  int32_t b_curr = digitalRead(PIN_ENC_B_R);

  if(a_curr == b_curr){
    count_r--;
  }else{
    count_r++;
  }
}

void encoder_open() {
  pinMode(PIN_ENC_A_L, INPUT_PULLUP);
  pinMode(PIN_ENC_B_L, INPUT_PULLUP);
  pinMode(PIN_ENC_A_R, INPUT_PULLUP);
  pinMode(PIN_ENC_B_R, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_L), enc_change_l, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_R), enc_change_r, CHANGE);

  enc_msg.data.size = 2; // メッセージ配列のサイズを2に設定
  enc_msg.data.data = (int32_t *)malloc(enc_msg.data.size * sizeof(int32_t)); // 配列のメモリを確保
  enc_msg.data.data[0] = 0;
  enc_msg.data.data[1] = 0;
}