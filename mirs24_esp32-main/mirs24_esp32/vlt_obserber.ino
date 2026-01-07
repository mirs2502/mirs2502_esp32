void vlt_watch(){
  vlt_1 = analogRead(PIN_BATT_1);
  vlt_2 = analogRead(PIN_BATT_2);
}

void vlt_setup(){
  pinMode(PIN_BATT_1, ANALOG);
  pinMode(PIN_BATT_2, ANALOG);
  
  vlt_msg.data.size = 2; // メッセージ配列のサイズを2に設定
  vlt_msg.data.data = (double *)malloc(vlt_msg.data.size * sizeof(double)); // 配列のメモリを確保
  vlt_msg.data.data[0] = 0;
  vlt_msg.data.data[1] = 0;
}