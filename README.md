# SmartCarControl execution process
1. 执行Raspberry2Arduino_send中发送脚本 x2.sh：
   tail -f input.txt | ./gettingstarted
   该脚本不断轮询input.txt中的内容，一旦检测到即通过调用Raspberry2Arduino_send中的gettingstarted功能将数据通过2.4GHz模块发送给百米外的Arduino
2. 智能车主控程序videoRobot.cpp中，车辆停止后，程序检测当前安全状态，将状态信息逐行写入到input.txt文件中。
3. 1中x2.h脚本将数据逐行发送出去，发送后将txt文件中数据清空，用于下次发送
4. 开机自启动以上两个进程。
