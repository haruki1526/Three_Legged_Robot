/*
  PINGでネットワーク上の対象を検索

  ターゲット:BAUD=1Mの機器
*/

#include <stdio.h>
#include "dxlib.h"
#include <ros/ros.h>
#include <robo_state/robo_command.h>


// マクロ定義
#define COMPORT   "/dev/ttyACM0"  // ポート番号
#define BAUDRATE  (500000)      // ボーレート[bps]
#define MAX_ID 9

class command_execute{
public:
	void receive(const robo_state::robo_command &commander);
	


	~command_execute();
	
	command_execute(){
		if ((dev = DX_OpenPort (COMPORT, BAUDRATE))) {
    			printf ("Open success\n");

    		int i;
    		uint32_t num=100;
    		TDxAlarmStatus stat[253];
    		if (DX_Ping2 (dev, &num, stat, &err)) {
      			for (i = 0; i < num; i++)
    		   	 	printf("Found ID=%d %02X\n", stat[i].id, stat[i].Status);
   	 	}
	
		}else{
			printf("Open error\n");
		}



	}

	


private:
	int id[MAX_ID];

	uint16_t position[MAX_ID];
	uint16_t velocity[MAX_ID]={50,50,50,50,50,50,50,50,50};


	TDeviceID  dev;
  	TErrorCode  err;

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("position_command",100, &command_execute::receive,this);
};






void command_execute::receive(const robo_state::robo_command &commander){

		for(int i=0; i<MAX_ID; i++){
			position[i]=195.5766*(M_PI+commander.theta[i])-102.4;

		}

		for(int i=0;i<MAX_ID;i++){
			//printf("id=%d pos=%d  ",(uint8_t)i+1, (uint16_t)position[i]);	
			DX_WriteWordData(dev, (uint8_t)i+1, 32, (uint16_t)velocity[i], &err);    
		
	   		DX_WriteWordData(dev, (uint8_t)i+1, 30, (uint16_t)position[i], &err);     
		}
}
	

	
command_execute::~command_execute(){
	DX_ClosePort(dev);
	printf("port close\n");
 	printf("Fin\n");
}

int main (int argc, char **argv){
	ros::init(argc, argv, "dynamixel_operating");
	command_execute execute;

	ros::spin();
}



