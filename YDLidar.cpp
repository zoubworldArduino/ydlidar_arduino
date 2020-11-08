/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR Arduino
 *
 *  Copyright 2015 - 2018 EAI TEAM
 *  http://www.eaibot.com
 * 
 */
#include "YDLidar.h"

YDLidar::YDLidar()
    : _bined_serialdev(NULL)
{
    point.distance = 0;
    point.angle = 0;
    point.quality = 0;
}


YDLidar::~YDLidar()
{
    end();
}

// open the given serial interface and try to connect to the YDLIDAR
bool YDLidar::begin(HardwareSerial &serialobj,uint32_t baudrate)
{
    if (isOpen()) {
      end(); 
    }
    _bined_serialdev = &serialobj;
    _bined_serialdev->end();
    _bined_serialdev->begin(baudrate);
	return true;
}

// close the currently opened serial interface
void YDLidar::end(void)
{
    if (isOpen()) {
       _bined_serialdev->end();
       _bined_serialdev = NULL;
    }
}


// check whether the serial interface is opened
bool YDLidar::isOpen(void)
{
    return _bined_serialdev?true:false; 
}

// ask the YDLIDAR for its device health

result_t YDLidar::getHealth(device_health & health, uint32_t timeout) {
    result_t  ans;
	uint8_t  recvPos = 0;
    uint32_t currentTs = millis();
    uint32_t remainingtime;
    uint8_t *infobuf = (uint8_t*)&health;
	lidar_ans_header response_header;
	if (!isOpen()) {
		return RESULT_FAIL;
	}

	{

		ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH,NULL,0);
		if (ans != RESULT_OK) {
			return ans;
		}


		if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
			return ans;
		}

		if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
			return RESULT_FAIL;
		}

		if (response_header.size < sizeof(device_health)) {
			return RESULT_FAIL;
		}

		while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = _bined_serialdev->read();
            if (currentbyte<0) continue;    
            infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(device_health)) {
                return RESULT_OK;
            }
       	}
	}
	return RESULT_TIMEOUT;
}


// ask the YDLIDAR for its device info 
result_t YDLidar::getDeviceInfo(device_info & info, uint32_t timeout) {
    result_t  ans;
	uint8_t  recvPos = 0;
    uint32_t currentTs = millis();
    uint32_t remainingtime;
    uint8_t *infobuf = (uint8_t*)&info;
	lidar_ans_header response_header;
	if (!isOpen()) {
		return RESULT_FAIL;
	}

	{

		ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO,NULL,0);
		if (ans != RESULT_OK) {
			return ans;
		}


		if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
			return ans;
		}

		if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
			return RESULT_FAIL;
		}

		if (response_header.size < sizeof(lidar_ans_header)) {
			return RESULT_FAIL;
		}

		while ((remainingtime=millis() - currentTs) <= timeout) {
            int currentbyte = _bined_serialdev->read();
            if (currentbyte<0) continue;    
             infobuf[recvPos++] = currentbyte;

            if (recvPos == sizeof(device_info)) {
                return RESULT_OK;
            }
       	}
	}

	return RESULT_TIMEOUT;
}

// stop the scanPoint operation
result_t YDLidar::stop(void)
{
    if (!isOpen()) return RESULT_FAIL;
    result_t ans = sendCommand(LIDAR_CMD_FORCE_STOP,NULL,0);    
    return ans;
}

// start the scanPoint operation
result_t YDLidar::startScan(bool force, uint32_t timeout ) {
    result_t ans;

    if (!isOpen()) return RESULT_FAIL;
    
    stop(); //force the previous operation to stop

    {
       
        if ((ans = sendCommand(force?LIDAR_CMD_FORCE_SCAN:LIDAR_CMD_SCAN, NULL, 0)) != RESULT_OK) {
		return ans;
	}

	lidar_ans_header response_header;
	if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
		return ans;
	}

	if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
		return RESULT_FAIL;
	}

	if (response_header.size < sizeof(node_info)) {
		return RESULT_FAIL;
	}
    }
    return RESULT_OK;
}

/** pre computated table with excel to unload the CPU,
computation done in les than 25 instruction
the accuracy is 1 degre
*/
int32_t compute_AngCorrect(int32_t dist)
{
  if((dist<19))
  {
      if (dist<1) return 87;
      if (dist<2) return 85;
      if (dist<3) return 82;
      if (dist<4) return 79;
      if (dist<5) return 77;
      if (dist<6) return 74;
      if (dist<7) return 71;
      if (dist<8) return 69;
      if (dist<9) return 66;
      if (dist<10) return 64;
      if (dist<11) return 61;
      if (dist<12) return 59;
      if (dist<13) return 57;
      if (dist<14) return 55;
      if (dist<15) return 53;
      if (dist<16) return 51;
      if (dist<17) return 49;
      if (dist<18) return 47;
      if (dist<19) return 45;
  }
    if((dist<43))
  {
    if (dist<20) return 44;
    if (dist<21) return 42;
    if (dist<22) return 40;
    if (dist<23) return 39;
    if (dist<24) return 38;
    if (dist<25) return 36;
    if (dist<26) return 35;
    if (dist<27) return 34;
    if (dist<28) return 33;
    if (dist<29) return 31;
    if (dist<30) return 30;
    if (dist<31) return 29;
    if (dist<32) return 28;
    if (dist<33) return 27;
    if (dist<35) return 26;
    if (dist<36) return 25;
    if (dist<37) return 24;
    if (dist<38) return 23;
    if (dist<40) return 22;
    if (dist<41) return 21;
    if (dist<43) return 20;
  }
      if((dist<93))
  {
    if (dist<45) return 19;
    if (dist<46) return 18;
    if (dist<48) return 17;
    if (dist<50) return 16;
    if (dist<53) return 15;
    if (dist<55) return 14;
    if (dist<58) return 13;
    if (dist<61) return 12;
    if (dist<64) return 11;
    if (dist<67) return 10;
    if (dist<71) return 9;
    if (dist<76) return 8;
    if (dist<81) return 7;
    if (dist<86) return 6;
    if (dist<93) return 5;
      }
if (dist<100) return 4;
if (dist<109) return 3;
if (dist<119) return 2;
if (dist<131) return 1;
if (dist<147) return 0;
if (dist<166) return -1;
if (dist<191) return -2;
if (dist<226) return -3;
if (dist<276) return -4;
if (dist<354) return -5;
if (dist<495) return -6;
if (dist<825) return -7;
if (dist<2500) return -8;

}

// wait scan data
result_t YDLidar::waitScanDot( uint32_t timeout) {
	int recvPos = 0; 
	uint32_t startTs = millis();
	uint32_t waitTime;
	uint8_t nowPackageNum;
	node_info node;
	static node_package package;
	static float IntervalSampleAngle = 0;
	static float IntervalSampleAngle_LastPackage = 0;
	static uint16_t FirstSampleAngle = 0;
	static uint16_t LastSampleAngle = 0;
	static uint16_t CheckSum = 0;

	
	static uint16_t CheckSumCal = 0;
	static uint16_t SampleNumlAndCTCal = 0;
	static uint16_t LastSampleAngleCal = 0;
	static bool CheckSumResult = true;
	static uint16_t Valu8Tou16 = 0;

	uint8_t *packageBuffer = (uint8_t*)&package.package_Head;
	uint8_t  package_Sample_Num = 0;
	int32_t AngleCorrectForDistance;

	int  package_recvPos = 0;

	if(package_Sample_Index == 0) {
		recvPos = 0;
		while ((waitTime=millis() - startTs) <= timeout) {
			int currentByte = _bined_serialdev->read();
        	if (currentByte<0) continue;
			switch (recvPos) {
			case 0:
				if(currentByte!=(PH&0xFF)){
					continue;
				}
				break;
			case 1:
				CheckSumCal = PH;
				if(currentByte!=(PH>>8)){
					recvPos = 0;
					continue;
				}
				break;
			case 2:
				SampleNumlAndCTCal = currentByte;
				if ((currentByte != CT_Normal) && (currentByte != CT_RingStart)){	
					recvPos = 0;
					continue;
				}
				break;
			case 3:
				SampleNumlAndCTCal += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
				package_Sample_Num = currentByte;
				break;
			case 4:
				if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
					FirstSampleAngle = currentByte;
				} else {
					recvPos = 0;
					continue;
				}
				break;
			case 5:
				FirstSampleAngle += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
				CheckSumCal ^= FirstSampleAngle;
				FirstSampleAngle = FirstSampleAngle>>1;
				break;
			case 6:
				if (currentByte & LIDAR_RESP_MEASUREMENT_CHECKBIT) {
					LastSampleAngle = currentByte;
				} else {
					recvPos = 0;
					continue;
				}
				break;
			case 7:
				LastSampleAngle += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
				LastSampleAngleCal = LastSampleAngle;
				LastSampleAngle = LastSampleAngle>>1;
				if(package_Sample_Num == 1){
					IntervalSampleAngle = 0;
				}else{
					if(LastSampleAngle < FirstSampleAngle){
						if((FirstSampleAngle > 17280) && (LastSampleAngle < 5760)){
							IntervalSampleAngle = ((float)(23040 + LastSampleAngle - FirstSampleAngle))/(package_Sample_Num-1);
							IntervalSampleAngle_LastPackage = IntervalSampleAngle;
						} else{
							IntervalSampleAngle = IntervalSampleAngle_LastPackage;
						}
					} else{
						IntervalSampleAngle = ((float)(LastSampleAngle -FirstSampleAngle))/(package_Sample_Num-1);
						IntervalSampleAngle_LastPackage = IntervalSampleAngle;
					}
				}
				break;
			case 8:
				CheckSum = currentByte;	
				break;
			case 9:
				CheckSum += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
				break;
			}			
			packageBuffer[recvPos++] = currentByte;

			if (recvPos  == PackagePaidBytes ){
				package_recvPos = recvPos;
				break;				

			}
		}

		if(PackagePaidBytes == recvPos){
			startTs = millis();
			recvPos = 0;
			int package_sample_sum = package_Sample_Num<<1;

			while ((waitTime=millis() - startTs) <= timeout) {
				int currentByte = _bined_serialdev->read();
        		if (currentByte<0){
					continue;
				}
				if((recvPos &1) == 1){
					Valu8Tou16 += (currentByte<<LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT);
					CheckSumCal ^= Valu8Tou16;
				}else{
					Valu8Tou16 = currentByte;	
				}
										
				packageBuffer[package_recvPos+recvPos] =currentByte;					
				recvPos++;
				if(package_sample_sum == recvPos){
					package_recvPos += recvPos;
					break;
				}
			}

			if(package_sample_sum != recvPos){
				return RESULT_FAIL;
			}
		} else {
			return RESULT_FAIL;
		}
		CheckSumCal ^= SampleNumlAndCTCal;
		CheckSumCal ^= LastSampleAngleCal;

		if(CheckSumCal != CheckSum){	
			CheckSumResult = false;
		}else{
			CheckSumResult = true;
		}

	}
	uint8_t package_CT;
	package_CT = package.package_CT;    
	if(package_CT == CT_Normal){
		node.sync_quality = Node_Default_Quality + Node_NotSync;
	} else{
		node.sync_quality = Node_Default_Quality + Node_Sync;
	}

	if(CheckSumResult == true){
		node.distance_q2 = package.packageSampleDistance[package_Sample_Index];
				
        int32_t tmp=node.distance_q2;
        tmp=tmp>>2;
		if(node.distance_q2/4 != 0){
        int32_t t=node.distance_q2;
/*
        t=t>>2;// *0.25
        AngleCorrectForDistance = (int32_t)((atan(  ( (float)( ((int32_t)(21.8*155.3) - 22*t) )/(float)(155*t)))  ) *3667);
        
        
			AngleCorrectForDistance = (int32_t)((atan(((21.8*(155.3 - (node.distance_q2*0.25f)) )/155.3)/(node.distance_q2*0.25f)))*3666.93);
        */
                       AngleCorrectForDistance= compute_AngCorrect(tmp);
        
		}else{
			AngleCorrectForDistance = 0;		
		}
		float sampleAngle = IntervalSampleAngle*package_Sample_Index;
		if((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) < 0){
			node.angle_q6_checkbit = (((uint16_t)(FirstSampleAngle + sampleAngle + AngleCorrectForDistance + 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
		}else{
			if((FirstSampleAngle + sampleAngle + AngleCorrectForDistance) > 23040){
				node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance - 23040))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
			}else{
				node.angle_q6_checkbit = ((uint16_t)((FirstSampleAngle + sampleAngle + AngleCorrectForDistance))<<LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT) + LIDAR_RESP_MEASUREMENT_CHECKBIT;
			} 
		}
	}else{
		node.sync_quality = Node_Default_Quality + Node_NotSync;
		node.angle_q6_checkbit = LIDAR_RESP_MEASUREMENT_CHECKBIT;
		node.distance_q2 = 0;
		package_Sample_Index = 0;
		return RESULT_FAIL;
	}
/*
    point.distance = node.distance_q2*0.25f;
    point.angle = (node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f;
    */
    
    point.distance = node.distance_q2>>2;
    point.angle = (node.angle_q6_checkbit >> LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)>>6;// div 64
    
    point.quality = (node.sync_quality>>LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);    
    point.startBit = (node.sync_quality & LIDAR_RESP_MEASUREMENT_SYNCBIT);

	package_Sample_Index++;
	nowPackageNum = package.nowPackageNum;	
	if(package_Sample_Index >= nowPackageNum){
		package_Sample_Index = 0;	
	}

	return RESULT_OK;
}

 bool YDLidar::isScanPointReady(void)
                {
                  if (_bined_serialdev->available()>20)
                     return true;
        return package_Sample_Index != 0;
    }
//send data to serial
result_t YDLidar::sendCommand(uint8_t cmd, const void * payload, size_t payloadsize) {
	cmd_packet pkt_header;
	cmd_packet * header = &pkt_header;
	uint8_t checksum = 0;
	if (payloadsize && payload) {	
		cmd |= LIDAR_CMDFLAG_HAS_PAYLOAD;
	}

	header->syncByte = LIDAR_CMD_SYNC_BYTE;
	header->cmd_flag = cmd&0xff;

	_bined_serialdev->write((uint8_t *)header, 2) ;
	if((cmd & LIDAR_CMDFLAG_HAS_PAYLOAD)){
		checksum ^= LIDAR_CMD_SYNC_BYTE;
		checksum ^= (cmd&0xff);
		checksum ^= (payloadsize & 0xFF);
		for (size_t pos = 0; pos < payloadsize; ++pos) {
			checksum ^= ((uint8_t *)payload)[pos];		}

		uint8_t sizebyte = payloadsize;
		_bined_serialdev->write(&sizebyte, 1);
		_bined_serialdev->write((const uint8_t *)payload, sizebyte);
		_bined_serialdev->write(&checksum, 1);
	}
	return RESULT_OK;
}


// wait response header
result_t YDLidar::waitResponseHeader(lidar_ans_header * header, uint32_t timeout) {
	int  recvPos = 0;
	uint32_t startTs = millis();
	uint8_t  *headerBuffer = (uint8_t *)(header);
	uint32_t waitTime;

	while ((waitTime=millis() - startTs) <= timeout) {
		int currentbyte = _bined_serialdev->read();
       	if (currentbyte<0) continue;
        	switch (recvPos) {
        		case 0:
            		if (currentbyte != LIDAR_ANS_SYNC_BYTE1) {
                		continue;
           			}
            		break;
       			case 1:
            		if (currentbyte != LIDAR_ANS_SYNC_BYTE2) {
                		recvPos = 0;
                		continue;
            		}
            		break;
        	}  
            //    tab[recvPos]=(uint8_t )currentbyte;
        	headerBuffer[recvPos++] = (uint8_t )currentbyte;

        	if (recvPos == sizeof(lidar_ans_header)) {
            		return RESULT_OK;
        	}
	}
    return RESULT_TIMEOUT;
}

