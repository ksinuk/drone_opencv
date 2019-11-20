#include "header_define.hpp"
#include "header1.hpp"
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>

//============== I2C ==========================
#ifdef GPIO_CONNET
#include<errno.h>
#include<wiringPi.h>
#include<wiringPiI2C.h>

#define I2C_ADDRESS 0x68 //가속도 센서 접근 주소
#define AX_ADD 0x3B //가속도 x값을 나타내는 레지스터
#define AY_ADD 0x3D //가속도 y값을 나타내는 레지스터
#define AZ_ADD 0x3F //가속도 z값을 나타내는 레지스터
#define TEMP_ADD 0x41 //센서의 온도값을 나타내는 레지스터
#define GX_ADD 0x43 //x축의 회전속도를 나타내는 레지스터
#define GY_ADD 0x45 //y축의 회전속도를 나타내는 레지스터
#define GZ_ADD 0x47 //z축의 회전속도를 나타내는 레지스터
#define RESET_MPU6050 0x6B //가속도 센서를 초기화 하는 레지스터

#define BIT_SCALE 32768.0 //16비트 레지스터의 최대값
#define GYRO_SCALE 250.0 //회전속도의 최대 측정 값은 250도/s
#define GRABITY 9.80665 //중력 가속도 값
#define ACC_SCALE 19.6133 //가속도의 최대 측정 값은 중력 가속도의 2배에 해당하는 가속도

#endif
//==================================================

LocateDrone::LocateDrone(int i) //드론 위치 및 시간 초기화
{
	trans=1;
	ax = 0.0 , ay = 0.0 , az = 0.0 , size_acc = 0.0;
	vx = 0.0 , vy = 0.0 , vz = 0.0;
	x = 0.0 , y = 0.0 , z = 0.0;
	gax = 0.0 , gay = 0.0 , gaz = 0.0;
	gx = 0.0 , gy = 0.0 , gz = 0.0;
	dax = 0.0 , day = 0.0 , daz = 0.0;
	dgax = 0.0 , dgay = 0.0 , dgaz = 0.0;

	MPU_fd = 0;
	clock_gettime(CLOCK_MONOTONIC,&before_time); //드론 작동 시작 시간 저장

}

int LocateDrone::ok(){return trans;} //가속도 센서 정상 작동 여부 반환

int LocateDrone::I2C_began() //6축가속도 센서를 i2c로 초기화
{
#ifdef GPIO_CONNET
	MPU_fd = wiringPiI2CSetup(I2C_ADDRESS); // 가속도 센서에 접근
	wiringPiI2CWriteReg8(MPU_fd , RESET_MPU6050 , 0); // 센서 초기화

	reset_error_data(); // 오차값 계산
	clock_gettime(CLOCK_MONOTONIC,&before_time); //센서 초기화 시간 저장
#endif
	return 0;
}

int LocateDrone::reset_error_data() //센서의 오차값을 알아내는 함수
{
	dax = 0.0 ; day = 0.0 ; daz = 0.0; //오차값 초기화
	dgax = 0.0; dgay = 0.0; dgaz = 0.0;

#ifdef GPIO_CONNET
	usleep(100000); //100ms 지연 
	for(int i=0;i<10;i++) //각각의 레지스터 값을 10개씩 구해서 더한다.
	{
		read_mpu6050(); //드론이 정지한 상태에서 가속도와 회전속도를 구한다.
		dgax += gax; dgay += gay; dgaz += gaz; //10개의 회전속도 샘플을 더한다.
	
		float size_acc = sqrt(ax*ax + ay*ay + az*az); //가속도의 크기를 구한다.
		dax += ax - ax*GRABITY/size_acc; //가속도 값에서 중력값을 뺀 실제 가속도를 구한다.
		day += ay - ay*GRABITY/size_acc;
		daz += az - az*GRABITY/size_acc;

		usleep(10000); //10ms 지연
	}
	dax /= 10; day /= 10; daz /= 10; //10개의 샘플 값의 평균값을 구한다.
	dgax /= 10; dgay /= 10; dgaz /= 10;

	puts("========================================"); //결과 출력
	puts("mpu6050 error sreach");
	printf("dax = %.3f , day = %.3f , daz = %.3f \n",dax,day,daz);
	printf("dgax = %.3f , dgay = %.3f , dgaz = %.3f \n",dgax,dgay,dgaz);
	puts("========================================");
#endif

	return 0;
}

void LocateDrone::whereis() //현제 드론의 위치를 구한다.
{
#ifdef GPIO_CONNET
	float time_gap=time_gap_check(); //마지막 위치 측정 이후 걸린 시간 확인

	read_mpu6050(); //가속도와 회전 속도 구하기

	ax=ax-dax; ay=ay-day; az=az-daz; //오차값 제거
	gax=gax-dgax; gay=gay-dgay; gaz=gaz-dgaz;

	gx = 0.95*(gx + gax*time_gap) + 0.05*atan2(ax,sqrt(ay*ay+az*az))*180/MY_PI; //드론의 회전값 구하기
	gy = 0.95*(gy + gay*time_gap) + 0.05*atan2(ay,sqrt(ax*ax+az*az))*180/MY_PI;
	gz = 0.95*(gz + gaz*time_gap) + 0.05*atan2(az,sqrt(ay*ay+ax*ax))*180/MY_PI;

	size_acc = sqrt(ax*ax+ay*ay+az*az); //가속도의 크기
	ax -= GRABITY*sin(gx*MY_PI/180); //중력을 뺀 실제 가속도
	ay -= GRABITY*sin(gy*MY_PI/180);
	az -= GRABITY*sin(gz*MY_PI/180);
		
	vx += ax*time_gap; //속도
	vy += ay*time_gap;
	vz += az*time_gap;

	x += vx*time_gap; //이동 거리
	y += vy*time_gap;
	z += vz*time_gap;
#endif
}

float LocateDrone::time_gap_check() // 마지막으로 측정한 시간과 현제 시간의 차이값 반환
{
	struct timespec after;
	clock_gettime(CLOCK_MONOTONIC,&after); //현재 시간 확인

	float output = after.tv_sec - before_time.tv_sec; //이전 측정 시간과 현제 시간의 차이값 계산
	output += (after.tv_nsec - before_time.tv_nsec)/1000000000.0; 

	before_time = after; //현제 시간 저장

	return output;
}

int LocateDrone::prn_locate() //드론 위치 클래스에 저장되어 있는 변수를 모니터에 출력
{
	printf("x = %.3f , y = %.3f , z = %.3f , ",x , y , z);
	printf("vx = %.3f , vy = %.3f , vz = %.3f \n",vx , vy , vz);
	printf("ax = %.3f , ay = %.3f , az = %.3f , ",ax , ay , az);
	printf("size acc = %.3f , error = %.3f \n",size_acc , (size_acc-ACC_SCALE/2)/(ACC_SCALE/2));
	printf("gx = %.3f , gy = %.3f , gz = %.3f , ",gx,gy,gz);
	printf("gax = %.3f , gay = %.3f , gaz = %.3f\n\n",gax,gay,gaz);

	return 0;
}

float LocateDrone::read_data(int addr) //가속도 센서의 16비트 레지스터 읽기
{
	short in = wiringPiI2CReadReg8(MPU_fd , addr); //wiringPiI2CReadReg8 :  8비트 레지스터를 읽어내는 삼수
	in = (in<<8) | wiringPiI2CReadReg8(MPU_fd , addr+1); //8비트 값을 2번 읽어서 16비트 정수 값 만들기

	float out = (float)in;

	return out;
}

int LocateDrone::read_mpu6050() //가속도 센서에서 가속도 값과 회전속도 값 읽기
{
	ax = read_data(AX_ADD)*(ACC_SCALE/BIT_SCALE);
	ay = read_data(AY_ADD)*(ACC_SCALE/BIT_SCALE);
	az = read_data(AZ_ADD)*(ACC_SCALE/BIT_SCALE);

	gax = read_data(GX_ADD)*(GYRO_SCALE/BIT_SCALE);
	gay = read_data(GY_ADD)*(GYRO_SCALE/BIT_SCALE);
	gaz = read_data(GZ_ADD)*(GYRO_SCALE/BIT_SCALE);

	return 0;
}

int LocateDrone::wirte_data(FILE *fd) //데이터 파일에 드론의 위치값 쓰기
{
	if(fd==NULL)return -1;

	fwrite(&x,sizeof(float),1,fd);
	fwrite(&y,sizeof(float),1,fd);
	fwrite(&z,sizeof(float),1,fd);
	fwrite(&gx,sizeof(float),1,fd);
	fwrite(&gy,sizeof(float),1,fd);
	fwrite(&gz,sizeof(float),1,fd);	

	return 0;
}

int LocateDrone::make_textfile(FILE *fb) //데이터 파일을 텍스트 파일로 변환
{
	FILE *ft;
	
	ft=fopen("drone_list.txt","a+"); //텍스트 파일 열기
	if (ft == NULL) 
	{
	     printf("don`t open text file\n");
	     return -1;
	}
	
	rewind(fb); //데이터 파일의 읽는 위치를 처음으로 돌림
	
	while(!feof(fb))
	{
		char text[128]="";
		fread(text,1,128,fb); //저장한 이미지의 제목을 읽는다.
		if(!text[0])break; //저장한 이미지가 없으면 파일을 종료한다.
		fputs(text,ft); //텍스트에 이미지 제목을 쓴다.
		
		fputs("\n",ft);
	#ifdef GPIO_CONNET
		float file_data[6];
	
		fread(file_data,sizeof(float),6,fb);// 데이터 파일에서 이미지 저장 당시의 드론의 위치를 불러 들인다.
		fprintf(ft,"x = %f , y = %f , z = %f \n",file_data[0],file_data[1],file_data[2]); //드론의 위치를 텍스트 파일에 쓴다.
		fprintf(ft,"dx = %f , dy = %f , dz = %f\n",file_data[3],file_data[4],file_data[5]);
	#endif
		fputs("===================================\n\n",ft);
	}
	
	fclose(ft); //텍스트 파일 종료
	fclose(fb); //데이터 파일 종료
	
	return 0;
}
	
