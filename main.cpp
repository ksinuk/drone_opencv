#include "header1.hpp"

#define IMG_HEIGHT 300 //카메라 이미지의 세로 길이
#define IMG_WIDTH 300 // 카메라 이미지의 가로 길이

CvCapture* incap; //카메라의 정보가 담긴 구조체 변수
IplImage* img; //카메라로 부터 들어오는 원본 이미지
int img_error=0; //카메라 에러 여부를 나타내는 변수 , 카메라에 문제 있을 경우 1로 바뀜
int redboxctn=0; // 카메라 이미지에서 표지판이 연속적으로 찍힌 프레임의 갯수 , 표지판이 없어지면 0으로 리셋됨
int manual=0; //표지판 자동/수동 저장 여부 변수 , 0이면 자동 , 1이면 수동 저장
int non_save=0; //이미지 저장 여부 , 1이면 저장한다.

LocateDrone locate(0); //드론의 위치를 저장하는 클래스 변수

FILE *fd; //이미지 파일의 이름과 드론의 위치를 임시 저장하는 파일

int main()
{
	incap = cvCreateCameraCapture(0); //카메라 정보를 입력
	
	if(incap == NULL) //카메라를 사용할 수 없는 경우 프로그램 종료
	{
		perror("opencv : camera error -> incap == NULL\n");
		return -1;
	}
	
	fd=fopen("drone_list.dat","wb+"); // 저장되는 이미지의 데이터 파일
	if (fd == NULL) 
	{
	     printf("don`t open data file\n");
	     return -1;
	}

	/********** 카메라의 크기와 프레임 조절 ****************/
	cvSetCaptureProperty(incap, CV_CAP_PROP_FRAME_WIDTH, IMG_WIDTH);
	cvSetCaptureProperty(incap, CV_CAP_PROP_FRAME_HEIGHT, IMG_HEIGHT);
	cvSetCaptureProperty(incap, CV_CAP_PROP_FPS,15);
	
	cvNamedWindow("output"); //이미지 모니터에 출력 준비

	/********** 자이로스코프 + 가속도 센서 초기화 ****************/
#ifdef GPIO_CONNET
	locate.I2C_began();
#endif

	/*********************** 카메라 동작 ************************/
	while(1)
	{
		locate.whereis(); //현재 드론의 좌표 저장
		
		if(manual==0) main_camera(MY_PI/2,0); //표지판을 찾아내는 함수
		else manual_camera(); //카메라로부터 입력 받은 이미지만 출력하는 함수

		char inchar = cvWaitKey(1); //프로그램 조작키 입력

		if(inchar==27) //Esc 키는 프로그램 종료
			break;
		else if(inchar=='s') //s키는 표지판 수동 저장 
		{
			if(locate.ok()!=0)
			{
				char text[128]="manual_"; //수동 저장

				time_text(text); //이미지 제목에 촬영 당시 시간을 덧붙임
				strcat(text,".jpg"); //확장자
							
				cvSaveImage(text, img); //이미지 저장
				
				printf("save %s  :  \n",text);
			#ifdef GPIO_CONNET
				locate.prn_locate(); //위치 출력
			#endif			
				fwrite(text,1,128,fd); //이미지 이름 데이터 파일에 저장
			#ifdef GPIO_CONNET
				locate.wirte_data(fd);	//촬영 당시 드론 위치를 데이터 파일에 저장
			#endif
			}
			else puts("save error ");
		}
		else if(inchar=='t') //t키는 드론 위치 확인
		{
			if(locate.ok()!=0)
			{
				char text[128]="none_"; //이미지 저장 없음
				time_text(text); //text에 촬영 당시 시간을 덧붙임
							
				printf("save %s  :  \n",text);
			#ifdef GPIO_CONNET
				locate.prn_locate(); //위치 출력
			#endif			
				fwrite(text,1,128,fd); //촬영 당시의 시간을 데이터 파일에 저장
			#ifdef GPIO_CONNET
				locate.wirte_data(fd);	//촬영 당시 드론 위치를 데이터 파일에 저장
			#endif			
			}
			else puts("save error ");
		}

		else if(inchar=='m') //m키는 카메라로부터 입력 받은 이미지만 출력하게 한다.
		{
			manual=1;
			puts("manual mode \n");
		}
		else if(inchar=='a') //a키는 자동으로 표지판의 위치를 찾도록 하게 한다.
		{
			non_save=0;
			manual=0;
			puts("auto mode \n");
		}
		else if(inchar=='n') //n키는 이미지 자동 저장 사용 안함
		{
			non_save=1;
			puts("non save \n");
		}
		else if(inchar=='r') //r키는 사속도 센서 보정값 재측정
		{
			locate.reset_error_data();
		}
	}
	cvReleaseCapture(&incap); //카메라 해제
	cvDestroyAllWindows(); //출력 영상 종료
	cvReleaseImage(&img); //이미지 해제
	
	locate.make_textfile(fd); //데이터 파일을 텍스트 파일로 변환
	
	return 0;
}

int main_camera(double earth_rad,int road0point) //표지판을 자동으로 찾아내는 함수
{	
	img = cvQueryFrame(incap); //카메라로 부터 이미지를 받아 낸다.
	if(img==NULL) { //카메라에서 이미지를 받지 못하면 에러
		if( img_error==0)printf("img = 0\n");		
		img_error = 1;

		return -1;
	}
	img_error=0;

	/************************ 상하 반전 ******************/
	for (int y = 0; y < (img->height)/2; y++)
	{	uchar* ptr1 = (uchar*)(img->imageData + y*img->widthStep);
		uchar* ptr2 = (uchar*)(img->imageData + (img->height - 1 - y)*img->widthStep);
		uchar temp_uchar = 0;

		for (int x = 0; x < img->width; x++)
		{	for(int pn=0;pn < 3;pn++)
			{	temp_uchar = ptr1[3*x+pn];
				ptr1[3*x+pn] = ptr2[3*x+pn];
				ptr2[3*x+pn] = temp_uchar;
			} 
		}
	}

	/********** 이미지 변수 초기화 ************/
	IplImage* after_fillter = cvCreateImage(cvGetSize(img), 8, 3); 
	IplImage* red = cvCreateImage(cvGetSize(img), 8, 1);
	IplImage* black = cvCreateImage(cvGetSize(img), 8, 1);
	IplImage* output = cvCloneImage(img); 
	IplImage* temp = cvCreateImage(cvGetSize(img), 8, 1);

	/******************* 검은색 및 빨간색 영역 분리 *********************/	
	light_removal(img,after_fillter,0); //조명 제거
	cvCvtColor(after_fillter, after_fillter, CV_BGR2HSV); //필터를 거친 이미지를 HSV로 변환

	cvZero(red);cvZero(black); //이미지 변수 초기화

	for (int y = 0; y < img->height; y++)
	{	
		int angle=13; //색상의 범위
		uchar* ptr_red = (uchar*)(red->imageData + y*red->widthStep);
		uchar* ptr_black = (uchar*)(black->imageData + y*black->widthStep);

		uchar* ptr = (uchar*)(after_fillter->imageData + y*after_fillter->widthStep);

		for (int x = 0; x < img->width; x++)
		{	
			if(ptr[3*x+2]<64) ptr_black[x] = 255; //조명 값이 64보다 작으면 검은색
			else if(ptr[3*x+1]>128)
			{
				if(180-angle<ptr[3*x] || ptr[3*x]<=angle) ptr_red[x] = 255; //채도가 128 이상이고 색상값이 0 근처에 있으면 빨간색
			}
		}
	}

	cvCvtColor(after_fillter, after_fillter, CV_HSV2BGR);
	
	/**************** 입력 이미지 외각선 생성 *************/
	CvMemStorage* storge_red=cvCreateMemStorage(0);
	CvSeq* firstcontour_red = del_binerror(red,storge_red,1,100); //빨간색 외곽선 생성

	/********************** arrow process *********************/
	RedBoxList *RedBox0List = NULL;

	RedBox0List = arrow_search(firstcontour_red,red, black,0); //표지판의 위치를 저장한 리스트 생성 

	CvScalar RedBox_color = cvScalar(255,255,0); //표지판을 나타내는 색상
	CvScalar Arrow_color = cvScalar(255,0,0); //화살표를 나타내는 색상
	paint_redbox(RedBox0List, output, RedBox_color,Arrow_color, 0); //결과 이미지에 표지판의 위치를 그리기

	/******************* 표지판 자동 저장 ********************/

	if(RedBox0List !=NULL) //표지판이 있는 경우 자동 저장
	{
		RedBox0List = RedBox0List->first; //표지판 리스트의 첫 데이터로 이동

		redboxctn = (redboxctn+1)%200; //카메라 이미지에서 표지판이 연속적으로 찍힌 프레임의 갯수
		if(non_save==0 && redboxctn==4) //자동 저장 모드 + 표지판을 연속적으로 발견한 경우 일정 간격으로 저장
		{
			if(locate.ok()!=0) //드론 위치 확인
			{
				char text[128]="auto_"; // 자동 저장
				time_text(text); // 이미지 제목에 촬영 시간 적음
				strcat(text,".jpg"); // 확장자
							
				cvSaveImage(text, img); // 이미지 저장

				printf("save %s  :  \n",text);
			#ifdef GPIO_CONNET
				locate.prn_locate(); // 촬영 당시의 드론 위치 모니터에 출력
			#endif			
				fwrite(text,1,128,fd); // 저장된 이미지 제목 데이터 파일에 저장
			#ifdef GPIO_CONNET
				locate.wirte_data(fd); // 촬영 당시의 드론 위치 데이터 파일에 저장		
			#endif			
			}
			else puts("save error ");
		}
	}
	else redboxctn=0;

	/*********************** 이미지를 모니터에 출력 *******************************************/	
	cvShowImage("output",output);
	cvShowImage("after_fillter",after_fillter);
	cvShowImage("red",red);
	cvShowImage("black",black);

	/********************** Release ************************************************/
	if(after_fillter!=NULL) cvReleaseImage(&after_fillter);

	if(red!=NULL) cvReleaseImage(&red);
	if(black!=NULL) cvReleaseImage(&black);
	if(output!=NULL) cvReleaseImage(&output);

	if(temp!=NULL) cvReleaseImage(&temp);

	if(storge_red!=NULL) cvReleaseMemStorage(&storge_red);

	if(RedBox0List!=NULL) free_redbox(RedBox0List);

	return 0;
}

int manual_camera() //카메라 이미지만 출력하는 함수
{
	img = cvQueryFrame(incap);
	if(img==NULL) {
		if( img_error==0)printf("img = 0\n");		
		img_error = 1;

		return -1;
	}
	img_error=0;
	
	cvShowImage("manual",img);
	
	return 0;
}
