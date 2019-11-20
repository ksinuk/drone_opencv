#include "header1.hpp" 


int light_removal(IplImage* img,IplImage* out,int debugging) //입력받은 이미지에서 조명을 제거하는 함수
{
	if(img==NULL || out==NULL)return -1; //입출력 할 수 있는 이미지가 없으면 에러
	//cvSmooth( img,img,CV_BLUR,5,5);

	cvCvtColor(img,out,CV_BGR2HSV); //RGB이미지에서 HSV이미지로 변환해서 조명값을 추출한다.

	int dmul[256]; //제곱값을 미리 계산하여 저장하는 배열
	for(int i=0;i<256;i++) dmul[i]=i*i/255;

	int histogram[256]; //0으초 초기화 된 히스토그램 배열
	for(int i=0;i<256;i++) histogram[i]=0; 

	float Nt = out->height*out->width; //이미지에 있는 픽셀의 양

	for(int y = 0; y<out->height; y++) //히스토그램 계산
	{	uchar* ptr = (uchar*)(out->imageData + y*out->widthStep);
		for(int x = 0; x<out->width; x++)
		{	histogram[ptr[3*x+2]]++;
		}
	}

	/*************************** Histogram Equalization ****************************/
	for(int i=1;i<256;i++) histogram[i]+=histogram[i-1]; //히스토그램 누적합 계산
	
	for(int i=1;i<256;i++) // 누적 히스토그램의 최대값을 255로 조정
	{   float temp_h =((float)histogram[i])*255.0/Nt;
		histogram[i] = (temp_h > 255.0) ? 255 : temp_h;
	}
			
	for(int y = 0; y<out->height; y++) //누적 히스토그램을 이용해서 이미지의 명암을 고르게 한다.
	{   uchar* ptr = (uchar*)(out->imageData + y*out->widthStep);
		for(int x = 0; x<out->width; x++)
		{
			ptr[3*x+2] = histogram[ptr[3*x+2]];
		}
	}

	/**************************** histogram strach **********************************/
	for(int i=0;i<256;i++) histogram[i]=0; //사용한 히스토그램 초기화

	for(int y = 0; y<out->height; y++) //Histogram Equalization한 영상에서 히스토그램 계산
	{	uchar* ptr = (uchar*)(out->imageData + y*out->widthStep);
		for(int x = 0; x<out->width; x++)
		{	histogram[ptr[3*x+2]]++;
		}
	}

	int low_cut = Nt*0.2 , high_cut = Nt*0.2, sum=0; //밝기를 0으로 만들 픽셀의 갯수 , 밝기를 255로 만들 픽셀의 갯수 , 계산 중인 픽셀의 총 합
	int low_i=0 , high_i=0; //픽셀의 밝기가 low_i 보다 작으면 0, high_i보다 크면 255가 된다.

	for(int i=0;i<256;i++) //low_i 계산
	{	sum+=histogram[i];
		if(sum>=low_cut){low_i=i;break;}
	}

	sum=0;
	for(int i=255;i>=0;i--) //high_i 계산
	{	sum+=histogram[i];
		if(sum>=high_cut){high_i=i;break;}
	}

	for(int i=0;i<low_i;i++) histogram[i] = 0; //픽셀의 밝기가 low_i 보다 작으면 0
	for(int i=255;i>high_i;i--) histogram[i] = 255; //픽셀의 밝기가  high_i보다 크면 255

	float scale = 255.0/(float)(high_i - low_i); //픽셀의 밝기가 low_i와 high_i의 중간이면 다음과 같이 계산한다.
	for(int i=low_i;i<=high_i;i++)
	{
		histogram[i] = (i-low_i)*scale;
		histogram[i] = (histogram[i] > 255) ? 255 : histogram[i];
	}

	for(int y = 0; y<out->height; y++) //픽셀의 밝기 분포를 퍼트려서 명암의 구분을 선명하게 한다.
	{	uchar* ptr = (uchar*)(out->imageData + y*out->widthStep);
		for(int x = 0; x<out->width; x++)
		{	ptr[3*x+2] = dmul[histogram[ptr[3*x+2]]];
		}
	}

	cvCvtColor(out,out,CV_HSV2BGR); //HSV에서 RGB로 복구한다.

	return 0;
}

CvSeq* del_binerror(IplImage* img,CvMemStorage* storge,int opt,double area) //이진 이미지에서 외곽선을 추출한다.
{
	if(img==NULL)return NULL; //사용할 이진이미지가 없으면 에러

	if(opt) //침식과 팽창연산을 통해서 노이즈를 제거한다.
	{
		cvErode(img,img,NULL,1);
		cvDilate(img,img,NULL,2);
		cvErode(img,img,NULL,1);
	}
	CvSeq* firstcontour_img=NULL; //외곽선을 저장하는 변수
	cvFindContours(img,storge,&firstcontour_img,sizeof(CvContour),CV_RETR_CCOMP); //외곽선 추출 함수
	cvSetZero(img);
	if(opt) firstcontour_img = del_small_contour(firstcontour_img,area,0);//불필요한 작은 외곽선 제거
	cvDrawContours(img,firstcontour_img,255,128,10,-1);	//저장된 외곽선만 가지는 픽셀만 그려진 이미지 출력

	return firstcontour_img; //외곽선이 저장된 장소를 가리키는 포인터 반환
}


