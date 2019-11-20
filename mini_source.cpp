#include "header1.hpp"
#include <time.h>

#define SWAP(a,b) { int t;t=a;a=b;b=t; }

void BubbleSort(int *ar, int num ,int dir) //버블 정렬 함수
{
	for (int i=0;i<num-1;i++) 
	{	for(int j = 1; j<num-i; j++)
		{	if((ar[j-1] > ar[j]) && (dir == 0)) SWAP(ar[j-1],ar[j]); 
			if((ar[j-1] < ar[j]) && (dir != 0))SWAP(ar[j-1],ar[j]);
		}
	}
}

double calrad(CvPoint corner[]) //삼각형이 기울어진 정도를 게산하는 함수 
{
	if(corner == NULL) return 1000;

	CvPoint mm; //3점의 무게중심 좌표
	mm.x = (corner[0].x+corner[1].x+corner[2].x)/3;
	mm.y = (corner[0].y+corner[1].y+corner[2].y)/3;
	double output;

	int lentri[3]; //무게중심에서 3점 사이의 거리의 제곱 값
	for(int i=0;i<3;i++) lentri[i] = (corner[i].x - mm.x)*(corner[i].x - mm.x) + (corner[i].y - mm.y)*(corner[i].y - mm.y);

	//무게중심에서 가장 멀리 떨어진 점과 무게중심을 연결하는 직선의 각도를 계산한다.
	if(lentri[0] >= lentri[1] && lentri[0] >= lentri[2]) output = atan2(corner[0].y - mm.y , corner[0].x - mm.x);
	else if(lentri[1] >= lentri[2]) output = atan2(corner[1].y - mm.y , corner[1].x - mm.x);
	else output = atan2(corner[2].y - mm.y , corner[2].x - mm.x);

	return output;
}

double Triarea(CvPoint a,CvPoint b,CvPoint c) //삼각형 면적 계산 함수 
{
	double ax = a.x-c.x;
	double ay = a.y-c.y;
	double bx = b.x-c.x;
	double by = b.y-c.y;

	return 0.5*abs(ax*by-ay*bx);
}

double GetAngleABC(CvPoint a, CvPoint b, CvPoint c) //세점 사이의 각도를 측정하는 함수
{
    CvPoint ab = cvPoint(a.x - b.x, a.y - b.y); //점b를 원점으로 하고 a와 c사이의 각도를 계산한다.
    CvPoint cb = cvPoint(c.x - b.x, c.y - b.y);
 
    double dot = (ab.x * cb.x + ab.y * cb.y); // 벡터 ab와 cb의 내적값
    double cross = (ab.x * cb.y - ab.y * cb.x); // 벡터 ab와 cb의 외적값
	 
    double alpha = atan2(cross, dot); //역탄젠트 함수로 각도를 구한다.
 
    return alpha;
}

void trade_cvPoint(CvPoint *a,CvPoint *b) //x,y 좌표 교환 함수 
{
	int tempx = a->x;
	int tempy = a->y;

	a->x=b->x;
	a->y=b->y;

	b->x=tempx;
	b->y=tempy;
}

int free_rectlist(RectList* list) //할당 받은 구조체 RectList 해제 함수
{
	RectList* next=NULL;
	RectList* nows=list;
	while(nows!=NULL)
	{
		next = nows->next;
		free(nows);
		nows = next;
	}

	return 0;
}

int free_redbox(RedBoxList* boxlist) //할당 받은 구조체 RedBoxList 해제 함수
{
	RedBoxList* next=NULL;
	RedBoxList* nows=boxlist;
	while(nows!=NULL)
	{
		next = nows->next;
		free_rectlist(nows->Arrow_List);
		free(nows);
		nows = next;
	}

	return 0;
}

int time_text(char *text) //텍스트에 현재 날짜와 시간을 출력하는 함수
{
	if(text==NULL)return -1; //출력할 텍스트가 없으면 에러
	
	time_t now = time(NULL); //현재 날짜와 시간을 입력 받는다. (1970년 1월 1일 0시부터 함수를 호출할 때 까지의 초 카운트 )
	struct tm *times = localtime(&now); //32비트로 이루어진 날짜+시간 변수를 사용하기 편하게 구조체로 변환한다.
	times->tm_year += 1900; //입력받은 년도를 현재 달력과 같게 한다.
	
	char time_text[128]; //출력해야 할 날짜+시간 텍스트
	sprintf(time_text,"%d-%d-%d,%d-%d-%d",times->tm_year,times->tm_mon,times->tm_mday,times->tm_hour,times->tm_min,times->tm_sec);	
	
	strcat(text,time_text); //사용자가 원하는 텍스트에 날짜+시간을 이어 붙인다.
	
	return 0;
	
}


