#include <opencv2/opencv.hpp> 
#include <raspicam/raspicam_cv.h>
#include "header_define.hpp"

#define MY_PI 3.14159265358979
/******** 함수 설명은 각각의 소스 참조 ***********/

struct RectList //화살표 연결 리스트
{
	CvRect Rect; //화살표의 크기와 위치
	double angle; //화살의 회전 각도
	CvPoint center; //화살의 중심점
	struct RectList* first; 
	struct RectList* prev;
	struct RectList* next;
};

struct RedBoxList //표지판 연결 리스트
{
	RectList* Arrow_List; //표지판 안에 있는 화살표 모음
	CvPoint corner[4]; //표지판의 꼭지점
	double angle; //표지판의 회전 각도
	struct RedBoxList* first;
	struct RedBoxList* prev;
	struct RedBoxList* next;
};

class LocateDrone //가속도 센서 관련 클래스
{
private:
	int trans; //송수신 성공 여부
	float ax,ay,az,size_acc; //가속도 값과 크기
	float vx,vy,vz; //속도
	float x,y,z; //이동거리
	float gax,gay,gaz; //회전속도
	float gx,gy,gz; //회전량
	float dax, day , daz; //가속도의 오차값
	float dgax, dgay , dgaz; //회전속도의 오차값

	int MPU_fd;
	struct timespec before_time;
public:
	LocateDrone(int i);
	int I2C_began();
	void whereis();
	float time_gap_check();
	int prn_locate();
	float read_data(int addr);
	int read_mpu6050();
	int reset_error_data();	
	int ok();
	int wirte_data(FILE *fd);
	int make_textfile(FILE *fb);
};

/************** main *******************/
int main_camera(double earth_rad,int road0point);
int manual_camera();

/****** arrow source function *****/
RedBoxList* arrow_search(CvSeq* input_contour_red, IplImage* in_red, IplImage* in_black,int debugging);
void paint_redbox(RedBoxList* RedBox_start,IplImage *output, CvScalar color_box, CvScalar color_arrow,int debugging);
double arrow_matching(CvSeq* arrow_seq,CvPoint vertex[3],int debugging);
int check_redbox_angle(RedBoxList* RedBox,int debugging);

/************** fillter *********************/
int light_removal(IplImage* img,IplImage* out,int debugging);
CvSeq* del_binerror(IplImage* img,CvMemStorage* storge,int opt,double area);

/******************* free mem **************************/
int free_rectlist(RectList* list);
int free_redbox(RedBoxList* boxlist);

/****** sub sourece function *****/
CvSeq* del_small_contour(CvSeq* firstcontour,double length,int debuging);
int clean_contour(CvSeq* firstcontour,int debuging=0);
double tri_matching(CvSeq* contour,CvPoint* center,int debugging);
double quad_matching(CvSeq* contour,CvPoint center[] ,int debugging);


/***** mini sourece function****/
void BubbleSort(int *ar, int num ,int dir=0);
double calrad(CvPoint corner[]);
double Triarea(CvPoint a,CvPoint b,CvPoint c);
double GetAngleABC(CvPoint a, CvPoint b, CvPoint c);
void trade_cvPoint(CvPoint *a,CvPoint *b);
int time_text(char *text);
























