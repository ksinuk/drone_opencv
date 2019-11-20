#include "header1.hpp"

RedBoxList* arrow_search(CvSeq* input_contour_red, IplImage* in_red, IplImage* in_black,int debugging) //표지판의 위치를 확인하는 함수
{
	if(input_contour_red==NULL  || in_red==NULL  || in_black==NULL) //표지판은 빨간색과 검은색으로 만들어 진다.
	{	if(debugging) printf("input_contour_red==NULL \n");
		return NULL;
	}

	CvSeq* red_pointer = input_contour_red;

	IplImage* desk_black = cvCloneImage(in_black); //검은색 화살표를 저장한 이미지
	IplImage* desk_redbox = cvCreateImage(cvGetSize(in_red), 8, 1);cvZero(desk_redbox); //표지판 1개를 임시로 그려내는 이미지

	IplImage* desk_and = cvCreateImage(cvGetSize(in_red), 8, 1);cvZero(desk_and); //표지판 안의 화살표를 그려내는 이미지

	RedBoxList* redbox_start=NULL; //출력할 표지판 리스트
	RedBoxList* redbox = NULL; //현재 만들고 있는 표지판 구조체
	RedBoxList* redbox_temp = NULL; //이전에 만들었던 표지판을 저장하는 임시 구조체

	int redbox_ctn=0; //지금까지 찾아낸 표지판의 갯수
	while(red_pointer!=NULL)
	{
		CvPoint center[4]; //표지판의 꼭지점
		cvZero(desk_redbox);cvZero(desk_and); //계산 전에 임시 이미지 초기화

		if(red_pointer->v_next!=NULL) 
		{
			if( quad_matching((red_pointer->v_next),center,0) <0.2) //빨간색 외곽선이 사각형 외각선이 맞는 경우
			{
				cvDrawContours(desk_redbox,red_pointer->v_next,255,255,0,-1); //사각형의 외곽선 안쪽까지 채워서 임시 이미지에 그려낸다.

				cvAnd(desk_redbox,desk_black,desk_and); //사각형 안에 있는 화살표를 그려낸다.

				CvSeq* arrow_seq=NULL; //표지판 안의 검은색 외곽선 리스트
				CvSeq* arrow_seq_start=NULL; //검은색 외곽선 리스트의 시작점
				CvMemStorage* arrow_mem = cvCreateMemStorage();
				cvFindContours(desk_and,arrow_mem,&arrow_seq_start,sizeof(CvContour),CV_RETR_CCOMP); //사각형의 안의 검은색 외곽선을 추출한다.
				arrow_seq=del_small_contour(arrow_seq_start,10,0); //작은 외곽선은 제거한다.

				RectList* arrow_start = NULL; //화살표 리스트의 시작점
				RectList* arrow = NULL; //화살표 리스트
				RectList* arrow_temp = NULL; //이전 화살표 리스트
			
				int arrow_ctn=0; //화살표의 갯수
				while(arrow_seq!=NULL) //사각형 안에 검은색 외곽선이 존재하는 경우
				{
					CvPoint tri_corner[3]; //삼각형의 꼭지점
					if(arrow_matching(arrow_seq,tri_corner,0)<0.2) //검은색 외곽선이 화살표가 맞을 경우
					{
						CvRect arrow_rect = cvBoundingRect(arrow_seq); //화살표의 위치 확인

						double angle_rad = calrad(tri_corner); //화살표의 방향 확인

						CvMoments arrow_moment; //화살표의 모멘트
						cvMoments(arrow_seq,&arrow_moment);
						int cogx = arrow_moment.m01/arrow_moment.m00; //화살표 무게중심의 좌표의 x값
						int cogy = arrow_moment.m10/arrow_moment.m00; //화살표 무게중심의 좌표의 y값
						CvPoint cog = cvPoint(cogx,cogy); //화살표 무게중심의 좌표
							
						/********** 화살표 리스트 생성 *************/
						if(arrow_ctn==0) //첫번째 화살표 리스트 생성
						{
							arrow_start = (RectList*)malloc(sizeof(RectList));
							arrow=arrow_start;
							arrow->prev = NULL;
						}
						else //두번째 이후 화살표 리스트 생성
						{
							arrow=(RectList*)malloc(sizeof(RectList));
							arrow->prev = arrow_temp;
							arrow->prev->next = arrow;
						}
						arrow->first = arrow_start;
						arrow->next = NULL;
						arrow->angle = angle_rad;
						arrow->Rect = arrow_rect;
						arrow->center = cog;

						arrow_temp = arrow; //지금 만들었던 화살표 리스트 임시 저장

						arrow_ctn++; //화살표의 갯수 증가
					}
					arrow_seq = arrow_seq->h_next; //다음 검은색 외곽선으로 넘어감
				}		

				if(arrow_ctn!=0) //빨간색 사각형 외곽선 안에 화살표가 있으면 표지판 리스트 생성
				{
					if(redbox_ctn==0) //첫번째 표지판 리스트 생성
					{
						redbox_start = (RedBoxList*)malloc(sizeof(RedBoxList));
						redbox = redbox_start;
						redbox->prev = NULL;
					}
					else //두번째 이후 표지판 리스트 생성
					{
						redbox = (RedBoxList*)malloc(sizeof(RedBoxList));
						redbox->prev =redbox_temp;
						redbox->prev->next = redbox;
					}
					redbox->first = redbox_start;
					redbox->next = NULL;

					redbox->corner[0] = center[0]; //표지판의 꼭지점 저장
					redbox->corner[1] = center[1];
					redbox->corner[2] = center[2];
					redbox->corner[3] = center[3];

					redbox_temp = redbox; //사용한 표지판 리스트 저장

					redbox_ctn++; //표지판의 갯수 증가

					redbox->Arrow_List = arrow_start; //표지판 안의 화살표 저장

				}

				clean_contour(arrow_seq_start); //외곽선 삭제
				cvClearMemStorage(arrow_mem); //외곽선 해제
			}
		}
		red_pointer = red_pointer->h_next; //다음 빨간색 외곽선으로 넘어감
	}
	check_redbox_angle(redbox_start,0); //각각의 표지판의 방향 결정, 저장

	cvReleaseImage(&desk_black); //임시 이미지 해제
	cvReleaseImage(&desk_redbox);
	cvReleaseImage(&desk_and);

	return redbox_start; //표지판의 리스트 출력
}

double arrow_matching(CvSeq* arrow_seq,CvPoint vertex[3],int debugging) //외곽선이 화살표인지 확인한다.
{
	double output = tri_matching(arrow_seq,vertex,0); //화살표는 삼각형 모양이다.
	return output;
}

int check_redbox_angle(RedBoxList* RedBox_1st,int debugging) //표지판의 방향을 계산한다.
{
	if(RedBox_1st==NULL)return -1;

	RedBoxList* RedBox = RedBox_1st; //계산중인 표지판
	RectList* arrow = RedBox->Arrow_List; //표지판 안의 화살표

	while(RedBox!=NULL)
	{
		arrow = RedBox->Arrow_List;
		if(arrow!=NULL)
		{
			double arrow_angle = 0; //화살표 각도의 합
			int ctn = 0; //화살표의 갯수
			while(arrow!=NULL)
			{
				arrow_angle += arrow->angle;
				arrow = arrow->next;
				ctn++;
			}
			RedBox->angle = arrow_angle/ctn; //화살표 방향의 평균값을 표지판의 방향으로 한다.
		}
		else RedBox->angle = 1000; //화살표가 없으므로 표지판이 아니다.
		RedBox = RedBox->next;
	}

	return 0;
}

void paint_redbox(RedBoxList* RedBox_start,IplImage *output, CvScalar color_box, CvScalar color_arrow,int debugging)
{   //표지판과 화살표의 위치를 화면에 그려낸다.
	RedBoxList *RedBox=RedBox_start;
	
	while(RedBox!=NULL)
	{
		cvLine(output,RedBox->corner[0],RedBox->corner[1],color_box,3); //표지판의 사각형 테두리를 그려낸다.
		cvLine(output,RedBox->corner[1],RedBox->corner[2],color_box,3);
		cvLine(output,RedBox->corner[2],RedBox->corner[3],color_box,3);
		cvLine(output,RedBox->corner[3],RedBox->corner[0],color_box,3);

		CvPoint redbox_center = cvPoint(0,0); //표지판의 중심에 화살표의 방향을 그린다.
		for(int i=0;i<4;i++)
		{	redbox_center.x += RedBox->corner[i].x/4;
			redbox_center.y += RedBox->corner[i].y/4;
		}
		CvScalar whiteColor = cvScalar(255,255,255);
		cvCircle(output, redbox_center, 20, whiteColor,3);
		cvLine(output,redbox_center,cvPoint(redbox_center.x+100*cos(RedBox->angle) , redbox_center.y +100*sin(RedBox->angle)),whiteColor,3);

		RectList *arrow=RedBox->Arrow_List; //표지판 안의 화살표를 화면에 그린다.
		while(arrow!=NULL)
		{
			CvPoint ptt1 = cvPoint(arrow->Rect.x,arrow->Rect.y);
			CvPoint ptt2 = ptt1;
			ptt2.x += arrow->Rect.width;
			ptt2.y += arrow->Rect.height;
			cvRectangle(output,ptt1,ptt2,color_arrow,3);

			arrow=arrow->next;
		}

		RedBox=RedBox->next;
	}
}

