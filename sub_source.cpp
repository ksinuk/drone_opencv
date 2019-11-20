#include "header1.hpp"

CvSeq* del_small_contour(CvSeq* firstcontour,double length,int debuging) //면적이 일정값보다 작은 외각선 삭제
{
	CvSeq* CvSeq_pointer=firstcontour;
	CvSeq* next_pointer=NULL;

	if( firstcontour==NULL)return NULL;

	while(CvSeq_pointer!=NULL)
	{
		if((cvContourArea(CvSeq_pointer)<length)) //삭제해야 할 외곽선이 있는 경우
		{
			if(CvSeq_pointer==firstcontour) firstcontour = CvSeq_pointer->h_next; //삭제 대상이 첫번째 외곽선 일 경우
			if(CvSeq_pointer->h_prev != NULL) CvSeq_pointer->h_prev->h_next = CvSeq_pointer->h_next; //이전 외곽선의 next 포인터를 변경
			if(CvSeq_pointer->h_next != NULL) CvSeq_pointer->h_next->h_prev = CvSeq_pointer->h_prev; //다음 외곽선의 prev 포인터를 변경
			next_pointer = CvSeq_pointer->h_next; //다음에 삭제 여부를 확인 할 외곽선 저장 

			if(CvSeq_pointer->v_next != NULL) clean_contour(CvSeq_pointer->v_next,0); //외관선 안쪽에도 외곽선도 삭제 한다.
			cvClearSeq(CvSeq_pointer); //면적이 작은 외곽선을 삭제한다.

			CvSeq_pointer = next_pointer; //다음 외곽선으로 넘어간다.
		}
		else if(CvSeq_pointer->v_next != NULL) //면적이 큰 외곽선 안에 외곽선이 있는 경우
		{
			CvSeq_pointer->v_next = del_small_contour(CvSeq_pointer->v_next,length,0); //안쪽 외곽선을 제귀호출해서 작은 외곽선 삭제
			CvSeq_pointer=CvSeq_pointer->h_next; //다음 외곽선으로 넘어간다.
		}
		else
		{
			CvSeq_pointer=CvSeq_pointer->h_next; //다음 외곽선으로 넘어간다.
		}

		if(firstcontour==NULL) break; //모든 외곽선을 삭제 했을 경우
	}

	return firstcontour;
}

int clean_contour(CvSeq* firstcontour,int debuging) //외곽선을 모두 삭제한다.
{
	CvSeq* contour = firstcontour;
	CvSeq* temp_seq = NULL;

	//if(firstcontour->v_prev != NULL) firstcontour->v_prev->v_next = NULL;

	while(contour!=NULL)
	{
		temp_seq = contour->h_next;
		if(contour->v_next!=NULL)clean_contour(contour->v_next,0); //외곽선의 안쪽 외곽선도 삭제한다.
		cvClearSeq(contour); //외곽선 삭제 함수
		contour = temp_seq;
	}

	return 0;
}

double tri_matching(CvSeq* contour,CvPoint* center,int debugging) //외곽선이 삼각형인 확인하는 함수
{
	if(contour==NULL) return -1; //외곽선이 없으면 에러

	//외곽선에서 임의의 점을 하나의 좌표를 찾는다. 
	CvPoint* p_1st = (CvPoint*)cvGetSeqElem(contour,0); //외곽선의 임의의 좌표 , 계산 이후에는 삼각형의 꼭지점
	int index_1st = 0; // 좌표 p_1st의 외곽선 상에서의 위치
	CvPoint* p_2nd = NULL; //삼각형의 꼭지점
	int index_last = 0; // 좌표 p_2nd의 외곽선 상에서의 위치

	for(int ctn = 0; ctn<2; ctn++) //2개의 삼각형의 꼭지점 찾기
	{
		/*삼각형 외곽선의 임의의 점에서 가장 먼 곳에 위치한 외각선 위의 점은 삼각형의 꼭지점이다.*/
		int max_i = 0;
		double max_len = 0;
		for(int i = 1; i<contour->total; i++)
		{
			CvPoint* p = (CvPoint*)cvGetSeqElem(contour,i);
			double len = (p_1st->x - p->x)*(p_1st->x - p->x)+(p_1st->y - p->y)*(p_1st->y - p->y);
			if(len>max_len)
			{
				max_i= i;
				max_len = len;
			}
		}

		if(ctn==0) //삼각형의 첫번째 꼭지점
		{
			p_1st = (CvPoint*)cvGetSeqElem(contour,max_i);
			index_1st = max_i;
		}
		else if(ctn==1) //삼각형의 두번째 꼭지점
		{
			p_2nd = (CvPoint*)cvGetSeqElem(contour,max_i);
			index_last = max_i;
		}
	}

	/*삼각형의 꼭지점 중 2개의 위치를 알고 있을 경우 남은 하나는 삼각형의 면적을 이용한다.
	  3 꼭지점 사이의 넓이는 2개의 꼭지점과 다른 점 사이의 넓이 보다 크다는 것을 이용한다.*/
	CvPoint* p_3rd = NULL; //삼각형의 마지막 꼭지점 찾기
	double max_area = 0; //3꼭지점 사이의 넓이
	for(int i = 0; i<contour->total; i++)
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem(contour,i);
		double tri_area = Triarea(*p_1st,*p_2nd,*p); //삼각형의 넓이를 알아내는 함수
		if(tri_area>max_area)
		{
			max_area = tri_area;
			p_3rd = p;
		}
	}
	*(center) = *p_1st; //예상되는 삼각형의 꼭지점을 반환한다.
	*(center+1) = *p_2nd;
	*(center+2) = *p_3rd;

	double real_area = cvContourArea(contour); //실제 외곽선 안의 면적을 계산한다.
	double output=abs(1-max_area/real_area); //세 꼭지점 넓이의 최대값과 실제 면적의 비율을 출력한다.
	return output;
}

double quad_matching(CvSeq* contour,CvPoint center[] ,int debugging) //외곽선이 사각형인 확인하는 함수
{
	if(contour==NULL) return -1; //외곽선이 없으면 에러

	//삼각형과 마찬가지로 임의의 점에서 가정 먼 점을 사각형의 꼭지점으로 한다.
	CvPoint* p_1st = (CvPoint*)cvGetSeqElem(contour,0); //사각형의 첫번째 꼭지점 , 사각형의 임의의 점
	int index_1st = 0; // 좌표 p_1st의 외곽선 상에서의 위치
	CvPoint* p_2nd = NULL; //사각형의 두번째 꼭지점
	int index_2nd = 0; // 좌표 p_2nd의 외곽선 상에서의 위치
	for(int ctn = 0; ctn<2; ctn++)
	{
		int max_i = 0;
		double max_len = 0;
		for(int i = 1; i<contour->total; i++)
		{
			CvPoint* p = (CvPoint*)cvGetSeqElem(contour,i);
			double len = (p_1st->x - p->x)*(p_1st->x - p->x)+(p_1st->y - p->y)*(p_1st->y - p->y);
			if(len>max_len)
			{
				max_i= i;
				max_len = len;
			}
		}
		if(ctn==0)
		{
			p_1st = (CvPoint*)cvGetSeqElem(contour,max_i);
			index_1st = max_i;
		}
		else if(ctn==1)
		{
			p_2nd = (CvPoint*)cvGetSeqElem(contour,max_i);
			index_2nd = max_i;
		}
	}

	//사각형을 2개의 삼각형으로 나눈뒤 면적이 큰 삼각형의 3번째 꼭지점을 알아낸다.
	//3번째 꼭지점은 삼각형과 마찬가지로 3점의 면적이 최대로 만드는 점으로 한다.
	double max_area_tri = 0;
	int index_3rd = 0;
	for(int i = 0; i<contour->total; i++)
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem(contour,i);
		double tri_area = Triarea(*p_1st,*p_2nd,*p);
		if(tri_area>max_area_tri)
		{
			max_area_tri = tri_area;
			index_3rd = i;
		}
	}

	//3개의 꼭지점을 순서대로 정렬한다.
	int iarray[3]={index_1st,index_2nd,index_3rd}; //3꼭지점의 위치
	BubbleSort(iarray,3); //꼭지점 정렬
	CvPoint p_array[3]; //3개의 꼭지점의 좌표
	for(int i = 0;i<3; i++)
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem(contour,iarray[i]);
		p_array[i] = *p;
	}

	//사각형 안의 2개의 삼각형 중에서 크기가 작은 삼각형의 꼭지점을 알아낸다.
	//3개의 꼭지점 중 인접한 2개의 꼭지점과 그 사이의 점 사이의 넓이가 최대가 하는 점을 찾는다.
	double max_area2 = 0;
	int index_4th = 0;
	for(int i = iarray[0]; i<iarray[1]; i++) //마지막 꼭지점이 꼭지점0과 1 사이에 있다고 가정
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem(contour,i);
		double tri_area = Triarea(p_array[0],p_array[1],*p);
		if(tri_area>max_area2)
		{
			max_area2 = tri_area;
			index_4th = i;
		}
	}
	for(int i = iarray[1]; i<iarray[2]; i++) //마지막 꼭지점이 꼭지점1과 2 사이에 있다고 가정
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem(contour,i);
		double tri_area = Triarea(p_array[1],p_array[2],*p);
		if(tri_area>max_area2)
		{
			max_area2 = tri_area;
			index_4th = i;
		}
	}
	for(int i = iarray[2]; i< contour->total; i++) //마지막 꼭지점이 꼭지점2와 마지막 점 사이에 있다고 가정
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem(contour,i);
		double tri_area = Triarea(p_array[2],p_array[0],*p);
		if(tri_area>max_area2)
		{
			max_area2 = tri_area;
			index_4th = i;
		}
	}
	for(int i = 0; i< iarray[0]; i++) //마지막 꼭지점이 첫번째 꼭지점과 0 사이에 있다고 가정
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem(contour,i);
		double tri_area = Triarea(p_array[2],p_array[0],*p);
		if(tri_area>max_area2)
		{
			max_area2 = tri_area;
			index_4th = i;
		}
	}
	
	int outarray[4]={index_1st,index_2nd,index_3rd,index_4th}; //찾아낸 4개의 꼭지점을 순서대로 정렬한다.
	BubbleSort(outarray,4);
	for(int i = 0; i<4; i++) //4개의 꼭지검을 출력한다.
	{
		CvPoint* p = (CvPoint*)cvGetSeqElem(contour,outarray[i]);
		center[i] = *p;
	}
	
	double real_area = cvContourArea(contour); //실제 외곽선 안의 넓이
	double quad_area = max_area_tri + max_area2; //2개의 삼각형을 더한 4개의 꼭지점의 넓이
	double output=abs(1-quad_area/real_area); //계산한 넓이와 실제 넓이의 비율을 출력한다.
	return output;
}



