#ifndef _CAMERA_
#define _CAMERA_

#include "myvector.h"

class camera
{
public:
	myvector eyepoint;
	myvector lookpoint; //固定
	
	
	myvector vector_to; //视点指向相机位置的向量
	
	double length; //相机位置距离目标点的距离长度
	
	double baseLength; //一次沿向量移动的移动增量  给定
	
	myvector vector_lr; //旋转轴1  固定
	myvector vector_ud;// 旋转轴2  
	
	double baseAngle;//一次旋转的角度增量 给定
	
	myvector vectorUp;//照相机向上的向量
	
	camera();
	
	~camera();
	
	void calVectorTo();
	void calLength();
	void calVectorUp();
	void calVector_UD();
	void calEyePoint();
	
	void see();
	
	void rotate_LR(int lr);
	void rotate_UD(int ud);
	void move(int cf);
	
	void matrixMult(double matrix[16],myvector& vector_now);
};
#endif

