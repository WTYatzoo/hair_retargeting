#include "head.h"
#include "camera.h"

using namespace std;
void camera::calVectorTo()
{
	vector_to=eyepoint-lookpoint;
}
void camera::calVectorUp()
{
	vectorUp.x=vector_to.y*vector_ud.z-vector_ud.y*vector_to.z;
	vectorUp.y=vector_to.z*vector_ud.x-vector_to.x*vector_ud.z;
	vectorUp.z=vector_to.x*vector_ud.y-vector_ud.x*vector_to.y;
}
void camera::calLength()
{
	length=sqrt(vector_to.len_sq());
}

void camera::calVector_UD()
{
	calVectorTo();

	vector_ud.x=vectorUp.y*vector_to.z-vector_to.y*vectorUp.z;
	vector_ud.y=vector_to.x*vectorUp.z-vectorUp.x*vector_to.z;
	vector_ud.z=vectorUp.x*vector_to.y-vector_to.x*vectorUp.y;
}

void camera::calEyePoint()
{
	eyepoint=vector_to+lookpoint;
}

camera::camera()
{
	lookpoint=myvector(0,0,0);
	this->length=40;
	eyepoint=myvector(0,-40,0);

	this->baseAngle=5;
	this->baseLength=1;

	vector_lr=myvector(0,0,1);
	vector_ud=myvector(1,0,0);

	calVectorTo();
	calVectorUp();

}

camera::~camera()
{
	;
}

void camera::matrixMult(double matrix[16],myvector& vector_now)
{
	double vec[3];
	int i;
	for(i=0;i<3;i++)
	{
		vec[i]=vector_now.x*matrix[i]+vector_now.y*matrix[i+4]+vector_now.z*matrix[i+8]+matrix[i+12]; //具体原理参见3D图形学 沃特著
	//	printf("%lf %lf %lf %lf test\n",matrix[i],matrix[i+4],matrix[i+8],matrix[i+12]);
	}
	i=3;
	//printf("%lf %lf %lf %lf test\n",matrix[i],matrix[i+4],matrix[i+8],matrix[i+12]);

	vector_now.set(vec[0],vec[1],vec[2]);
	return ;
}

void  camera::see()
{
	glMatrixMode(GL_MODELVIEW);  //model and view 既可以模型变换(移动模型，缩放模型，旋转模型等等) 也可以视图变换(移动照相机) 可以想象二者是可以互相转换的
	glLoadIdentity();
	gluLookAt(eyepoint.x,eyepoint.y,eyepoint.z,lookpoint.x,lookpoint.y,lookpoint.z,vectorUp.x,vectorUp.y,vectorUp.z);
}

void camera::rotate_LR(int lr)
{
	double matrix_now[16];
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glRotated(lr*baseAngle,vector_lr.x,vector_lr.y,vector_lr.z);
	glGetDoublev(GL_MODELVIEW_MATRIX,matrix_now);  //注意当前矩阵用一维向量保存时是按列优先存储的
	glPopMatrix();
	matrixMult(matrix_now,vector_to); //矩阵和列向量乘法运算
	calEyePoint();
	calVector_UD(); //重新计算旋转轴2

	see();
}

void camera::rotate_UD(int ud)
{
	double matrix_now[16]; 
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glRotated(ud*baseAngle,vector_ud.x,vector_ud.y,vector_ud.z);
	glGetDoublev(GL_MODELVIEW_MATRIX,matrix_now);
	glPopMatrix();
	matrixMult(matrix_now,vector_to);
	calEyePoint();
	calVectorUp(); //重新计算照相机向上的向量 

	see();

}

void camera::move(int cf) 
//放大时，因为是以中心点和视点的向量做平移变换，因此最大放大到视点位于中心点时即不可以继续放大，此处因为写法限制了观察范围
{
	double matrix_now[16];
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glTranslated(vector_to.x*cf*baseLength/length,vector_to.y*cf*baseLength/length,vector_to.z*cf*baseLength/length);
	glGetDoublev(GL_MODELVIEW_MATRIX,matrix_now);
	glPopMatrix();
	matrixMult(matrix_now,vector_to);
	calEyePoint();
	calVectorTo();
	calLength();
	
	see();
}
