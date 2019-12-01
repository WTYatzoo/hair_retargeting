#include "head.h"
#include "test.h"
using namespace std;

void saveAsVTK()
{
  
  stringstream ss;
  string count_s;
  ss<<count_num;
  ss>>count_s;
  string name="./res_now/myobeject_";
  name+=count_s;
  name+=".vtk";
  FILE *fp;
  int i;
  printf("%s \n",name.c_str());
  fp=fopen(name.c_str(),"w");
  fprintf(fp,"# vtk DataFile Version 2.0\n");
  fprintf(fp,"quad\n");
  fprintf(fp,"ASCII\n");
  fprintf(fp,"DATASET UNSTRUCTURED_GRID\n");
  fprintf(fp,"POINTS %d double\n",myobject->num_vertex);
  for(i=0;i<myobject->num_vertex;i++)
    {
      fprintf(fp,"%lf %lf %lf\n",myobject->myvertexs[i].location_deform.x,myobject->myvertexs[i].location_deform.y,myobject->myvertexs[i].location_deform.z);
    }
  fprintf(fp,"CELLS %d %d\n",myobject->num_face,myobject->num_face*5);
  for(i=0;i<myobject->num_face;i++)
    {
      fprintf(fp,"4 %d %d %d %d\n",myobject->myquads[i].index_vertex[0],myobject->myquads[i].index_vertex[1],myobject->myquads[i].index_vertex[2],myobject->myquads[i].index_vertex[3]);
    }
  fprintf(fp,"CELL_TYPES %d\n",myobject->num_face);
  for(i=0;i<myobject->num_face;i++)
    {
      fprintf(fp,"9\n");
    }

  fclose(fp);
  
}
void saveBmp(const char* name,int width,int height,unsigned char* data)
{
	/*BITMAPFILEHEADER hdr;
	BITMAPINFOHEADER infoHdr;
	infoHdr.biSize=40;
	infoHdr.biWidth=width;
	infoHdr.biHeight =height;
	infoHdr.biPlanes =1;
	infoHdr.biBitCount =24;
	infoHdr.biCompression =0;
	infoHdr.biSizeImage=width*height*3;
	infoHdr.biXPelsPerMeter=0;
	infoHdr.biYPelsPerMeter=0;
	infoHdr.biClrUsed=0;
	infoHdr.biClrImportant=0;
	
	hdr.bfType = 0x4D42;
	hdr.bfReserved1 = 0;
	hdr.bfReserved2 = 0;
	hdr.bfOffBits = 54;
	hdr.bfSize =(DWORD)(sizeof(BITMAPFILEHEADER)+sizeof(BITMAPINFOHEADER)+width* height * 3);
	
	unsigned char x;
	for(int i=0;i<infoHdr.biSizeImage;i+=3)
	{
		x=data[i];
		data[i]=data[i+2];
		data[i+2]=x;
	}
	
	FILE *fd;
	fd=fopen(name,"wb");
	fwrite(&hdr,sizeof(BITMAPFILEHEADER),1,fd);
	fwrite(&infoHdr,sizeof(BITMAPINFOHEADER),1,fd);
	fwrite(data,width* height*3,1,fd);
	fclose(fd);*/
}


void saveSceneImage()
{
	int view[4];
	glGetIntegerv(GL_VIEWPORT,view);
	int bufferSize=view[2]*view[3]*3;
	void* color_buffer=malloc(bufferSize);
	glReadPixels(view[0],view[1],view[2],view[3],GL_RGB,GL_UNSIGNED_BYTE,color_buffer);
	
	string st,loc;
	loc="D:/test/test4/";
	stringstream  ss;
	ss<<count_num;
	ss>>st;
	st+=".bmp";
	loc+=st;
	saveBmp(loc.c_str(),view[2],view[3],(unsigned char*)color_buffer);
	free(color_buffer);	
}


void drawObject()
{
	switch(kind)
	{
	case 0:
		myobject->testdraw();
		break;
	}
}

void display(void)
{
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	drawObject();

	glutSwapBuffers(); //双缓冲时开启
	glFlush();
	
	glutPostRedisplay();
}

void makeSmooth()
{
	glEnable(GL_LINE_SMOOTH); 
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_LINE_SMOOTH_HINT,GL_DONT_CARE);
	glLineWidth(1.0);
	
	glPointSize(1.0);
	
	glEnable(GLUT_MULTISAMPLE);
	glEnable(GL_POLYGON_SMOOTH);
}

void lightControl()
{
	float light_position[] ={0,-10,0,0};
	float white_light[] ={1,1,1.5,1};
	float lmod_ambient[]={0.5,0.5,0.65,1}; 
	float spot_direction[]={0,1,0}; //聚光灯的轴的向量方向
	glShadeModel(GL_SMOOTH);
	glLightfv(GL_LIGHT0,GL_POSITION,light_position);
	glLightfv(GL_LIGHT0,GL_DIFFUSE,white_light);
	glLightfv(GL_LIGHT0,GL_SPECULAR,white_light);
//	glLightf(GL_LIGHT0,GL_SPOT_CUTOFF,90);
//	glLightfv(GL_LIGHT0,GL_SPOT_DIRECTION,spot_direction);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT,lmod_ambient);

	//glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_NORMALIZE);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST); 
	glClearDepth(1.0);

	glEnable(GL_TEXTURE_2D);

}

void init()
{
	glutInitDisplayMode(GLUT_DEPTH|GLUT_DOUBLE|GLUT_RGBA) ;  // 双缓冲 RGBA(A表示透明度) 开深度
	glutInitWindowSize(400,400) ; //定义窗口位置 
	glutInitWindowPosition(100,100) ; //创建窗口 
	glutCreateWindow("curvature_test") ; 	
	glClearColor(1,1,1,1);
    makeSmooth(); // 抗锯齿	

	//反面剔除 否则有视觉错误
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);

	lightControl();

	myobject=new object();
	mycamera=new camera();
	kind=0; //测试绘制
	count_num=0;
}

void reshape(int w,int h) //首次绘制和屏幕大小调节时glutReshapeFunc()调用该函数
{
	glViewport(0,0,w,h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(60.0,(double)w/(double)h,1,500);  
	//  void APIENTRY gluPerspective ( GLdouble fovy, GLdouble aspect, GLdouble zNear, GLdouble zFar) 
	//最后两个参数必须是正数，否则会出现投影计算异常，深度值计算异常，不可以为零
	
	glMatrixMode(GL_MODELVIEW);
	mycamera->see();
}


void keyboard(unsigned char key,int x,int y)
{
	switch(key)
	{
	case 'a':
		mycamera->rotate_LR(1);  
		break;
	case 'd':
		mycamera->rotate_LR(-1); //考虑是照相机在移动，因此按d呈现物体逆时针旋转，其实是相机顺时针旋转
		break;
	case 'w':
		mycamera->rotate_UD(1);
		break;
	case 's':
		mycamera->rotate_UD(-1);
		break;
	case 'k':
		mycamera->move(-1);
		break;
	case 'l':
		mycamera->move(1);
		break;
	case  'q':
	  //	saveSceneImage();
	        saveAsVTK();
		count_num++;
		break;
	case 'o':
		exit(0);
		break;
	}
	glutPostRedisplay();
}

int main(int argc,char**argv)
{ 
	glutInit(&argc,argv) ; //显示模式初始化 
	init();	

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutMainLoop() ; 

	
	return 0 ;
}
