#include "head.h"
#include "vertex_grid.h"
#include "grid.h"
#include "object.h"
#include "io.h"

using namespace std;
using namespace Eigen;
using namespace boost;
using namespace Json;
#define min(a,b) (((a) < (b)) ? (a) : (b))

static int vertex_collection[2][5][4][3]={
  { 
    {
      {
	0,0,0
	  },
	{
	  1,1,0
	    },
	  {
	    0,1,1
	      },
	    { 
	      1,0,1
		}
    },
      {
	{
	  0,1,1
	    },
	  {
	    1,0,1
	      },
	    {
	      1,1,1
		},
	      {
		1,1,0
		  }
      },
	{
	  {
	    0,0,0
	      },
	    {
	      0,0,1
		},
	      {
		0,1,1
		  },
		{
		  1,0,1
		    }
	},
	  {
	    {
	      0,0,0
		},
	      {
		1,1,0
		  },
		{
		  0,1,0
		    },
		  {
		    0,1,1
		      }
	  },
	    {
	      {
		0,0,0
		  },
		{
		  1,0,0
		    },
		  {
		    1,1,0
		      },
		    {
		      1,0,1
			}
	    }
  },
    { 
      {
	{
	  0,0,1
	    },
	  {
	    1,1,1
	      },
	    {
	      0,1,0
		},
	      {
		1,0,0	
		  }
      },
	{
	  {
	    0,0,1
	      },
	    {
	      0,1,1
		},
	      {
		1,1,1
		  },
		{
		  0,1,0
		    }
	},
	  {
	    {
	      0,0,1
		},
	      {
		1,0,1
		  },
		{
		  1,1,1
		    },
		  {
		    1,0,0
		      }
	  },
	    {
	      {
		0,0,0
		  },
		{
		  0,1,0
		    },
		  {
		    1,0,0
		      },
		    {
		      0,0,1
			}
	    },
	      {
		{
		  1,0,0
		    },
		  {
		    0,1,0
		      },
		    {
		      1,1,0
			},
		      {
			1,1,1
			  }
	      }
    }
};

object::object()
{
  getObjData();
  double h;
  vector<pair<myvector,myvector> > controlPointsPair;
  {
    //homer.txt//h=0.1;
    h=3.0;//head
    // controlPointsPair.push_back(make_pair(myvector(0,0,0),myvector(1,1,1)));

    //controlPointsPair.push_back(make_pair(myvector(-0.28,0.06,0.13),myvector(-0.28,0.06,0.13)));
    //controlPointsPair.push_back(make_pair(myvector(0.28,0.06,0.13),myvector(0.28,0.06,0.13)));
    //controlPointsPair.push_back(make_pair(myvector(0,0,0),myvector(0,0,0)));
    // controlPointsPair.push_back(make_pair(myvector(0,0.5,0),myvector(0,0.5,0.3)));
    // controlPointsPair.push_back(make_pair(myvector(0.28,-0.5,0),myvector(0.4,-0.22,0)));
    /*
      controlPointsPair.push_back(make_pair(myvector(0.31,0.55,0.05),myvector(0.4,0.55,0.25)));
      controlPointsPair.push_back(make_pair(myvector(0.31,0.55,0.1),myvector(0.4,0.55,0.3)));
      controlPointsPair.push_back(make_pair(myvector(0.31,0.55,0.15),myvector(0.4,0.55,0.35)));
      controlPointsPair.push_back(make_pair(myvector(0.31,0.55,-0.05),myvector(0.4,0.55,0.15)));
      controlPointsPair.push_back(make_pair(myvector(0.31,0.55,-0.1),myvector(0.4,0.55,0.1)));
    
      controlPointsPair.push_back(make_pair(myvector(-0.28,0.55,0.05),myvector(-0.4,0.55,0.05)));
      controlPointsPair.push_back(make_pair(myvector(-0.28,0.55,0.1),myvector(-0.4,0.55,0.1)));
      controlPointsPair.push_back(make_pair(myvector(-0.28,0.55,0.15),myvector(-0.4,0.55,0.15)));
      controlPointsPair.push_back(make_pair(myvector(-0.28,0.55,-0.05),myvector(-0.4,0.55,-0.05)));
      controlPointsPair.push_back(make_pair(myvector(-0.28,0.55,-0.1),myvector(-0.4,0.55,-0.1)));
    
      controlPointsPair.push_back(make_pair(myvector(0.31,-0.5,0.05),myvector(0.4,-0.5,0.05)));
      controlPointsPair.push_back(make_pair(myvector(0.31,-0.5,0.1),myvector(0.4,-0.5,0.1)));
      controlPointsPair.push_back(make_pair(myvector(0.31,-0.5,0.15),myvector(0.4,-0.5,0.15)));
      controlPointsPair.push_back(make_pair(myvector(0.31,-0.5,-0.05),myvector(0.4,-0.5,-0.05)));
      controlPointsPair.push_back(make_pair(myvector(0.31,-0.5,-0.1),myvector(0.4,-0.5,-0.1)));
    
      controlPointsPair.push_back(make_pair(myvector(-0.28,-0.5,0.05),myvector(-0.4,-0.5,0.05)));
      controlPointsPair.push_back(make_pair(myvector(-0.28,-0.5,0.1),myvector(-0.4,-0.5,0.1)));
      controlPointsPair.push_back(make_pair(myvector(-0.28,-0.5,0.15),myvector(-0.4,-0.5,0.15)));
      controlPointsPair.push_back(make_pair(myvector(-0.28,-0.5,-0.05),myvector(-0.4,-0.5,-0.05)));
      controlPointsPair.push_back(make_pair(myvector(-0.28,-0.5,-0.1),myvector(-0.4,-0.5,-0.1)));
   
      //  controlPointsPair.push_back(make_pair(myvector(0.28,-0.5,-0.15),myvector(0.4,-0.22,-0.15)));
      */   
  }
  deform(myvertexs_head,h,controlPointsPair);

  for(int i=0;i<num_vertex_head;i++)
    {
      int index_now=myvertexs_head_index[i];
      myvertexs[index_now].location_deform=myvertexs_head[i].location_deform;
    }
  
  for(int i=num_vertex_head;i<num_vertex_hair_head;i++)
    {
      int index_now=myvertexs_head_index[i];
      myvertexs_hair[index_now].location=myvertexs_head[i].location_deform;
    }
  string name_hair_deformed_vtk="./res_now/hair_deformed.vtk";
  
  io myio=io();
  myio.saveHairAsVTK(myvertexs_hair,myedges_hair,name_hair_deformed_vtk);

  /////////////////////////////////////////////////////////////////////
  int ct_now=0;
  for(int i=0;i<num_curve;i++)
    {
      int curve_vertex_num=mycurves[i].curve_vertex.size();
      for(int j=0;j<curve_vertex_num;j++)
	{
	  mycurves[i].curve_vertex[j].location=myvertexs_hair[ct_now].location;
	  ct_now++;
	}
    }
  
  /////////////////////////////////////////////////////////////////////
 
  string name_hair_json_output="./res_now/hair_deformed.json";
  Value root_json;
  for(int i=0;i<num_curve;i++)
    {
      Value pt;
      for(int j=0;j<mycurves[i].curve_vertex.size();j++)
	{
	  Value loc;
	  loc.append(mycurves[i].curve_vertex[j].location.x);
	  loc.append(mycurves[i].curve_vertex[j].location.y);
	  loc.append(mycurves[i].curve_vertex[j].location.z);
	  pt.append(loc);
	}
      root_json[mycurves[i].name]=pt;
    }
  Json::StyledWriter sw;

  // Json::FastWriter sw;
  ofstream os;
  os.open(name_hair_json_output);
  os << sw.write(root_json);
  os.close();
}

object::~object()
{
	
}

void  object::testdraw()
{
  int i;
  int siz=myquads.size();
  quad now;
  glColor3d(0,1,1);
  for(i=0;i<siz;i++)
    {
      now=myquads[i];
      glBegin(GL_POLYGON);
      {	
	glVertex3d(myvertexs[now.index_vertex[0]].location_deform.x,myvertexs[now.index_vertex[0]].location_deform.y,myvertexs[now.index_vertex[0]].location_deform.z);
	glVertex3d(myvertexs[now.index_vertex[1]].location_deform.x,myvertexs[now.index_vertex[1]].location_deform.y,myvertexs[now.index_vertex[1]].location_deform.z);
	glVertex3d(myvertexs[now.index_vertex[2]].location_deform.x,myvertexs[now.index_vertex[2]].location_deform.y,myvertexs[now.index_vertex[2]].location_deform.z);
	glVertex3d(myvertexs[now.index_vertex[3]].location_deform.x,myvertexs[now.index_vertex[3]].location_deform.y,myvertexs[now.index_vertex[3]].location_deform.z);	        
      }
      glEnd();
    }
}

void object::getObjData()
{
  /*
  string name_0="/home/wtyatzoo/project/model/xmov/face/face_origin.obj";
  string name_1="/home/wtyatzoo/project/model/xmov/face/face_1.obj";
  
  string name_hair="/home/wtyatzoo/project/model/xmov/face/hair_origin.obj";
  string name_hair_json="/home/wtyatzoo/project/model/xmov/face/hair_origin.json";

  string name_hair_vtk="./res_now/hair_origin.vtk";
  string name_hair_json_output="./res_now/hair_origin.json";
  */

  string name_0="/home/wtyatzoo/project/model/xmov/face/test/obj/Topo_A_Body_Original.obj";
  string name_1="/home/wtyatzoo/project/model/xmov/face/test/obj/Topo_A_Body_Adj.obj";
  
  string name_hair="/home/wtyatzoo/project/model/xmov/face/hair_origin.obj";
  //string name_hair_json="/home/wtyatzoo/project/model/xmov/face/test/maya/test.json";
  string name_hair_json="/home/wtyatzoo/project/model/xmov/face/test/test.json";
  
  string name_hair_vtk="./res_now/hair_origin.vtk";
  string name_hair_json_output="./res_now/hair_origin.json";

  //////////////////////////////////////////////////////
  
  property_tree::ptree root;
  property_tree::ptree items;
  property_tree::read_json<property_tree::ptree>(name_hair_json, root);

  
  for (boost::property_tree::ptree::iterator it = root.begin(); it != root.end(); ++it)
    {
      curve curve_now;
      string key = it->first;
      //cout<<key<<endl;
      curve_now.name=key;
      property_tree::ptree pt_1 = root.get_child(key);
      for (property_tree::ptree::iterator it_1 = pt_1.begin(); it_1 != pt_1.end(); ++it_1)
	{
	  property_tree::ptree pt_2=it_1->second;
	  int count=0;
	  double loc[3];
	  for(property_tree::ptree::iterator it_2 = pt_2.begin(); it_2 != pt_2.end(); ++it_2)
	    {
	      double now = it_2->second.get_value<double>();
	      loc[count]=now;
	      count++;
	      //cout<<now<<endl;
	    }
	  vertex v_now=vertex(myvector(loc[0],loc[1],loc[2]));
	  myvertexs_hair.push_back(v_now);
	  curve_now.curve_vertex.push_back(v_now);
	}
      mycurves.push_back(curve_now);
    }

  printf("curve size:%d\n",mycurves.size());
  num_curve=mycurves.size();

  int max_rand=128;
  int num_vertex_hair_count=0;
  for(int i=0;i<num_curve;i++)
    {
      int num_vertex_now=mycurves[i].curve_vertex.size();
      double c0=rand()%max_rand/(double)(max_rand+1);
      double c1=rand()%max_rand/(double)(max_rand+1);
      double c2=rand()%max_rand/(double)(max_rand+1);
      for(int j=0;j<num_vertex_now-1;j++)
	{
	  e e_here;
	  e_here.v1=num_vertex_hair_count+j;
	  e_here.v2=num_vertex_hair_count+j+1;
	  e_here.c0=c0;
	  e_here.c1=c1;
	  e_here.c2=c2;
	  myedges_hair.push_back(e_here);
	}
      num_vertex_hair_count+=num_vertex_now;
    }

  //////////////////////////////////////////////////////
  
  io myio=io();
  myio.getVertexAndQuadFromObj(myvertexs,myquads,name_0);
  num_vertex=myvertexs.size();
  num_face=myquads.size();

  //17262
  myio.getVertexAndQuadFromObj(myvertexs_target,myquads_target,name_1);
  num_vertex_target=myvertexs_target.size();
  num_face_target=myquads_target.size();

  
  //myio.getVertexAndHairFromObj(myvertexs_hair,myedges_hair,name_hair);
  num_vertex_hair=myvertexs_hair.size();
  num_edge_hair=myedges_hair.size();
  printf("num_vertex_hair: %d num_edge_hair: %d\n",num_vertex_hair,num_edge_hair);

  myio.saveHairAsVTK(myvertexs_hair,myedges_hair,name_hair_vtk);
}
void object::deform(vector< vertex >& myvertexs_head,double h, vector<pair< myvector, myvector> >& controlPointsPair)
{
  double y_bottom=myvertexs[17262].location.y;
  printf("y_bottom: %lf\n",y_bottom);

  //int num_control=0;
  //int max_num_control=1000;
  for(int i=0;i<num_vertex;i++)
    {
      if(myvertexs[i].location.y>y_bottom)
	{
	  myvertexs_head_index.push_back(i);
	  myvertexs_head.push_back(myvertexs[i]);

	  controlPointsPair.push_back(make_pair(myvertexs[i].location,myvertexs_target[i].location));
	}
    }

  num_vertex_head=myvertexs_head.size();
  printf("num_vertex_head: %d\n",num_vertex_head);

  
  for(int i=0;i<num_vertex_hair;i++)
    {
      if(myvertexs_hair[i].location.y>y_bottom)
	{
	  myvertexs_head_index.push_back(i);
	  myvertexs_head.push_back(myvertexs_hair[i]);
	}
    }
  num_vertex_hair_head=myvertexs_head.size();
  printf("num_vertex_hair_head: %d\n",num_vertex_hair_head);
  
  
  double max_min_xyz[3][2];
  double len[3];
  int div[3];
  max_min_xyz[0][0]=max_min_xyz[0][1]=myvertexs_head[0].location.x;
  max_min_xyz[1][0]=max_min_xyz[1][1]=myvertexs_head[0].location.y;
  max_min_xyz[2][0]=max_min_xyz[2][1]=myvertexs_head[0].location.z;
  
  int num_vertex_head_here=myvertexs_head.size();
  int i,j,k;
  for(i=0;i<num_vertex_head_here;i++)
    {
      vertex vertex_now=myvertexs_head[i];
      if(vertex_now.location.x>max_min_xyz[0][0])
	{
	  max_min_xyz[0][0]=vertex_now.location.x;
	}
      else if(vertex_now.location.x<max_min_xyz[0][1])
	{
	  max_min_xyz[0][1]=vertex_now.location.x;
	}
      if(vertex_now.location.y>max_min_xyz[1][0])
	{
	  max_min_xyz[1][0]=vertex_now.location.y;
	}
      else if(vertex_now.location.y<max_min_xyz[1][1])
	{
	  max_min_xyz[1][1]=vertex_now.location.y;
	}
      if(vertex_now.location.z>max_min_xyz[2][0])
	{
	  max_min_xyz[2][0]=vertex_now.location.z;
	}
      else if(vertex_now.location.z<max_min_xyz[2][1])
	{
	  max_min_xyz[2][1]=vertex_now.location.z;
	}
    }

  double eps=1e-3;
  for(i=0;i<3;i++)
    {
      max_min_xyz[i][0]+=eps;
      max_min_xyz[i][1]-=eps;
    }
  for(i=0;i<3;i++)
    {
      len[i]=max_min_xyz[i][0]-max_min_xyz[i][1];
      div[i]=floor(len[i]/h)+1;
      printf("max_min_xyz[%d] %lf %lf\n",i,max_min_xyz[i][0],max_min_xyz[i][1]);
    }

  printf("div %d %d %d \n",div[0],div[1],div[2]);

  int div_vertex[3];
  for(i=0;i<3;i++)
    {
      div_vertex[i]=div[i]+1;
    }

  int*** index_vertex_grid=(int***) new int**[div_vertex[0]];
  for(i=0;i<div_vertex[0];i++)
    {
      index_vertex_grid[i]=(int**) new int*[div_vertex[1]];
      for(j=0;j<div_vertex[1];j++)
	{
	  index_vertex_grid[i][j]=new int[div_vertex[2]];
	}
    }
  int index_now=0;
  vector< vertex_grid > myvertexsOfGrid;
  myvector location_now;
  for(i=0;i<div_vertex[0];i++)
    {
      for(j=0;j<div_vertex[1];j++)
	{
	  for(k=0;k<div_vertex[2];k++)
	    {
	      location_now=myvector(max_min_xyz[0][1]+i*h,max_min_xyz[1][1]+j*h,max_min_xyz[2][1]+k*h);
	      myvertexsOfGrid.push_back(vertex_grid(location_now));
	      index_vertex_grid[i][j][k]=index_now;
	      index_now++;
	    }
	}
    }
  int num_vertexOfGrid=myvertexsOfGrid.size();

  printf("%d num_vertexOfGrid\n",num_vertexOfGrid);
  grid*** mygrid=(grid***) new grid**[div[0]];
  
  for(i=0;i<div[0];i++)
    {
      mygrid[i]=(grid**) new grid*[div[1]];
      for(j=0;j<div[1];j++)
	{
	  mygrid[i][j]=new grid[div[2]];
	}
    }
  
  int a,b,c,d,e;
  int kind=0;
  int kind_out=0;
  int kind_out_out=0;
  for(i=0;i<div[0];i++)
    {
      kind_out=kind_out_out;
      for(j=0;j<div[1];j++)
	{
	  kind=kind_out;
	  for(k=0;k<div[2];k++)
	    {
	      mygrid[i][j][k].kind=kind;
	      for(a=0;a<2;a++)
		{
		  for(b=0;b<2;b++)
		    {
		      for(c=0;c<2;c++)
			{
			  mygrid[i][j][k].index_vertex[a][b][c]=index_vertex_grid[i+a][j+b][k+c];   
			}
		    }
		}
	      kind=!kind;
	    }
	  kind_out=!kind_out;
	}
      kind_out_out=!kind_out_out;
    }
  int xx,yy,zz;
  for(i=0;i<div[0];i++)
    {
      for(j=0;j<div[1];j++)
	{
	  for(k=0;k<div[2];k++)
	    {
	      int kind_now = mygrid[i][j][k].kind;
	      for(a=0;a<5;a++)
		{
		  for(b=0;b<4;b++)
		    {
		      xx=vertex_collection[kind_now][a][b][0];
		      yy=vertex_collection[kind_now][a][b][1];
		      zz=vertex_collection[kind_now][a][b][2];
		      mygrid[i][j][k].mytet[a].index_vertex[b]=mygrid[i][j][k].index_vertex[xx][yy][zz];
		      
		    }
		}
	    }
	}
    }


  {
    FILE* fp=fopen("./res_now/mytet_ori.vtk","w");
    fprintf(fp,"# vtk DataFile Version 2.0\n");
    fprintf(fp,"tetra\n");
    fprintf(fp,"ASCII\n");
    fprintf(fp,"DATASET UNSTRUCTURED_GRID\n");
    fprintf(fp,"POINTS %d double\n",num_vertexOfGrid);
    for(i=0;i<num_vertexOfGrid;i++)
      {
	fprintf(fp,"%lf %lf %lf\n",myvertexsOfGrid[i].location.x,myvertexsOfGrid[i].location.y,myvertexsOfGrid[i].location.z);
      }

    int num_grid=div[0]*div[1]*div[2];
    fprintf(fp,"CELLS %d %d\n",5*num_grid,25*num_grid);

    for(i=0;i<div[0];i++)
      {
	for(j=0;j<div[1];j++)
	  {
	    for(k=0;k<div[2];k++)
	      {
		for(a=0;a<5;a++)
		  {
		    fprintf(fp,"4");
		    for(b=0;b<4;b++)
		      {
			fprintf(fp," %d",mygrid[i][j][k].mytet[a].index_vertex[b]);
		      }
		    fprintf(fp,"\n");
		  }
	      }
	  }
      }

    fprintf(fp,"CELL_TYPES %d\n",5*num_grid);
    for(i=0;i<5*num_grid;i++)
      {
	fprintf(fp,"10\n");
      }
    fclose(fp);
  }

  
  int which_grid[3];
  for(i=0;i<num_vertex_head_here;i++)
    {
      location_now=myvertexs_head[i].location;
      which_grid[0]=floor((location_now.x-max_min_xyz[0][1])/h);
      which_grid[1]=floor((location_now.y-max_min_xyz[1][1])/h);
      which_grid[2]=floor((location_now.z-max_min_xyz[2][1])/h);

      tetrahedron tet_now;
      MatrixXd matrix_test=MatrixXd::Random(4,4);
      int mark=-2;
      int mark_now=-2;
      
      bool find;
      
      for(j=0;j<5;j++)
	{
	  find = true;
	  tet_now=mygrid[which_grid[0]][which_grid[1]][which_grid[2]].mytet[j];
	  
	  for(a=0;a<4;a++)
	    {
	      matrix_test(a,0)=myvertexsOfGrid[tet_now.index_vertex[a]].location.x;
	      matrix_test(a,1)=myvertexsOfGrid[tet_now.index_vertex[a]].location.y;
	      matrix_test(a,2)=myvertexsOfGrid[tet_now.index_vertex[a]].location.z;
	      matrix_test(a,3)=1;
	    }
	  mark=(matrix_test.determinant()>0?1:-1);

	  for(a=0;a<4;a++)
	    {
	      matrix_test(a,0)=location_now.x;
	      matrix_test(a,1)=location_now.y;
	      matrix_test(a,2)=location_now.z;

	      double det=matrix_test.determinant();

	      matrix_test(a,0)=myvertexsOfGrid[tet_now.index_vertex[a]].location.x;
	      matrix_test(a,1)=myvertexsOfGrid[tet_now.index_vertex[a]].location.y;
	      matrix_test(a,2)=myvertexsOfGrid[tet_now.index_vertex[a]].location.z;
	       
	      if(abs(det) <= 1e-15)
		{
		  continue;
		}
	      mark_now=(det>0?1:-1);
	      if(mark==mark_now)
		{
		  continue;
		}
	      else
		{
		  find=false;
		  break;
		}
	    }
	  if(find==true)
	    {
	      break;
	    }
	}
      myvertexs_head[i].index_tet[0]=which_grid[0];
      myvertexs_head[i].index_tet[1]=which_grid[1];
      myvertexs_head[i].index_tet[2]=which_grid[2];
      if(find==true)
	{
	  myvertexs_head[i].index_tet[3]=j;
	}
    }

  
  //deform vertexOfGrid
  
  {
    for(i=0;i<num_vertexOfGrid;i++)
      {
	myvertexsOfGrid[i].location_deform=myvertexsOfGrid[i].location;
      }

    int index_vertex[4];
    myvector loc_need[3];
    MatrixXd m_ori=MatrixXd::Random(3,3);
    for(i=0;i<div[0];i++)
      {
	for(j=0;j<div[1];j++)
	  {
	    for(k=0;k<div[2];k++)
	      {
		for(a=0;a<5;a++)
		  {
		    for(b=0;b<4;b++)
		      {
			index_vertex[b]=mygrid[i][j][k].mytet[a].index_vertex[b];
		      }
		    for(b=1;b<4;b++)
		      {
			loc_need[b-1]=myvertexsOfGrid[index_vertex[b]].location-myvertexsOfGrid[index_vertex[0]].location;		      
		      }
		    for(b=0;b<3;b++)
		      {
			m_ori(0,b)=loc_need[b].x; m_ori(1,b)=loc_need[b].y; m_ori(2,b)=loc_need[b].z;
		      }
		    mygrid[i][j][k].mytet[a].vol=abs(m_ori.determinant())*0.1667;
		    mygrid[i][j][k].mytet[a].X_inverse=m_ori.inverse();
		  }
	      }
	  }
      }
    
    //反复被计算
    MatrixXd gradient_local=MatrixXd::Random(12,1);
    MatrixXd X=MatrixXd::Random(12,1); 
    MatrixXd B=MatrixXd::Random(9,1); 
    MatrixXd rot=MatrixXd::Random(3,3); 
    MatrixXd def=MatrixXd::Random(3,3); 

    MatrixXd X_all=MatrixXd::Random(num_vertexOfGrid*3,1);

    
    MatrixXd M_gra_global(num_vertexOfGrid*3,1);
    VectorXd deltaX(num_vertexOfGrid*3);
    VectorXd gradient_global(num_vertexOfGrid*3);

    myvector loc_def_need[3];
    MatrixXd m_def=MatrixXd::Random(3,3);

    //只被算一次
    MatrixXd hessian_local=MatrixXd::Random(12,12);
    MatrixXd A=MatrixXd::Random(9,12); 
    MatrixXd hesM_global=MatrixXd::Random(num_vertexOfGrid*3,num_vertexOfGrid*3);


    /*
    MatrixXd selM=MatrixXd::Random(num_vertexOfGrid*3,num_vertexOfGrid*3);
    MatrixXd PX=MatrixXd::Random(num_vertexOfGrid*3,1);
    double w_sel=3;
    MatrixXd posFinal=MatrixXd::Random(num_vertexOfGrid*3,1);
    */

    MatrixXd selM=MatrixXd::Random(num_vertex_head*3,num_vertexOfGrid*3);
    MatrixXd PX=MatrixXd::Random(num_vertex_head*3,1);
    double w_sel=10;
    MatrixXd posFinal=MatrixXd::Random(num_vertex_head*3,1);

    
    SparseMatrix < double > hessian_global(num_vertexOfGrid*3,num_vertexOfGrid*3);
    //SparseLU<SparseMatrix<double>,COLAMDOrdering<int> > linearSolver;//使用LU分解不要用QR分解

    //SimplicialLLT<SparseMatrix<double>,COLAMDOrdering<int> > linearSolver;
    SimplicialLLT<SparseMatrix<double> > linearSolver;
    vector< Triplet< double > > tripletsForHessian_global;

    //计算　selM & posFinal
    selM.fill(0);
    posFinal.fill(0);
    int siz_Pair=controlPointsPair.size();

    myvector pos_begin,pos_end;
    int index_tet[4];
    MatrixXd X_here=MatrixXd::Random(3,3);
    MatrixXd X_inverse_here=MatrixXd::Random(3,3);
    MatrixXd M_pos=MatrixXd::Random(3,1);
    MatrixXd bcd=MatrixXd::Random(3,1);
    MatrixXd factor=MatrixXd::Random(4,1);
    for(i=0;i<siz_Pair;i++)
      {
	pos_begin=controlPointsPair[i].first;
	pos_end=controlPointsPair[i].second;

	which_grid[0]=floor((pos_begin.x-max_min_xyz[0][1])/h);
	which_grid[1]=floor((pos_begin.y-max_min_xyz[1][1])/h);
	which_grid[2]=floor((pos_begin.z-max_min_xyz[2][1])/h);

	tetrahedron tet_now;
	MatrixXd matrix_test=MatrixXd::Random(4,4);
	int mark=-2;
	int mark_now=-2;
      
	bool find;
      
	for(j=0;j<5;j++)
	  {
	    find = true;
	    tet_now=mygrid[which_grid[0]][which_grid[1]][which_grid[2]].mytet[j];
	  
	    for(a=0;a<4;a++)
	      {
		matrix_test(a,0)=myvertexsOfGrid[tet_now.index_vertex[a]].location.x;
		matrix_test(a,1)=myvertexsOfGrid[tet_now.index_vertex[a]].location.y;
		matrix_test(a,2)=myvertexsOfGrid[tet_now.index_vertex[a]].location.z;
		matrix_test(a,3)=1;
	      }
	    mark=(matrix_test.determinant()>0?1:-1);

	    for(a=0;a<4;a++)
	      {
		matrix_test(a,0)=pos_begin.x;
		matrix_test(a,1)=pos_begin.y;
		matrix_test(a,2)=pos_begin.z;

		double det=matrix_test.determinant();

		matrix_test(a,0)=myvertexsOfGrid[tet_now.index_vertex[a]].location.x;
		matrix_test(a,1)=myvertexsOfGrid[tet_now.index_vertex[a]].location.y;
		matrix_test(a,2)=myvertexsOfGrid[tet_now.index_vertex[a]].location.z;
	       
		if(abs(det) <= 1e-15)
		  {
		    continue;
		  }
		mark_now=(det>0?1:-1);
		if(mark==mark_now)
		  {
		    continue;
		  }
		else
		  {
		    find=false;
		    break;
		  }
	      }
	    if(find==true)
	      {
		break;
	      }
	  }
	index_tet[0]=which_grid[0];
	index_tet[1]=which_grid[1];
	index_tet[2]=which_grid[2];
	if(find==true)
	  {
	    index_tet[3]=j;
	  }
	for(j=0;j<4;j++)
	  {
	    index_vertex[j]=mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].index_vertex[j];
	   
	  }
	for(j=0;j<3;j++)
	  {
	    X_here(0,j)=myvertexsOfGrid[index_vertex[j+1]].location.x-myvertexsOfGrid[index_vertex[0]].location.x;
	    X_here(1,j)=myvertexsOfGrid[index_vertex[j+1]].location.y-myvertexsOfGrid[index_vertex[0]].location.y;
	    X_here(2,j)=myvertexsOfGrid[index_vertex[j+1]].location.z-myvertexsOfGrid[index_vertex[0]].location.z;
	  }
	X_inverse_here=X_here.inverse();
	M_pos(0,0)=pos_begin.x-myvertexsOfGrid[index_vertex[0]].location.x;
	M_pos(1,0)=pos_begin.y-myvertexsOfGrid[index_vertex[0]].location.y;
	M_pos(2,0)=pos_begin.z-myvertexsOfGrid[index_vertex[0]].location.z;
	
	bcd=X_inverse_here*M_pos;

	factor(0,0)=1-bcd(0,0)-bcd(1,0)-bcd(2,0);
	factor(1,0)=bcd(0,0); factor(2,0)=bcd(1,0); factor(3,0)=bcd(2,0);

	for(a=0;a<3;a++)
	  {
	    for(b=0;b<4;b++)
	      {
		selM(i*3+a,index_vertex[b]*3+a)=factor(b,0);
	      }
	  }
	posFinal(i*3,0)=pos_end.x; posFinal(i*3+1,0)=pos_end.y; posFinal(i*3+2,0)=pos_end.z;
      }
    // hessian matrix 是常量　因此我们提出来先做，不放入迭代过程

    printf("cal hessian \n");
    hesM_global.fill(0);
    for(i=0;i<div[0];i++)
      {
	for(j=0;j<div[1];j++)
	  {
	    for(k=0;k<div[2];k++)
	      {
		for(a=0;a<5;a++)
		  {
		    A.fill(0);
		    for(b=0;b<4;b++)
		      {
			index_vertex[b]=mygrid[i][j][k].mytet[a].index_vertex[b];
		      }
		    for(b=0;b<3;b++)
		      {
			for(c=0;c<3;c++)
			  {
			    A(b*3+c,3+b)=mygrid[i][j][k].mytet[a].X_inverse(0,c);
			    A(b*3+c,6+b)=mygrid[i][j][k].mytet[a].X_inverse(1,c);
			    A(b*3+c,9+b)=mygrid[i][j][k].mytet[a].X_inverse(2,c);
			    A(b*3+c,0+b)=-1*(mygrid[i][j][k].mytet[a].X_inverse(0,c)+mygrid[i][j][k].mytet[a].X_inverse(1,c)+mygrid[i][j][k].mytet[a].X_inverse(2,c));
			  }
		      }
		    mygrid[i][j][k].mytet[a].A=A;
		    hessian_local=mygrid[i][j][k].mytet[a].vol*A.transpose()*A;

		    for(b=0;b<4;b++)
		      {
			for(c=0;c<3;c++)
			  {
			    for(d=0;d<4;d++)
			      {
				for(e=0;e<3;e++)
				  {
				    hesM_global(index_vertex[b]*3+c,index_vertex[d]*3+e)+=hessian_local(b*3+c,d*3+e);
				  }
			      }
			  }
		      }
		  }
	      }
	  }
      }

    printf("control point constraint begin\n");
    hesM_global+=w_sel*selM.transpose()*selM;
    printf("control point constraint end\n");

    int a=num_vertexOfGrid*3;
    for(i=0;i<a;i++)
      {
	for(j=0;j<a;j++)
	  {
	    if(abs(hesM_global(i,j))>=0.00000000000001)
	      {
		tripletsForHessian_global.emplace_back(i,j,hesM_global(i,j));
	      }
	  }
      }
    
    hessian_global.setFromTriplets(tripletsForHessian_global.begin(),tripletsForHessian_global.end());
    hessian_global.makeCompressed();
    printf("linearSolver start\n");
    linearSolver.compute(hessian_global);
    printf("linearSolver end\n");

    //终止迭代的一般是设置deltaX变化不大时终止，而不设置成多靠近目标位置时结束，因为收敛程度可能并不能达到

    int iteration=0;
    double energy=0;
    while(1)
      {
	iteration++;
	M_gra_global.fill(0);
	energy=0;
	for(i=0;i<div[0];i++)
	  {
	    for(j=0;j<div[1];j++)
	      {
		for(k=0;k<div[2];k++)
		  {
		    for(a=0;a<5;a++)
		      {
			for(b=0;b<4;b++)
			  {
			    index_vertex[b]=mygrid[i][j][k].mytet[a].index_vertex[b];
			  }
			for(b=1;b<4;b++)
			  {
			    loc_def_need[b-1]=myvertexsOfGrid[index_vertex[b]].location_deform-myvertexsOfGrid[index_vertex[0]].location_deform; 
			  }
			for(b=0;b<3;b++)
			  {
			    m_def(0,b)=loc_def_need[b].x; m_def(1,b)=loc_def_need[b].y; m_def(2,b)=loc_def_need[b].z;
			  }
		      
			def=m_def*mygrid[i][j][k].mytet[a].X_inverse;
			JacobiSVD<MatrixXd> svd(def, ComputeThinU | ComputeThinV);
			rot=svd.matrixU()*(svd.matrixV().transpose());

	      		double energy_here=0;

			for(b=0;b<3;b++)
			  {
			    for(c=0;c<3;c++)
			      {
				energy_here+=pow((def(b,c)-rot(b,c)),2);
			      }
			  }
			energy_here=sqrt(energy_here);
			energy_here*=mygrid[i][j][k].mytet[a].vol;

			energy+=energy_here;
			for(b=0;b<3;b++)
			  {
			    for(c=0;c<3;c++)
			      {
				B(b*3+c,0)=rot(b,c);
			      }
			  }
		      
			for(b=0;b<4;b++)
			  {
			    X(b*3+0,0)=myvertexsOfGrid[index_vertex[b]].location_deform.x;
			    X(b*3+1,0)=myvertexsOfGrid[index_vertex[b]].location_deform.y;
			    X(b*3+2,0)=myvertexsOfGrid[index_vertex[b]].location_deform.z;
			  }
			gradient_local=mygrid[i][j][k].mytet[a].vol*mygrid[i][j][k].mytet[a].A.transpose()*(mygrid[i][j][k].mytet[a].A*X-B);
		      
			for(b=0;b<4;b++)
			  {
			    for(c=0;c<3;c++)
			      {
				M_gra_global(index_vertex[b]*3+c,0)+=gradient_local(b*3+c,0);
			      }
			  }
		     
		      }
		  }
	      }
	  }
	printf("energy part 1:%lf \n",energy);
	for(i=0;i<num_vertexOfGrid;i++)
	  {
	    X_all(i*3,0)=myvertexsOfGrid[i].location_deform.x;
	    X_all(i*3+1,0)=myvertexsOfGrid[i].location_deform.y;
	    X_all(i*3+2,0)=myvertexsOfGrid[i].location_deform.z;
	  }
	PX=selM*X_all;
	M_gra_global+=(w_sel*selM.transpose()*(PX-posFinal));

	int a=num_vertexOfGrid*3;	
	double energy_here=0;
	for(i=0;i<a;i++)
	  {
	    energy_here+=pow(PX(i,0)-posFinal(i,0),2);
	  }
	energy_here=w_sel*sqrt(energy_here);
	printf("energy part 2:%lf \n",energy_here);
	energy+=energy_here;
	
        for(i=0;i<a;i++)
	  {
	    gradient_global(i)=-1*M_gra_global(i,0);   
	  }

	deltaX=linearSolver.solve(gradient_global);	

	//check
	double sum=0;
	for(i=0;i<num_vertexOfGrid;i++)
	  {
	    sum+=sqrt(deltaX(i*3)*deltaX(i*3)+deltaX(i*3+1)*deltaX(i*3+1)+deltaX(i*3+2)*deltaX(i*3+2))/sqrt(X_all(i*3,0)*X_all(i*3,0)+X_all(i*3+1,0)*X_all(i*3+1,0)+X_all(i*3+2,0)*X_all(i*3+2,0));
	  }
	printf("energy %lf\n",energy);
	printf("sum %lf\n",sum);
	if(sum<0.01)
	  {
	    break;
	  }
	else
	  {
	    for(i=0;i<num_vertexOfGrid;i++)
	      {
		myvertexsOfGrid[i].location_deform.x+=deltaX(i*3);
		myvertexsOfGrid[i].location_deform.y+=deltaX(i*3+1);
		myvertexsOfGrid[i].location_deform.z+=deltaX(i*3+2);
	      }
	  }
      }

  }
  printf("ok\n");
  
  {
    FILE* fp=fopen("./res_now/mytet_deform.vtk","w");
    fprintf(fp,"# vtk DataFile Version 2.0\n");
    fprintf(fp,"tetra\n");
    fprintf(fp,"ASCII\n");
    fprintf(fp,"DATASET UNSTRUCTURED_GRID\n");
    fprintf(fp,"POINTS %d double\n",num_vertexOfGrid);
    for(i=0;i<num_vertexOfGrid;i++)
      {
	fprintf(fp,"%lf %lf %lf\n",myvertexsOfGrid[i].location_deform.x,myvertexsOfGrid[i].location_deform.y,myvertexsOfGrid[i].location_deform.z);
      }

    int num_grid=div[0]*div[1]*div[2];
    fprintf(fp,"CELLS %d %d\n",5*num_grid,25*num_grid);

    for(i=0;i<div[0];i++)
      {
	for(j=0;j<div[1];j++)
	  {
	    for(k=0;k<div[2];k++)
	      {
		for(a=0;a<5;a++)
		  {
		    fprintf(fp,"4");
		    for(b=0;b<4;b++)
		      {
			fprintf(fp," %d",mygrid[i][j][k].mytet[a].index_vertex[b]);
		      }
		    fprintf(fp,"\n");
		  }
	      }
	  }
      }

    fprintf(fp,"CELL_TYPES %d\n",5*num_grid);
    for(i=0;i<5*num_grid;i++)
      {
	fprintf(fp,"10\n");
      }
    fclose(fp);
  }

  printf("tet\n");

  int index_tet[4];
  int index_vertex[4];
      
  myvector loc_deform_need[3];
  MatrixXd m_def=MatrixXd::Random(3,3);

  MatrixXd loc_ori=MatrixXd::Random(3,1);
  MatrixXd loc_def=MatrixXd::Random(3,1);
  for(i=0;i<num_vertex_head_here;i++)
    {
      for(j=0;j<4;j++)
	{
	  index_tet[j]=myvertexs_head[i].index_tet[j];
	}
      //这层循环要写在这里，写在if里面则逻辑错误
      for(j=0;j<4;j++)
	{
	  index_vertex[j]=mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].index_vertex[j];
	}
      if(mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].get_P==0)
	{ 
	  for(j=1;j<4;j++)
	    { 
	      loc_deform_need[j-1]=myvertexsOfGrid[index_vertex[j]].location_deform-myvertexsOfGrid[index_vertex[0]].location_deform;
	    }
	  for(j=0;j<3;j++)
	    {
	      m_def(0,j)=loc_deform_need[j].x; m_def(1,j)=loc_deform_need[j].y; m_def(2,j)=loc_deform_need[j].z;
	    }
	  mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].P=m_def*mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].X_inverse;

	  mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].get_P=1;
	}
      loc_ori(0,0)=myvertexs_head[i].location.x-myvertexsOfGrid[index_vertex[0]].location.x;
      loc_ori(1,0)=myvertexs_head[i].location.y-myvertexsOfGrid[index_vertex[0]].location.y;
      loc_ori(2,0)=myvertexs_head[i].location.z-myvertexsOfGrid[index_vertex[0]].location.z;

      //cout<<mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].P<<endl;
      loc_def=mygrid[index_tet[0]][index_tet[1]][index_tet[2]].mytet[index_tet[3]].P*loc_ori;

      myvertexs_head[i].location_deform.x=loc_def(0,0)+myvertexsOfGrid[index_vertex[0]].location_deform.x;
      myvertexs_head[i].location_deform.y=loc_def(1,0)+myvertexsOfGrid[index_vertex[0]].location_deform.y;
      myvertexs_head[i].location_deform.z=loc_def(2,0)+myvertexsOfGrid[index_vertex[0]].location_deform.z;
    }

  printf("tet end\n");
}


