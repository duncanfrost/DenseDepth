// TGA Loader - 16/11/04 Codehead
#include "glTools.h"
#include <iostream>
#include <iomanip>


void multiplyGlMatrix(Eigen::Matrix4f _mat)
{
	GLfloat m[16];
	for(int i=0;i<4;i++)
		for(int j=0;j<4;j++)m[i+4*j]=_mat(i,j);
	glMultMatrixf(m);
}

void unset3DGLProjection()
{
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}


void set2DGLProjection()
{
	glDisable (GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	GLint viewport_ref[4];
	glGetIntegerv( GL_VIEWPORT, viewport_ref );
	glOrtho(0.0, viewport_ref[2],
		viewport_ref[3],0.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
}
void unset2DGLProjection()
{
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glEnable (GL_DEPTH_TEST);
}
cv::Mat Capture_Image()
{
	GLvoid* pixels;
	GLint viewport[4];
	glGetIntegerv(GL_VIEWPORT,viewport);

	int _width=viewport[2];int _height=viewport[3];
	pixels = (GLvoid *) malloc(_width * _height * 3);
	//glReadBuffer(GL_FRONT);
	glReadBuffer(GL_BACK);
	glReadPixels(0, 0, _width, _height, GL_RGB, GL_UNSIGNED_BYTE, pixels);
	unsigned char *pt_buf;
	pt_buf=(unsigned char*)pixels;


	cv::Mat frame;frame.create(_height, _width, CV_8UC3);
	for (int i=0; i<_width; ++i)
	      for (int j=0; j<_height; ++j)
	      {
		      frame.at<cv::Vec3b>(_height-1-j,i)[0]=pt_buf[(i+j*_width)*3+2];
		      frame.at<cv::Vec3b>(_height-1-j,i)[1]=pt_buf[(i+j*_width)*3+1];
		      frame.at<cv::Vec3b>(_height-1-j,i)[2]=pt_buf[(i+j*_width)*3+0];
	      }
	free(pixels);

	return frame;
}
cv::Mat Capture_ZBuffer()
{

	//save image
	 GLvoid* pixels;
	GLint viewport[4];     glGetIntegerv( GL_VIEWPORT, viewport );
	int _width=viewport[2];int _height=viewport[3];
	pixels = (GLvoid *) malloc(sizeof(GLfloat)*_width * _height);
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, _width, _height, GL_DEPTH_COMPONENT, GL_FLOAT, pixels);
	float *ptz_buf;
	ptz_buf=(float*)pixels;

	cv::Mat frame;frame.create(_height, _width, CV_32FC1);
	for (int i=0; i<_width; ++i)
	      for (int j=0; j<_height; ++j)
		      frame.at<float>(_height-1-j,i)=ptz_buf[(i+j*_width)];

	free(pixels);

	return frame;
}
cv::Mat Capture_realZ()
{

	//save image
	 GLvoid* pixels;
	GLint viewport[4];     glGetIntegerv( GL_VIEWPORT, viewport );
	int _width=viewport[2];int _height=viewport[3];
	pixels = (GLvoid *) malloc(sizeof(GLfloat)*_width * _height);
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, _width, _height, GL_DEPTH_COMPONENT, GL_FLOAT, pixels);
	float *ptz_buf;
	ptz_buf=(float*)pixels;

	GLdouble gldepth_rqange[2];
	glGetDoublev(GL_DEPTH_RANGE, gldepth_rqange);
	GLdouble tProjectionM[16];
	glGetDoublev(GL_PROJECTION_MATRIX, tProjectionM);

	cv::Mat frame;frame.create(_height, _width, CV_32FC1);
	for (int i=0; i<_width; ++i)
	      for (int j=0; j<_height; ++j)
	      {
		      float _z=ptz_buf[(i+j*_width)];
		      if(_z==1)
			      frame.at<float>(_height-1-j,i)= -1;
		      else
		      {
				double zndc=2.*(_z-0.5*gldepth_rqange[1]-0.5*gldepth_rqange[0])/(gldepth_rqange[1]-gldepth_rqange[0]);
				frame.at<float>(_height-1-j,i)=tProjectionM[14]/(zndc-tProjectionM[11]);
		      }

	      }

	free(pixels);

	return frame;
}
cv::Mat Capture_displayable_ZBuffer()
{

	//save image
	 GLvoid* pixels;
	GLint viewport[4];     glGetIntegerv( GL_VIEWPORT, viewport );
	int _width=viewport[2];int _height=viewport[3];
	pixels = (GLvoid *) malloc(sizeof(GLfloat)*_width * _height);
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, _width, _height, GL_DEPTH_COMPONENT, GL_FLOAT, pixels);
	float *ptz_buf;
	ptz_buf=(float*)pixels;

	//CVD::Image<CVD::byte> img(CVD::ImageRef(_width,_height));
	cv::Mat img;img.create(_height, _width, CV_8UC1);

	float min_z=ptz_buf[0];
	float max_z=ptz_buf[0];
	for (int i=0; i<_width; ++i)
	      for (int j=0; j<_height; ++j)
	      {
		       float val=ptz_buf[(i+j*_width)];
		       if(val<min_z)min_z=val;
		       if(val>max_z)max_z=val;
	      }

	for (int i=0; i<_width; ++i)
	      for (int j=0; j<_height; ++j)
	      {
		       unsigned char val=(unsigned char)(255.*(ptz_buf[(i+j*_width)]-min_z)/(max_z-min_z));
		       img.at<uchar>(_height-1-j,i)=val;
	      }
	free(pixels);

	return img;
}
float zbuffer_to_zreal(float _z)
{
	if(_z==1)return -1;//background

	GLdouble gldepth_rqange[2];
	glGetDoublev(GL_DEPTH_RANGE, gldepth_rqange);
	GLdouble tProjectionM[16];
	glGetDoublev(GL_PROJECTION_MATRIX, tProjectionM);
	double zndc=2.*(_z-0.5*gldepth_rqange[1]-0.5*gldepth_rqange[0])/(gldepth_rqange[1]-gldepth_rqange[0]);
	//return proj_mat[2][3]/(zndc-proj_mat[2][2]);
	return tProjectionM[14]/(zndc-tProjectionM[11]);
}
float zreal_to_zbuffer(float _z)
{
	GLdouble gldepth_rqange[2];
	glGetDoublev(GL_DEPTH_RANGE, gldepth_rqange);
	GLdouble tProjectionM[16];
	glGetDoublev(GL_PROJECTION_MATRIX, tProjectionM);
	//return 0.5*(gldepth_rqange[1]-gldepth_rqange[0])*(proj_mat[2][2]+proj_mat[2][3]/_z)+0.5*gldepth_rqange[1]+0.5*gldepth_rqange[0];
	return 0.5*(gldepth_rqange[1]-gldepth_rqange[0])*(tProjectionM[14]+tProjectionM[11]/_z)+0.5*gldepth_rqange[1]+0.5*gldepth_rqange[0];
}
#include <fstream>
void floatImageSave(const char* szFilename, cv::Mat &img)
{
	if(img.type()==CV_32FC1)
	{
		std::ofstream fout;
		fout.open(szFilename);

		int _w=img.size().width;
		int _h=img.size().height;
		fout.write((const char*)&_w,sizeof(int));
		fout.write((const char*)&_h,sizeof(int));
		fout.write((const char*)(&img.at<float>(0,0)),img.size().width*img.size().height*sizeof(float));
		fout.close();
	}
}
bool floatImageRead(const char* szFilename, cv::Mat &img)
{
	std::ifstream fout;
	fout.open(szFilename);
	if(!fout.is_open())
	{
		std::cerr<<"floatImageRead : problem loading the float image "<<szFilename<<std::endl;
		return false;
	}

	int width,height;
	fout.read((char*)&width,sizeof(int));
	fout.read((char*)&height,sizeof(int));
	img.create(height, width, CV_32FC1);
	fout.read((char*)&img.at<float>(0,0),width*height*sizeof(float));
	fout.close();
	return true;
}

void getDisplayableImage(cv::Mat &img,cv::Mat &img_disp,float ignored_value)
{
	bool init=false;
	float min_z=img.at<float>(0,0);
	float max_z=min_z;

	for (int i=0; i<img.size().width; ++i)
	for (int j=0; j<img.size().height; ++j)
	{
		float val=img.at<float>(j,i);
		if(val!=ignored_value)
		{
		if(!init)
		{
			init=true;
			max_z=val;
			min_z=val;
		}
		else
		{
			if(val>max_z)max_z=val;
			if(val<min_z)min_z=val;
		}
		}
	}
	img_disp.create(img.size().height, img.size().width, CV_8UC3);
	if(max_z!=min_z)
	for (int i=0; i<img.size().width; ++i)
		for (int j=0; j<img.size().height; ++j)
	{
		float val=img.at<float>(j,i);
		if(val!=ignored_value)
		{
			uchar col=(unsigned char)(255.*(val-min_z)/(max_z-min_z));
			img_disp.at<cv::Vec3b>(j,i)[0]=col;
			img_disp.at<cv::Vec3b>(j,i)[1]=col;
			img_disp.at<cv::Vec3b>(j,i)[2]=col;
		}
		else
		{
			uchar col=0;
			img_disp.at<cv::Vec3b>(j,i)[0]=col;
			img_disp.at<cv::Vec3b>(j,i)[1]=col;
			img_disp.at<cv::Vec3b>(j,i)[2]=col;
		}
	}
	else
	for (int i=0; i<img.size().width; ++i)
		for (int j=0; j<img.size().height; ++j)
	{
		uchar col=0;
		img_disp.at<cv::Vec3b>(j,i)[0]=col;
		img_disp.at<cv::Vec3b>(j,i)[1]=col;
		img_disp.at<cv::Vec3b>(j,i)[2]=col;
	}
}

void drawText(GLint x, GLint y, const char* s, GLfloat r, GLfloat g, GLfloat b,
              void* font)
{
	int lines;
	y+=18;//so that param x,y corresponds to top left of text.
	const char* p;
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	GLint viewport_ref[4];
	glGetIntegerv( GL_VIEWPORT, viewport_ref );

	glOrtho(0.0, viewport_ref[2],
		0.0, viewport_ref[3], -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glColor3f(r,g,b);
	glRasterPos2i(x, viewport_ref[3]-y);
	for(p = s, lines = 0; *p; p++) {
		if (*p == '\n')
		{
			lines++;
			glRasterPos2i(x, viewport_ref[3]-y-(lines*18));
		}
		glutBitmapCharacter(font, *p);
	}
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glColor3f(1.,1.,1.);

}
