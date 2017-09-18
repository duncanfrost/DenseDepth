//Copyright 2008 Isis Innovation Limited
// TGA Loader - 16/11/04 Codehead
#include "glTexture.h"


bool is_pow2(const int x) { return ((x & -x) == x); }

void GetGlBufferFromCV(cv::Mat &Img,unsigned char *&pImage,int &iWidth,
                       int &iHeight,float &w_util,float &h_util)
 {
	using namespace std;
	unsigned long ulSize;

	int uiBPP;

	int iWidth_im=Img.cols;
	int iHeight_im=Img.rows;

	if(!is_pow2(iWidth_im))
	{
	    int wpow=log(iWidth_im)/log(2);
	    iWidth=pow(2,(int)wpow+1);
	     w_util=(float)iWidth_im/iWidth;
	}
	else
	{
	    iWidth=iWidth_im;
	    w_util=1.;
	}

	if(!is_pow2(iHeight_im))
	{
	    int wpow=log(iHeight_im)/log(2);
	    iHeight=pow(2,(int)wpow+1);
	    h_util=(float)iHeight_im/iHeight;
	}
	else
	{
	    iHeight=iHeight_im;
	    h_util=1.;
	}

	uiBPP=4;
	unsigned long nPixels=iWidth*iHeight;
	ulSize=uiBPP*nPixels;
	pImage=new unsigned char[ulSize];

	if(Img.type() == CV_8UC3)
	for (int i=0; i<iWidth_im; ++i)
	      for (int j=0; j<iHeight_im; ++j)
	      {
		      pImage[(i+j*iWidth)*uiBPP+3] = 255;
		      pImage[(i+j*iWidth)*uiBPP+2] = Img.at<cv::Vec3b>(iHeight_im-1-j,i)[0];
		      pImage[(i+j*iWidth)*uiBPP+1] = Img.at<cv::Vec3b>(iHeight_im-1-j,i)[1];
		      pImage[(i+j*iWidth)*uiBPP+0] = Img.at<cv::Vec3b>(iHeight_im-1-j,i)[2];

	      }
	else
 	for (int i=0; i<iWidth_im; ++i)
	      for (int j=0; j<iHeight_im; ++j)
	      {

		      pImage[(i+j*iWidth)*uiBPP+3] = 255;
		      pImage[(i+j*iWidth)*uiBPP+2] = Img.at<unsigned char>(iHeight_im-1-j,i);
		      pImage[(i+j*iWidth)*uiBPP+1] = Img.at<unsigned char>(iHeight_im-1-j,i);
		      pImage[(i+j*iWidth)*uiBPP+0] = Img.at<unsigned char>(iHeight_im-1-j,i);


	      }
}



void LoadTexture(cv::Mat &Img,TextureSet &Textureset)
{

	unsigned char *pImage;
	int iWidth,iHeight;
	float w_util,h_util;
	GetGlBufferFromCV(Img,pImage,iWidth,iHeight,w_util,h_util);

	if(!Textureset.isAllocated)
		glGenTextures(1,&Textureset.Texture);            // Allocate space for texture
	glBindTexture(GL_TEXTURE_2D,Textureset.Texture); // Set our Tex handle as current

	Textureset.Text_wu=w_util;
	Textureset.Text_hu=h_util;

	// Create the texture
	glTexImage2D(GL_TEXTURE_2D,0,4,iWidth,iHeight,0,GL_RGBA,GL_UNSIGNED_BYTE,pImage);

	// Specify filtering and edge actions
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);
	Textureset.isAllocated=true;
	delete[] pImage;
}



void DrawTextureBackground(TextureSet &Tex)
{
	glDisable (GL_DEPTH_TEST);

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

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,Tex.Texture);

	glBegin(GL_POLYGON);

	glTexCoord2f (0, Tex.Text_hu);
	glVertex2f(0.0f,viewport_ref[3]);

	glTexCoord2f (0, 0);
	glVertex2f(0.0f,0.0f);

	glTexCoord2f (Tex.Text_wu, 0);
	glVertex2f(viewport_ref[2],0.0f);

	glTexCoord2f (Tex.Text_wu, Tex.Text_hu);
	glVertex2f(viewport_ref[2],viewport_ref[3]);

	glEnd();
	glDisable(GL_TEXTURE_2D);


	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	//glEnable(GL_LIGHTING);
	glEnable (GL_DEPTH_TEST);
}

void DrawTextureCoords(TextureSet &Tex, unsigned int x1, unsigned int y1,
                       unsigned int x2, unsigned int y2)
{
	glDisable (GL_DEPTH_TEST);

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

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,Tex.Texture);

	glBegin(GL_POLYGON);

	glTexCoord2f (0, Tex.Text_hu);
	glVertex2f(x1,y2);

	glTexCoord2f (0, 0);
	glVertex2f(x1,y1);

	glTexCoord2f (Tex.Text_wu, 0);
	glVertex2f(x2,y1);

	glTexCoord2f (Tex.Text_wu, Tex.Text_hu);
	glVertex2f(x2,y2);

	glEnd();
	glDisable(GL_TEXTURE_2D);


	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	//glEnable(GL_LIGHTING);
	glEnable (GL_DEPTH_TEST);
}