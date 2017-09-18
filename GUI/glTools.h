/*This file defines several functions used to simultaneously use
 * Toon and Opengl, a structure that regroups all the information
 * of the camera (intrinsic and extrinsic params) in Toon and GL
 * style.
 * It also defines a class Camera used by the viewers and a structure
 * to save textures to project them on surfaces.*/

#pragma once

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <iostream>
#include <fstream>
#include <memory.h>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>


void multiplyGlMatrix(Eigen::Matrix4f _mat);

//set projection matrix and modelview of opengl from cam, to use primitives as glVertex3f
void unset3DGLProjection();


//set 2D projection matrix to use things as glVertex2f (top left=0,0)
void set2DGLProjection();
void unset2DGLProjection();

//screenshot functions
cv::Mat Capture_Image();
cv::Mat Capture_ZBuffer();
cv::Mat Capture_realZ();
cv::Mat Capture_displayable_ZBuffer();

float zbuffer_to_zreal(float _z);
float zreal_to_zbuffer(float _z);

void floatImageSave(const char* szFilename, cv::Mat &img);
bool floatImageRead(const char* szFilename, cv::Mat &img);

void getDisplayableImage(cv::Mat &img,cv::Mat &img_disp,float ignored_value=-1e10);

void drawText(GLint x, GLint y, const char* s, GLfloat r, GLfloat g, GLfloat b, void *font);

inline void glTranslate( const Eigen::Vector3f & v )
{
    glTranslated(v(0), v(1), v(2));
}


inline void glMultMatrix( const Eigen::Matrix3f & m )
	{
		GLdouble glm[16];
		glm[0] = m(0,0); glm[1] = m(1,0); glm[2] = m(2,0); glm[3] = 0;
		glm[4] = m(0,1); glm[5] = m(1,1); glm[6] = m(2,1); glm[7] = 0;
		glm[8] = m(0,2); glm[9] = m(1,2); glm[10] = m(2,2); glm[11] = 0;
		glm[12] = 0; glm[13] = 0; glm[14] = 0; glm[15] = 1;
		glMultMatrixd(glm);
	}


inline void glMultMatrix( const Sophus::SO3f &so3 )
{
    glMultMatrix(so3.matrix());
}


inline void glMultMatrix( const Sophus::SE3f &se3 )
{
    glTranslate( se3.translation());
    glMultMatrix(se3.so3());
}


