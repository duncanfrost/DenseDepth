#include "Button.h"
#include "glTools.h"

Button::Button(std::string caption,
	void(*clickFunction)(), int x, int y, int w, int h)
{
	this->caption = caption;
	this->clickFunction = clickFunction;
	this->x = x; this->y = y;
	this->h = h; this->w = w;

	clicked = false;
	mouseIn = false;
}


bool Button::CheckClick(int state, int x, int y)
{
	mouseIn = (x > this->x && y > this->y &&
		x < this->x + this->w && y < this->y + this->h);
	if (state == GLUT_DOWN)
	{
		if (mouseIn)
		{
			clicked = true;
			return true;
		}

	}
	if (state == GLUT_UP)
	{
		if (mouseIn && clicked)
		{
			(*clickFunction)();
			return true;
		}
		clicked = false;
	}

	return false;
}

void Button::CheckMotion(int x, int y)
{
	mouseIn = (x > this->x && y > this->y &&
		x < this->x + this->w && y < this->y + this->h);
}

void Button::Draw()
{
	set2DGLProjection();

	if (clicked && mouseIn)
		glColor3f(0.8f, 0.5f, 0.0f);
	else
		glColor3f(1.0f, 0.7f, 0.0f);
	glBegin(GL_QUADS);
	glVertex2i(x, y); glVertex2i(x + w, y);
	glVertex2i(x + w, y + h); glVertex2i(x, y + h);
	glEnd();
	glColor3f(0.0f, 0.0f, 0.0f);

	glBegin(GL_LINE_LOOP);
	glVertex2i(x, y); glVertex2i(x + w, y);
	glVertex2i(x + w, y + h); glVertex2i(x, y + h);
	glEnd();

	drawText(x + 7, y + 7, caption.c_str(), 1, 1, 1, GLUT_BITMAP_HELVETICA_18);

	glColor3f(1.0f, 1.0f, 1.0f);
	unset2DGLProjection();
}
