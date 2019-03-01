/*
 * GL01Hello.cpp: Test OpenGL C/C++ Setup
 * modified from https://www3.ntu.edu.sg/home/ehchua/programming/opengl/HowTo_OpenGL_C.html 
 */

#include <GL/glew.h>
#include <GL/freeglut.h>; 
#include "cyMatrix.h"
#include "cyGL.h"
#include "cloth.h"


#define WINDOW_WIDTH 600
#define WINDOW_HEIGHT 600
#define ROTATE_RATE 0.005f
#define ZOOM_RATE 0.001f
#define CLOTHRES 10
#define CLOTH_WIDTH 1
#define CLOTH_HEIGHT 1

using namespace::cy;
enum PERSMODE {PERSP, ORTHO};
PERSMODE perspectiveMode = PERSP;

float bgcolor[3] = { 0.0f,0.0f,0.0f };
int mainWindow, programID, mouseFirstPressed[2] = { 1,1 };
int mouseState, mouseButton;

float camDist, xAngle, yAngle, t=0;
Point2f mousePos;
GLuint MVPID, VAO, VBO;
Cloth cloth;

/*GLfloat  v_array[9] = {-0.5f, -0.5f, 0,
					0.5f, -0.5f, 0,
					0.5f, 0.5f, 0 };*/
GLfloat v_array[CLOTHRES*CLOTHRES*6*3];

void initGlew() {
	GLenum res = glewInit();
	if (res != GLEW_OK)
	{
		fprintf(stderr, "Error: '%s'\n", glewGetErrorString(res));
		return;
	}
}

void initShader() {
	GLSLProgram prog;
	GLSLShader vshader, fshader;
	vshader.CompileFile("bin/glsl/vshader.vert", GL_VERTEX_SHADER);
	fshader.CompileFile("bin/glsl/fshader.frag", GL_FRAGMENT_SHADER);
	prog.CreateProgram();
	programID = prog.GetID();
	prog.AttachShader(vshader);
	prog.AttachShader(fshader);
	prog.Link();
	prog.Bind();
}

void setUniformMVP() {
	Matrix4f model, trans, rot, view, pers, mvp;
	float MVP[16];


	model.SetIdentity();
	trans.SetTrans(Point3f(0, 0,-camDist));
	rot.SetRotationXYZ(yAngle, xAngle, 0);
	view = trans * rot;

	if (perspectiveMode == PERSP)
		pers.SetPerspective(M_PI / 2.5f, 1.0f, 0.1f, 1000.0f);
	else
		pers.SetOrthogonal(1, -1, 1, -1, 0.1f, 1000.0f, camDist);

	mvp = pers * view *model;

	//modelview.SetIdentity();
	mvp.Get(MVP);
	glUniformMatrix4fv(MVPID, 1, GL_FALSE, MVP);
}

void initViewMatrices() {

	camDist = 3;
	xAngle = 1.579;
	yAngle = -0;

	// init & send modelview
	MVPID = glGetUniformLocation(programID, "MVP");
	setUniformMVP();

}


void init(char* filename) {
	//select clearing (background) color
	glClearColor(0.0, 0.0, 0.0, 0.0);

	initGlew();
	initShader();
	initViewMatrices();

	// create obj
	cloth.init(CLOTHRES, CLOTH_WIDTH, CLOTH_HEIGHT);
	cloth.fill_v_array(v_array);


	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(v_array), v_array, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	// (attribute index(pos=0), num of component(xyz=3), type, normalized?, bytes between two instance, attribute offset)
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);


}

void mouseFunc(int button, int state, int x, int y) {
	mouseButton = button;
	mouseState = state;
	if (state == GLUT_UP) {
		mouseFirstPressed[0] = 1;
		mouseFirstPressed[1] = 1;
	}

}

float distFromWindowCenter(float x, float y) {
	return (x - WINDOW_WIDTH / 2)*(x - WINDOW_WIDTH / 2) + (y - WINDOW_HEIGHT / 2)*(y - WINDOW_HEIGHT / 2);
}

void motionFunc(int x, int y) {

	if (mouseState == GLUT_DOWN && mouseButton == GLUT_LEFT_BUTTON) {
		if (mouseFirstPressed[0]) {
			mousePos = Point2f(x, y);
			mouseFirstPressed[0] = 0;
			return;
		}
		float deltaMousePos_x = x - mousePos.x;
		float deltaMousePos_y = y - mousePos.y;
		//std::cout << deltaMousePos_x << " " << deltaMousePos_y << std::endl;
		mousePos.x = x;
		mousePos.y = y;	
		xAngle += deltaMousePos_x*ROTATE_RATE;
		yAngle += deltaMousePos_y* ROTATE_RATE;
		mouseFirstPressed[0] = 1;
		return;
	}
	if (mouseState == GLUT_DOWN && mouseButton == GLUT_RIGHT_BUTTON) {
		if (mouseFirstPressed[1]) {
			mousePos = Point2f(x, y);
			mouseFirstPressed[1] = 0;
			return;
		}

		if((camDist > 1.0f) &&(camDist <500.0f) )
			camDist += ZOOM_RATE * (distFromWindowCenter(mousePos.x, mousePos.y) - distFromWindowCenter(x, y));

		mousePos.x = x;
		mousePos.y = y;
		mouseFirstPressed[1] = 1;
	}
}

 /* Handler for window-repaint event. Call back when the window first appears and
	whenever the window needs to be re-painted. */
void display() {
	glClearColor(bgcolor[0], bgcolor[1], bgcolor[2], 1.0f); // Set background color to black and opaque
	glClear(GL_COLOR_BUFFER_BIT);         // Clear the color buffer
	glLoadIdentity();
	
	setUniformMVP();
	glDrawArrays(GL_TRIANGLES, 0, 6*CLOTHRES*CLOTHRES);


	glutSwapBuffers();  
}

/* Idle function called between disply. Update time*/

void idle() {

	//t += 0.01;
	//cloth.move(t);
	cloth.computeForces();
	cloth.computeNextState_smp();
	cloth.incrementStep();
	cloth.fill_v_array(v_array);

	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(v_array), v_array, GL_STATIC_DRAW);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);


	glutSetWindow(mainWindow);
	glutPostRedisplay();
}

void processNormalKeys(unsigned char key, int x, int y) {
	// exit with esc key
	if (key == 27)
		exit(0);

	// switch perspective
	if (key == 'p') {
		if (perspectiveMode == ORTHO) perspectiveMode = PERSP;
		else if (perspectiveMode == PERSP) perspectiveMode = ORTHO;
	}
}

void processSpecialKeys(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_F6: 
		initShader();
		break;
	}

}

/* Main function: GLUT runs as a console application starting at main()  */
int main(int argc, char** argv) {
	if (argc < 1) { std::cout << "run with obj file" << std::endl; return 0; }
	glutInit(&argc, argv);                 // Initialize GLUT
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA); // init display mode
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);   // Set the window's initial width & height
	glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
	mainWindow = glutCreateWindow("project 2"); // Create a window with the given title

	// Register display callback handler for window re-paint
	glutDisplayFunc(display); 
	glutMouseFunc(mouseFunc);
	glutMotionFunc(motionFunc);
	glutIdleFunc(idle);

	// Register keyboard input callback handler 
	glutKeyboardFunc(processNormalKeys);
	glutSpecialFunc(processSpecialKeys);

	init(argv[1]);
	glutMainLoop();           // Enter the infinitely event-processing loop

	return 0;
}