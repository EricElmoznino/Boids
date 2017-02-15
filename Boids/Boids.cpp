/***********************************************************
             CSCD18 - UTSC , Fall 2016 version

                     Boids.cpp

	This assignment will help you become familiar with
	the basic structure and shape of an OpenGL program.

	Please take time to read through the code and note
	how the viewing parameters, viewing volume, and
	general OpenGL options are set. You will need to
	change those in future assignments.

	You should also pay attention to the way basic
	OpenGL drawing commands work and what they do.
        You should check the OpenGL reference manual
        for the different OpenGL functions you will
        find here. In particular, those that set up
        the viewing parameters.

	Note that this program is intended to display
	moving objects in real time. As such, it is
	strongly recommended that you run this code locally,
	on one of the machines at the CS lab. Alternately,
	install the OpenGL libraries on your own computer.

	Working remotely over ssh, or working on a non-
	Linux machine will give you headaches.

    Instructions:

	The assignment handout contains a detailed
	description of what you need to do. Please be
	sure to read it carefully.

	You must complete all sections marked
        // TO DO

	In addition to this, you have to complete
	all information requested in the file called
	REPORT.TXT. Be sure to answer in that
	report any
	// QUESTION:
	parts found in the code below.

	Sections marked
	// CRUNCHY:
	Are worth extra credit. How much bonus you get
	depends on the quality of your extensions
	or enhancements. Be sure to indicate in your
	REPORT.TXT any extra work you have done.

	The code is commented, and the comments provide
	information about what the program is doing and
	what your task will be.

	As a reminder. Your code must compile and run
	on 'mathlab.utsc.utoronto.ca' under Linux. We will
	not grade code that fails to compile or run
	on these machines.

Written by: F. Estrada, Jun 2011.
            Main loop/init derived from older D18/418
            OpenGL assignments
            Updated, Sep. 2016
***********************************************************/

/*
  Headers for 3DS management - model loading for point clouds
*/
#include "include/lib3ds/types.h"
#include "include/lib3ds/file.h"
#include "include/lib3ds/mesh.h"

/*
   Headers for OpenGL libraries. If you want to run this
   on your computer, make sure you have installed OpenGL,
   GLUT, and GLUI

   NOTE: The paths below assume you're working on mathlab.
   On your system the libraries may be elsewhere. Be sure
   to check the correct location for include files and
   library files (you may have to update the Makefile)
*/

#include <OpenGL/OpenGL.h>
#include <GLUT/GLUT.h>
#include "imgui.h"
#include "imgui_impl_glut.h"

/* Standard C libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

// *************** GLOBAL VARIABLES *************************
#define MAX_BOIDS 2000
#define SPACE_SCALE 75
#define SPEED_SCALE 5
#define HISTORY 100                 // Amount of previous locations points to keep
const float PI = 3.14159;
int nBoids;				// Number of boids to dispay
float Boid_Location[MAX_BOIDS][3];	// Pointers to dynamically allocated
float Boid_Velocity[MAX_BOIDS][3];	// Boid position & velocity data
float Boid_Color[MAX_BOIDS][3];	 	// RGB colour for each boid
float Boid_Past_Locations[MAX_BOIDS][HISTORY][3];   // Previous locations of each boid
float *modelVertices;               // Imported model vertices
int Boid_Model_Vertex[MAX_BOIDS];	// Assigned model vertex for boid i
int n_vertices;                     // Number of model vertices
int nLeaders;						// How many leaders there are
int leaders[5];						// Array of leaders
float swimPhase;					// Controls swimming animation for boid
float swimSpeed;                    // Speed at which boid swims

// *************** USER INTERFACE VARIABLES *****************
int windowID;               // Glut window ID (for display)
int Win[2];                 // window (x,y) size
ImVec4 clear_color = ImColor(0.01f,0.01f,0.01f,1.0f);
float r_rule1;		    // Parameters of the boid update function.
float r_rule2;		    // see updateBoid()
float r_rule3;
float r_ruleLeader;		// Crunchy follow the leader
float k_rule1;
float k_rule2;
float k_rule3;
float k_rule0;
float k_ruleLeader;		// Crunchy follow the leader
float k_ruleHover;		// Crunchy hover over model vertex
float shapeness;
float global_rot;

// ***********  FUNCTION HEADER DECLARATIONS ****************
// Initialization functions
void initGlut(char* winName);
void GL_Settings_Init();
//float *read3ds(const char *name, int *n);

// Callbacks for handling events in glut
void WindowReshape(int w, int h);
void WindowDisplay(void);
void MouseClick(int button, int state, int x, int y);
void MotionFunc(int x, int y) ;
void PassiveMotionFunc(int x, int y) ;
void KeyboardPress(unsigned char key, int x, int y);
void KeyboardPressUp(unsigned char key, int x, int y);

void setupUI();

// Return the current system clock (in seconds)
double getTime();

// Functions for handling Boids
float sign(float x){if (x>=0) return(1.0); else return(-1.0);}
void updateBoid(int i);
void drawBoid(int i);
void HSV2RGB(float H, float S, float V, float *R, float *G, float *B);

// Functions to compute effects of boid rules in update function
void applyRule1(int boidIdx, float *v);
void applyRule2(int boidIdx, float *v);
void applyRule3(int boidIdx, float *v);
void followLeaders(int boidIdx, float *v);
void followModelVertex(int boidIdx, float *v);

// General helper functions
float distance(float *p1, float *p2, int dim);
int boidsInRange(int boidIdx, float range, int *allInRange);
bool isLeader(int boidIdx);
void assignToModelVertices();
void assignToColors();
void assignPastLocations();
void drawTrajectory(int i);
int min(int a,int b) {return a<b ? a : b;}

// ******************** FUNCTIONS ************************

/*
   main()

   Read command line parameters, initialize boid positions
   and velocities, and initialize all OpenGL data needed
   to set up the image window. Then call the GLUT main loop
   which handles the actual drawing
*/
int main(int argc, char** argv)
{
    // Process program arguments
    if(argc < 4 || argc > 5) {
        fprintf(stderr,"Usage: Boids width height nBoids [3dmodel]\n");
        fprintf(stderr," width & height control the size of the graphics window\n");
        fprintf(stderr," nBoids determined the number of Boids to draw.\n");
        fprintf(stderr," [3dmodel] is an optional parameter, naming a .3ds file to be read for 3d point clouds.\n");
        exit(0);
    }
    Win[0]=atoi(argv[1]);
    Win[1]=atoi(argv[2]);
    nBoids=atoi(argv[3]);

    if (nBoids>MAX_BOIDS)
    {
     fprintf(stderr,"Too Many Boids! Max=%d\n",MAX_BOIDS);
     exit(0);
    }

    // If a model file is specified, read it, normalize scale
    n_vertices=0;
    modelVertices=NULL;
    if (argc==5)
    {
     float mx=0;
     //n_vertices=nBoids;
     //modelVertices=read3ds(argv[4],&n_vertices);
     if (n_vertices>0)
     {
      fprintf(stderr,"Returned %d points\n",n_vertices);
      for (int i=0; i<n_vertices*3; i++)
       if (fabs(*(modelVertices+i))>mx) mx=fabs(*(modelVertices+i));
      for (int i=0; i<n_vertices*3; i++) *(modelVertices+i)/=mx;
      for (int i=0; i<n_vertices*3; i++) *(modelVertices+i)*=(SPACE_SCALE*.5);
     }
    }

    // Initialize Boid positions and velocity
    // Mind the SPEED_SCALE. You may need to change it to
    // achieve smooth animation - increase it if the
    // animation is too slow. Decrease it if it's too
    // fast and choppy.
    srand48(1522);
    for (int i=0; i<nBoids; i++)
    {
     // Initialize Boid locations and velocities randomly
     Boid_Location[i][0]=(-.5+drand48())*SPACE_SCALE;
     Boid_Location[i][1]=(-.5+drand48())*SPACE_SCALE;
     Boid_Location[i][2]=(-.5+drand48())*SPACE_SCALE;
     Boid_Velocity[i][0]=(-.5+drand48())*SPEED_SCALE;
     Boid_Velocity[i][1]=(-.5+drand48())*SPEED_SCALE;
     Boid_Velocity[i][2]=(-.5+drand48())*SPEED_SCALE;
    }
    
    // Initialize the past locations to the current one
    assignPastLocations();
    
    // Assign every boid to a model vertext, if model present
    assignToModelVertices();
    
    // Assign every boid a random color
    assignToColors();
    
    // Initialize glut, glui, and opengl
    glutInit(&argc, argv);
    initGlut(argv[0]);
    ImGui_ImplGlut_Init(false);
    GL_Settings_Init();

    // Initialize variables that control the boid updates
    r_rule1=15;
    r_rule2=1;
    r_rule3=25;
    r_ruleLeader=1;
    k_rule1=0.15;
    k_rule2=.5;
    k_rule3=.15;
    k_rule0=.25;
    k_ruleLeader=0.0;
    k_ruleHover=0.0;
    shapeness=0;
    global_rot=30;
    
    // Initialize variables that control the boid swimming animation
    swimPhase = 0.0;
    swimSpeed = 0.1;
    
    // Initialize leader list
    nLeaders = min(rand()%5 + 1, MAX_BOIDS);
    for (int i = 0; i < nLeaders; ++i) {
        leaders[i] = rand()%nBoids;
    }
    
    // Invoke the standard GLUT main event loop
    glutMainLoop();
    ImGui_ImplGlut_Shutdown();
    exit(0);         // never reached
}

// Initialize glut and create a window with the specified caption
void initGlut(char* winName)
{
    // Set video mode: double-buffered, color, depth-buffered
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    // We will learn about all of these later, for now, what you
    // need to know is that with the settings above, the graphics
    // window will keep track of the depth of objects so that
    // objects in the back can be properly obscured by objects
    // in front of them. The double buffer is used to ensure
    // smooth-looking animation.

    // Create window
    glutInitWindowPosition (0, 0);
    glutInitWindowSize(Win[0],Win[1]);
    windowID = glutCreateWindow(winName);

    // Setup callback functions to handle window-related events.
    // In particular, OpenGL has to be informed of which functions
    // to call when the image needs to be refreshed, and when the
    // image window is being resized.
    glutReshapeFunc(WindowReshape);   // Call WindowReshape whenever window resized
    glutDisplayFunc(WindowDisplay);   // Call WindowDisplay whenever new frame needed
    glutMouseFunc(MouseClick);
    glutMotionFunc(MotionFunc);
    glutKeyboardFunc(KeyboardPress);
    glutKeyboardUpFunc(KeyboardPressUp);
    glutPassiveMotionFunc(PassiveMotionFunc);
}

void MouseClick(int button, int state, int x, int y) {
    ImGui_ImplGlut_MouseButtonCallback(button, state, x, y);
}
void MotionFunc(int x, int y) {
    ImGui_ImplGlut_MotionCallback(x, y);
}
void PassiveMotionFunc(int x, int y) {
    ImGui_ImplGlut_PassiveMotionCallback(x, y);
}
void KeyboardPress(unsigned char key, int x, int y) {
    ImGui_ImplGlut_KeyCallback(key,x,y);
}
void KeyboardPressUp(unsigned char key, int x, int y) {
    ImGui_ImplGlut_KeyUpCallback(key,x,y);
}

// Quit button handler.  Called when the "quit" button is pressed.
void quitButton(int)
{
  if (modelVertices!=NULL && n_vertices>0) free(modelVertices);
  exit(0);
}

void setupUI()
{
    glDisable(GL_LIGHTING);
    ImGui_ImplGlut_NewFrame();
    ImGui::Begin("Boid CSC D18 Window");

    ////////////////////////////////////////
    // LEARNING OBJECTIVES:
    //
    // This part of the assignment is meant to help
    // you learn about:
    //
    // - How to create simple user interfaces for your
    //   OpenGL programs using GLUI.
    // - Defining and using controllers to change
    //   variables in your code.
    // - Using the GUI to change the behaviour of
    //   your program and the display parameters
    //   used by OpenGL.
    //
    // Be sure to check on-line references for GLUI
    // if you want to learn more about controller
    // types.
    //
    // See: http://www.eng.cam.ac.uk/help/tpl/graphics/using_glui.html
    //
    ////////////////////////////////////////

    ///////////////////////////////////////////////////////////
    // TO DO:
    //   Add controls to change the values of the variables
    //   used in the updateBoid() function. This includes
    //
    //   - k_rule1, k_rule2, k_rule3, k_rule0
    //   - r_rule2, r_rule3
    //
    //   Ranges for these variables are given in the updateBoid()
    //   function. Make sure to set the increments to a reasonable
    //   value.
    //
    //   An example control for r_rule1 is shown below.
    //
    //   In addition to this, add a single control to provide
    //   global rotation aound the vertical axis of the scene
    //   so that we can view the scene from different angles.
    //   The variable for this is called 'global_rot'.
    //
    ///////////////////////////////////////////////////////////

    ImGui::SetWindowFocus();
        ImGui::ColorEdit3("clear color", (float*)&clear_color);
//    EXAMPLE control for r_rule1
//    Controller name ---|
//                       v
    ImGui::SliderFloat(      "r_rule1",         &r_rule1, 10.0f, 100.0f);
//          Text to show --------|                 |       |       |
//          Variable whose value will be changed --|       |-------|
//          Min/Max values available ----------------------|-------|
    ImGui::SliderFloat(      "r_rule2",         &r_rule2, 1.0f, 15.0f);
    ImGui::SliderFloat(      "r_rule3",         &r_rule3, 10.0f, 100.0f);
    ImGui::SliderFloat(      "r_ruleLeader",    &r_ruleLeader, 1.0f, 10.0f);
    
    ImGui::SliderFloat(      "k_rule1",         &k_rule1, 0.0f, 1.0f);
    ImGui::SliderFloat(      "k_rule2",         &k_rule2, 0.0f, 1.0f);
    ImGui::SliderFloat(      "k_rule3",         &k_rule3, 0.0f, 1.0f);
    ImGui::SliderFloat(      "k_rule0",         &k_rule0, 0.0f, 1.0f);
    ImGui::SliderFloat(      "k_ruleLeader",    &k_ruleLeader, 0.0f, 1.0f);
    ImGui::SliderFloat(      "k_ruleHover",     &k_ruleHover, 0.0f, 1.0f);
    
    ImGui::SliderFloat(      "global_rot",     &global_rot, 0.0f, 360.0f);

    // Add "Quit" button
    if(ImGui::Button("Quit")) {
        quitButton(0);
    }

    //Some extra info
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);


    //End window
    ImGui::End();



    ImGui::Render();
    glEnable(GL_LIGHTING);
}

/*
  Reshape callback function. Takes care of handling window resizing
  events.
*/
void WindowReshape(int w, int h)
{
    // Setup projection matrix for new window

    // We will learn about projections later on. The projection mode
    // determines how 3D objects are 'projected' onto the 2D image.

    // Most graphical operations in OpenGL are performed through the
    // use of matrices. Below, we let OpenGL know that we will be
    // working with the GL_PROJECTION matrix, which controls the
    // projection of objects onto the image.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();			// Initialize with identity matrix

    // We will use perspective projection. This simulates a simple
    // pinhole camera.

    // The line below specifies the general shape and properties of
    // the viewing volume, that is, the region of space that is
    // visible within the image window.
    gluPerspective(45,1,15,500);
    //              ^ ^  ^  ^
    // FOV ---------| |  |  |		// See OpenGL reference for
    // Aspect Ratio --|  |  |		// more details on using
    // Near plane -------|  |		// gluPerspective()
    // Far plane -----------|

    // Set the OpenGL viewport - this corresponds to the 2D image
    // window. It uses pixels as units. So the instruction below
    // sets the image window to start at pixel coordinate (0,0)
    // with the specified width and height.
    glViewport(0,0,w,h);
    Win[0] = w;
    Win[1] = h;

    // glutPostRedisplay()	// Is this needed?
}

void GL_Settings_Init()
{
 // Initialize OpenGL parameters to be used throughout the
 // life of the graphics window

    // Set the background colour
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);

    // Enable alpha blending for transparency
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

/**** Illumination set up start ****/

    // The section below controls the illumination of the scene.
    // Nothing can be seen without light, so the section below
    // is quite important. We will discuss different types of
    // illumination and how they affect the appearance of objecs
    // later in the course.
    glClearDepth(1);
    glEnable(GL_DEPTH_TEST);    // Enable depth testing
    glEnable(GL_LIGHTING);      // Enable lighting
    glEnable(GL_LIGHT0);        // Enable LIGHT0 for diffuse illumination
    glEnable(GL_LIGHT1);        // Enable LIGHT1 for ambient illumination

    // Set up light source colour, type, and position
    GLfloat light0_colour[]={1.0,1.0,1.0};
    GLfloat light1_colour[]={.25,.25,.25};
    GLfloat light0_pos[]={500,0,500,0};
    glLightfv(GL_LIGHT0,GL_DIFFUSE,light0_colour);
    glLightfv(GL_LIGHT1,GL_AMBIENT,light1_colour);
    glLightfv(GL_LIGHT0,GL_POSITION,light0_pos);
    glShadeModel(GL_SMOOTH);

    // Enable material colour properties
    glEnable(GL_COLOR_MATERIAL);

/***** Illumination setup end ******/
}

/*
   Display callback function.
   This has to be called whenever the image needs refreshing,
   either as a result of updates to the graphical content of
   the window (e.g. animation is taking place), or as a
   result of events outside this window (e.g. other programs
   may create windows that partially occlude the OpenGL window,
   when the occlusion ends, the obscured region has to be
   refreshed)
*/
void WindowDisplay(void)
{
    // Clear the screen and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);


/***** Scene drawing start *********/

    // Here is the section where shapes are actually drawn onto the image.
    // In this case, we call a function to update the positions of boids,
    // and then draw each boid at the updated location.

    // Setup the model-view transformation matrix
    // This is the matrix that determines geometric
    // transformations applied to objects. Typical
    // transformations include rotations, translations,
    // and scaling.
    // Initially, we set this matrix to be the identity
    // matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Rotate the camera to the value in global_rot between 0 and 360 degrees
    gluLookAt(145*cos(global_rot*PI/180.0),145*sin(global_rot*PI/180.0),80,0,0,0,0,0,1);

    // Draw box bounding the viewing area
    glColor4f(.95,.95,.95,.95);
    glBegin(GL_LINE_LOOP);
     glVertex3f(-50,-50,-50);
     glVertex3f(-50,-50,50);
     glVertex3f(-50,50,50);
     glVertex3f(-50,50,-50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(-50,-50,-50);
     glVertex3f(-50,-50,50);
     glVertex3f(50,-50,50);
     glVertex3f(50,-50,-50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(-50,-50,-50);
     glVertex3f(-50,50,-50);
     glVertex3f(50,50,-50);
     glVertex3f(50,-50,-50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(50,50,50);
     glVertex3f(50,50,-50);
     glVertex3f(50,-50,-50);
     glVertex3f(50,-50,50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(50,50,50);
     glVertex3f(50,50,-50);
     glVertex3f(-50,50,-50);
     glVertex3f(-50,50,50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(50,50,50);
     glVertex3f(50,-50,50);
     glVertex3f(-50,-50,50);
     glVertex3f(-50,50,50);
    glEnd();

    for (int i=0; i<nBoids; i++)
    {
        updateBoid(i);		// Update position and velocity for boid i
        drawTrajectory(i);  // Draw the trajectory for boid i
        drawBoid(i);		// Draw this boid
    }
    swimPhase += swimSpeed;	// move the phase for the next boid animation

    setupUI();
    // Make sure all OpenGL commands are executed
    glFlush();

    // Swap buffers to enable smooth animation
    glutSwapBuffers();
/***** Scene drawing end ***********/

  // synchronize variables that GLUT uses

  // Tell glut window to update itself
  glutSetWindow(windowID);
  glutPostRedisplay();
}

void updateBoid(int i)
{
    /*
     This function updates the position and velocity of Boid i, read the handout
     and read the reference material in order to understand how the position and
     velocity are updated.
     */
    // Velocity changes obtained from r and k rules
    float v1[3], v2[3], v3[3], vLead[3], vHover[3];
    
    ///////////////////////////////////////////
    // TO DO: Complete this function to update
    // the Boid's velocity and location
    //
    // Reference: http://www.vergenet.net/~conrad/boids/pseudocode.html
    //
    // You need to implement Rules 1, 2, 3, and
    //  paco's Rule 0. You must also do the
    //  position update.
    //
    // Add at the top of this function any variables
    // needed.
    ///////////////////////////////////////////
    
    ///////////////////////////////////////////
    // LEARNING OBJECTIVES: This part of the assignment
    // is meant to help you learn about
    //
    // - Representing quantities using vectors
    //   and points (velocity, direction of motion,
    //   position, etc.)
    // - Vector manipulation. Finding vectors with
    //   a specific direction, adding and subtracting
    //   vector quantities.
    // - Thinking in terms of points, lines, and
    //   their relationship to vectors:
    //   Remember, a parametric line is given
    //   by l(k)= p + (k*v)
    //   where p is a point, v is  vector in the
    //   direction of the line, and k a parameter.
    // - Implementing simple rules that produce
    //   complex visual behaviour.
    //
    // Be sure you fully understand these ideas!
    // if you run into trouble, come to office
    // hours.
    ///////////////////////////////////////////
    
    
    ///////////////////////////////////////////
    //
    // TO DO:
    //
    // Boid update Rule 1:
    //  Move boids toward the common center of
    // mass.
    //
    //  In the reference, the center of
    // mass is computed for ALL boids. Here we
    // will do it a little differently:
    //
    //  Compute the center of mass for all boids
    // that are within a radius r_rule1 of the
    // current boid. r_rule1 is a variable that
    // can be manipulated via the user interface.
    //
    //  Once you have obtained a vector from
    // the current boid to the center of mass
    // you must update the velocity vector
    // for this Boid so that it will move
    // toward the center of mass.
    //
    //  The amount it will move toward the
    // center of mass is given by another user
    // interface variable called k_rule1.
    //
    // In effect:
    //  Boid_Velocity += (k_rule1) * V1
    //
    // where V1 is the vector from the current
    // boid position to the center of mass.
    //
    // Valid ranges are:
    //  10 <= r_rule1 <= 100
    //  0 <= k_rule1 <= 1
    ///////////////////////////////////////////
    
    applyRule1(i, v1);
    
    ///////////////////////////////////////////
    // QUESTION:
    //  Is this the optimal way to implement this
    // rule? can you see any problems or ways
    // to improve this bit?
    ///////////////////////////////////////////
    
    
    
    ///////////////////////////////////////////
    //
    // TO DO:
    //
    // Boid update Rule 2:
    //  Boids steer to avoid collision.
    //
    // For each boid j within a small distance
    // r_rule2 (changeable through the user
    // interface), compute a vector
    // from boid(i) to boid(j) (MIND THE DIRECTION!)
    // then subtract a small amount k_rule2
    // times this vector from boid(i)'s
    // velocity. i.e.
    //
    // Boid_Velocity[i] -= k_rule2 * V2
    //
    // where V2 is the vector from boid(i)
    // to void(j).
    //
    // k_rule2 can be manipulated through the
    // user interface.
    //
    // Valid ranges are
    //  1 <= r_rule2 <= 15
    //  0 <= k_rule2 <= 1
    ///////////////////////////////////////////
    
    applyRule2(i, v2);
    
    
    ///////////////////////////////////////////
    //
    // TO DO:
    //
    // Boid update Rule 3:
    //  Boids try to match speed with neighbours
    //
    //  Average the velocity of any boids within
    // a given distance r_rule3 from boid(i).
    // r_rule3 can be manipulated via the user
    // interface.
    //
    //  Once the average velocity has been
    // computed, add a small factor times this
    // velocity to the boid's velocity, i.e.
    //
    // Boid_Velocity[i] += k_rule3 * V3
    //
    // where V3 is the average velocity for
    // nearby boids and k_rule3 is a parameter
    // that can be set via the user interface.
    //
    // Valid ranges:
    //
    // 10 <= r_rule3 <= 100
    // 0 <= k_rule3 <= 1
    ///////////////////////////////////////////
    
    applyRule3(i, v3);
    
    // Crunchy effects
    followLeaders(i, vLead);
    followModelVertex(i, vHover);
    
    ///////////////////////////////////////////
    // DONE - CRUNCHY: Add a 'shapeness' component.
    //  this should give your Boids a tendency
    //  to hover near one of the points of a
    //  3D model imported from file. Evidently
    //  each Boid should hover to a different
    //  point, and you must figure out how to
    //  make the void fly toward that spot
    //  and hover more-or-less around it
    //  (depending on all Boid parameters
    //   for the above rules).
    //
    //  3D model data is imported for you when
    //  the user specifies the name of a model
    //  file in .3ds format from the command
    //  line.
    //
    //  The model data
    //  is stored in the modelVertices array
    //  and the number of vertices is in
    //  n_vertices (if zero, there is no model
    //  and the shapeness component should
    //  have no effect whatsoever)
    //
    //  The coordinates (x,y,z) of the ith
    //  mode, vertex can be accessed with
    //  x=*(modelVertices+(3*i)+0);
    //  y=*(modelVertices+(3*i)+1);
    //  z=*(modelVertices+(3*i)+2);
    //
    //  Evidently, if you try to access more
    //  points than there are in the array
    //  you will get segfault (don't say I
    //  didn't warn you!). Be careful with
    //  indexing.
    //
    //  shapeness should be in [0,1], and
    //  there is already a global variable
    //  to store it. You must add a slider
    //  to the GUI to control the amount of
    //  shapeness (i.e. how strong the shape
    //  constraints affect Boid position).
    //
    //  .3ds models can be found online, you
    //  *must ensure* you are using a freely
    //  distributable model!
    //
    //////////////////////////////////////////
    
    ///////////////////////////////////////////
    //
    // TO DO:
    //
    // Paco's Rule Zero,
    //
    //   Boids have inertia - they like to keep
    // going in the same direction as before.
    //
    //   Add a component to the boid velocity
    // that depends on the previous velocity
    // (i.e. before the above updates). The weight
    // of this term is given by k_rule0, which
    // can be set in the user interface.
    //
    // Vaid ranges:
    //   0 < k_rule0 < 1
    ///////////////////////////////////////////
    
    // Update the velocity with the inertia and the rest of the rules
    Boid_Velocity[i][0] += (v1[0]+v2[0]+v3[0]+vLead[0]+vHover[0])
    + k_rule0*Boid_Velocity[i][0];
    Boid_Velocity[i][1] += (v1[1]+v2[1]+v3[1]+vLead[1]+vHover[1])
    + k_rule0*Boid_Velocity[i][1];
    Boid_Velocity[i][2] += (v1[2]+v2[2]+v3[2]+vLead[2]+vHover[2])
    + k_rule0*Boid_Velocity[i][2];
    
    ///////////////////////////////////////////
    // Enforcing bounds on motion
    //
    //  This is already implemented: The goal
    // is to ensure boids won't stray too far
    // from the viewing area.
    //
    //  This is done exactly as described in the
    // reference.
    //
    //  Bounds on the viewing region are
    //
    //  -50 to 50 on each of the X, Y, and Z
    //  directions.
    ///////////////////////////////////////////
    if (Boid_Location[i][0]<-50) Boid_Velocity[i][0]+=1;
    if (Boid_Location[i][0]>50) Boid_Velocity[i][0]-=1;
    if (Boid_Location[i][1]<-50) Boid_Velocity[i][1]+=1;
    if (Boid_Location[i][1]>50) Boid_Velocity[i][1]-=1;
    if (Boid_Location[i][2]<-50) Boid_Velocity[i][2]+=1;
    if (Boid_Location[i][2]>50) Boid_Velocity[i][2]-=1;
    
    ///////////////////////////////////////////
    // Velocity Limit:
    //  This is already implemented. The goal
    // is simply to avoid boids shooting off
    // at unrealistic speeds.
    //
    //  You can tweak this part if you like,
    // or you can simply leave it be.
    //
    //  The speed clamping used here was determined
    // 'experimentally', i.e. I tweaked it by hand!
    ///////////////////////////////////////////
    Boid_Velocity[i][0]=sign(Boid_Velocity[i][0])*sqrt(fabs(Boid_Velocity[i][0]));
    Boid_Velocity[i][1]=sign(Boid_Velocity[i][1])*sqrt(fabs(Boid_Velocity[i][1]));
    Boid_Velocity[i][2]=sign(Boid_Velocity[i][2])*sqrt(fabs(Boid_Velocity[i][2]));
    
    ///////////////////////////////////////////
    // QUESTION: Why add inertia at the end and
    //  not at the beginning?
    ///////////////////////////////////////////
    
    ///////////////////////////////////////////
    //
    // TO DO:
    //
    // Finally (phew!) update the position
    // of this boid.
    ///////////////////////////////////////////
    
    Boid_Location[i][0] += Boid_Velocity[i][0]*1/SPEED_SCALE;
    Boid_Location[i][1] += Boid_Velocity[i][1]*1/SPEED_SCALE;
    Boid_Location[i][2] += Boid_Velocity[i][2]*1/SPEED_SCALE;
    
    ///////////////////////////////////////////
    // CRUNCHY:
    //
    //  Things you can add here to make the behaviour
    // more interesting. Be sure to note in your
    // report any extra work you have done.
    //
    // - Add a few obstacles (boxes or something like it)
    //   and add code to have boids avoid these
    //   obstacles
    //
    // - DONE - Follow the leader: Select a handful
    //   (1 to 5) boids randomly. Add code so that
    //   nearby boids tend to move toward these
    //   'leaders'
    //
    // - Make the updates smoother: Idea, instead
    //   of having hard thresholds on distances for
    //   the update computations (r_rule1, r_rule2,
    //   r_rule3), use a weighted computation
    //   where contributions are weighted by
    //   distance and the weight decays as a
    //   function of the corresponding r_rule
    //   parameter.
    //
    // - Add a few 'predatory boids'. Select
    //   a couple of boids randomly. These become
    //   predators and the rest of the boids
    //   should have a strong tendency to
    //   avoid them. The predatory boids should
    //   follow the standard rules. However,
    //   Be sure to plot the predatory boids
    //   differently so we can easily see
    //   who they are.
    //
    // - Make it go FAST. Consider and implement
    //   ways to speed-up the boid update. Hint:
    //   good approximations are often enough
    //   to give the right visual impression.
    //   What and how to approximate? that is the
    //   problem.
    //
    //   Thoroughly describe any crunchy stuff in
    //   the REPORT.
    //
    ///////////////////////////////////////////
    
    return;
}

void drawBoid(int i)
{
    /*
     This function draws a boid i at the specified location.
     */
    
    ///////////////////////////////////////////
    // LEARNING OBJECTIVES:
    //
    // This part of the assignment is meant to help
    // you learn about:
    //
    // - Creating simple shapes in OpenGL
    // - Positioning shapes in space for drawing
    // - Colour and transparency
    // - Manipulating the GL_MODELVIEW matrix
    //
    // Be sure to read your OpenGL reference
    // and make sure you understand the basic
    // concepts comprising transformations,
    // shape definition, and drawing shapes on
    // screen.
    //
    // Your TA will be able to help with programming
    // related issues during tutorial.
    //
    ///////////////////////////////////////////
    
    ///////////////////////////////////////////
    // TO DO: Complete this function to draw a
    // nice boid at the corresponding position
    // You may want to learn about how to define
    // OpenGL polygons, or about simple shapes
    // provided by the GLUT library.
    //
    // You may also want to consider giving
    // different boids different colours.
    //
    // Examples of boid shapes you could do
    // are fish, birds, butterflies, or
    // dragonflies, but really anything else
    // that's imaginative is fine.
    //
    ///////////////////////////////////////////
    
    // Drawing a NARWHAL
    
    // Animation angles for which to rotate body parts
    float *location = Boid_Location[i];
    float *velocity = Boid_Velocity[i];
    float *color = Boid_Color[i];
    float swimAngle = sin(swimPhase);	// base angle for fin/tail rotation
    float leftFinAngle = -50.0 - 20*swimAngle;
    float rightFinAngle = 50.0 + 20*swimAngle;
    float lowerBodyAngle = 10*swimAngle;
    float tailAngle = lowerBodyAngle + 30*swimAngle;
    
    // Find the angles for the direction in which the narwhal is pointing
    // in order to rotate it accordingly. Ommiting the 'roll', because it
    // would necessitate finding the acceleration of the narwhal. Thus,
    // the narwhal will never be rotated about it's long axes (axis of travel)
    float origin[3] = {0, 0, 0};
    float speed = distance(velocity, origin, 3);
    float speedXY = distance(velocity, origin, 2);
    float yaw = 180.0/PI*atan2(velocity[1], velocity[0]) + 90;
    float pitch = -180.0/PI*asinf(velocity[2]/speed) + 90;
    
    glColor4f(color[0], color[1], color[2], 1);
    GLUquadric *quad = gluNewQuadric();
    
    // Transform to the boid's position and orient to it's
    // velocity
    glPushMatrix();
    glTranslatef(location[0], location[1], location[2]);
    glRotatef(yaw, 0, 0, 1);
    glRotatef(pitch, 1, 0, 0);
    glScalef(1, 1, 1);
    
    // Draw the upper body
    glPushMatrix();
    glScalef(1, 1, 2);
    gluSphere(quad, 1, 4, 4);
    glPopMatrix();
    
    // Draw the horn
    glColor4f(0.7, 0.7, 0.55, 1);
    glPushMatrix();
    glTranslatef(0, -0.15, 1.8);
    glutSolidCone(0.15, 1.8, 4, 4);
    glPopMatrix();
    
    // Draw the eyes
    glColor4f(0.5, 0.85, 0.85, 1);
    glPushMatrix();
    glTranslatef(-0.3, 0.2, 1.7);
    gluSphere(quad, 0.2, 4, 4);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(0.3, 0.2, 1.7);
    gluSphere(quad, 0.2, 4, 4);
    glPopMatrix();
    
    // Draw the fins
    glColor4f(color[0], color[1], color[2], 1);
    glPushMatrix();
    glTranslatef(1, 0, 0.45);
    glRotatef(leftFinAngle, 0, 1, 0);
    glTranslatef(0, 0, -0.3);
    glScalef(0.2, 1, 2);
    gluSphere(quad, 0.35, 4, 4);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-1, 0, 0.45);
    glRotatef(rightFinAngle, 0, 1, 0);
    glTranslatef(0, 0, -0.3);
    glScalef(0.2, 1, 2);
    gluSphere(quad, 0.35, 4, 4);
    glPopMatrix();
    
    //Draw the lower body and tail
    glPushMatrix();
    // lower body
    glRotatef(lowerBodyAngle, 1, 0, 0);
    glPushMatrix();
    glTranslatef(0, 0, -1.2);
    glScalef(1, 1, 2.5);
    gluSphere(quad, 0.8, 4, 4);
    glPopMatrix();
    // tail
    glPushMatrix();
    glTranslatef(0.5, 0, -3);
    glRotatef(tailAngle, 1, 0, 0);
    glRotatef(-25, 0, 1, 0);
    glScalef(1.2, 0.5, 1.5);
    gluSphere(quad, 0.5, 4, 4);
    glPopMatrix();
    glPushMatrix();
    glTranslatef(-0.5, 0, -3);
    glRotatef(tailAngle, 1, 0, 0);
    glRotatef(25, 0, 1, 0);
    glScalef(1.2, 0.5, 1.5);
    gluSphere(quad, 0.5, 4, 4);
    glPopMatrix();
    glPopMatrix();
    
    glPopMatrix();
    
    ///////////////////////////////////////////
    // CRUNCHY:
    //
    //  DONE - Animate boids: You can draw your boids
    // differently on each frame to provide
    // animation. For example, birds may flap
    // their wings as they move, fish may
    // move their fins. Do something appropriate
    // that looks good for extra credit.
    //
    ///////////////////////////////////////////
    
    ///////////////////////////////////////////
    // CRUNCHY:
    //
    //  Add trails that show nicely the last
    // few positions of each boid, if done
    // properly this will give you very nice
    // trajectories in space.
    //
    //  You're free to add variables to keep
    // track of the Boids' past positions, and
    // you have to figure out how best to
    // display the trajectory. Make it look
    // awesome for a good bonus!
    //
    ///////////////////////////////////////////
    
}

void HSV2RGB(float H, float S, float V, float *R, float *G, float *B)
{
    // Handy function to convert a colour specified as an HSV triplet
    // to RGB values used by OpenGL. You can use this function to
 // set the boids' colours in a more intuitive way. To learn
 // about HSV colourspace, check the Wikipedia page.
 float c,x,hp,r1,g1,b1,m;

 hp=H*6;
 c=V*S;
 x=c*(1.0-fabs(fmod(hp,2)-1.0));
 if (hp<1){r1=c;g1=x;b1=0;}
 else if (hp<2){r1=x;g1=c;b1=0;}
 else if (hp<3){r1=0;g1=c;b1=x;}
 else if (hp<4){r1=0;g1=x;b1=c;}
 else if (hp<5){r1=x;g1=0;b1=c;}
 else{r1=c;g1=0;b1=x;}

 m=V-c;
 *R=r1+m;
 *G=g1+m;
 *B=b1+m;

 if (*R>1) *R=1;
 if (*R<0) *R=0;
 if (*G>1) *G=1;
 if (*G<0) *G=0;
 if (*B>1) *B=1;
 if (*B<0) *B=0;
}

//float *read3ds(const char *name, int *n)
//{
// /*
//   Read a model in .3ds format from the specified file.
//   If the model is read successfully, a pointer to a
//   float array containing the vertex coordinates is
//   returned.
//
//   Input parameter n is used to specify the maximum
//   number of vertex coordinates to return, as well
//   as to return the actual number of vertices read.
//
//
//   Vertex coordinates are stored consecutively
//   so each vertex occupies in effect 3 consecutive
//   floating point values in the returned array
// */
// Lib3dsFile *f;
// Lib3dsMesh *mesh;
// Lib3dsPoint *pt;
// int n_meshes, n_points;
// Lib3dsVector *vertex_data;
// float *vertices, *v_return;
// float inc_step;
// int idx;
//
// f=lib3ds_file_load(name);
// if (f==NULL)
// {
//  fprintf(stderr,"Unable to load model data\n");
//  *n=0;
//  return(NULL);
// }
//
// // Count meshes and faces
// n_points=0;
// n_meshes=0;
// mesh=f->meshes;
// while(mesh!=NULL)
// {
//  n_meshes++;
//  n_points+=mesh->points;
//  mesh=mesh->next;
// }
// fprintf(stderr,"Model contains %d meshes, %d points, %d coordinates\n",n_meshes,n_points,3*n_points);
//
// // Allocate data for vertex array and put all input points (from all meshes) in the array
// vertex_data=(Lib3dsVector *)calloc(n_points,sizeof(Lib3dsVector));
// mesh=f->meshes;
// while(mesh!=NULL)
// {
//  pt=mesh->pointL;
//  for (int i=0; i < mesh->points; i++)
//    memcpy((vertex_data+i),(pt+i),sizeof(Lib3dsVector));
//  mesh=mesh->next;
// }
//
// vertices=(float *)vertex_data;
// // Release memory allocated to the file data structure and return the vertex array
// lib3ds_file_free(f);
//
// if (n_points<(*n)) *(n)=n_points;                      // Less points than expected!
// v_return=(float *)calloc((*(n))*3,sizeof(float));      // Allocate space for n points
// inc_step=(n_points-1)/(*n);                            // Sampling step
// for (int i=0; i<(*(n)); i++)
// {
//  idx=floor(inc_step*i);
//  *(v_return+(3*i)+0)=*(vertices+(3*idx)+0);            // Mind the ordering! it's shuffled
//  *(v_return+(3*i)+1)=*(vertices+(3*idx)+2);            // to match our coordinate frame
//  *(v_return+(3*i)+2)=*(vertices+(3*idx)+1);
// }
//
// free(vertex_data);
// return(v_return);
//}

void applyRule1(int boidIdx, float *v) {
    float *self_position = Boid_Location[boidIdx];
    int nearby_boids[MAX_BOIDS];
    int nNearby = boidsInRange(boidIdx, r_rule1, nearby_boids);
    int neighbour_idx;
    float *neighbour_position;
    float centre[3] = {0, 0, 0};
    
    for (int i = 0; i < nNearby; i++) {
        neighbour_idx = nearby_boids[i];
        neighbour_position = Boid_Location[neighbour_idx];
        centre[0] += neighbour_position[0] / nNearby;
        centre[1] += neighbour_position[1] / nNearby;
        centre[2] += neighbour_position[2] / nNearby;
    }
    
    v[0] = (centre[0] - self_position[0]) * k_rule1;
    v[1] = (centre[0] - self_position[1]) * k_rule1;
    v[2] = (centre[0] - self_position[2]) * k_rule1;
}

void applyRule2(int boidIdx, float *v) {
    float *self_position = Boid_Location[boidIdx];
    int nearby_boids[MAX_BOIDS];
    int nNearby = boidsInRange(boidIdx, r_rule2, nearby_boids);
    int neighbour_idx;
    float *neighbour_position;
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
    
    for (int i = 0; i < nNearby; i++) {
        neighbour_idx = nearby_boids[i];
        neighbour_position = Boid_Location[neighbour_idx];
        v[0] -= (neighbour_position[0] - self_position[0]) * k_rule2;
        v[1] -= (neighbour_position[1] - self_position[1]) * k_rule2;
        v[2] -= (neighbour_position[2] - self_position[2]) * k_rule2;
    }
}

void applyRule3(int boidIdx, float *v) {
    int nearby_boids[MAX_BOIDS];
    int nNearby = boidsInRange(boidIdx, r_rule3, nearby_boids);
    int neighbour_idx;
    float *neighbour_velocity;
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
    
    for (int i = 0; i < nNearby; i++) {
        neighbour_idx = nearby_boids[i];
        if (neighbour_idx != boidIdx) {	// average not including self
            neighbour_velocity = Boid_Velocity[neighbour_idx];
            v[0] += k_rule3 * neighbour_velocity[0] / (nNearby-1);
            v[1] += k_rule3 * neighbour_velocity[1] / (nNearby-1);
            v[2] += k_rule3 * neighbour_velocity[2] / (nNearby-1);
        }
    }
}

void followLeaders(int boidIdx, float *v) {
    float *self_position = Boid_Location[boidIdx];
    int nearby_boids[MAX_BOIDS];
    int nNearby = boidsInRange(boidIdx, r_ruleLeader, nearby_boids);
    int neighbour_idx;
    float *neighbour_position;
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
    
    for (int i = 0; i < nNearby; i++) {
        neighbour_idx = nearby_boids[i];
        if (isLeader(neighbour_idx)) {
            neighbour_position = Boid_Location[neighbour_idx];
            v[0] += (neighbour_position[0] - self_position[0]) * k_ruleLeader;
            v[1] += (neighbour_position[1] - self_position[1]) * k_ruleLeader;
            v[2] += (neighbour_position[2] - self_position[2]) * k_ruleLeader;
        }
    }
}

void followModelVertex(int boidIdx, float *v) {
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
    if (n_vertices > 0) {
        int mIdx = Boid_Model_Vertex[boidIdx];
        float *self_position = Boid_Location[boidIdx];
        float *m_position = modelVertices + mIdx*3;
        v[0] = (m_position[0] - self_position[0]) * k_ruleHover;
        v[1] = (m_position[1] - self_position[1]) * k_ruleHover;
        v[2] = (m_position[2] - self_position[2]) * k_ruleHover;
    }
}

// Fills an array with indices of all the boids in range of boidIdx,
// and returns the size of the array
int boidsInRange(int boidIdx, float range, int *allInRange) {
    int nInRange = 0;
    float *self_position = Boid_Location[boidIdx];
    float *neighbour_position;
    float dist;
    
    for (int i = 0; i < nBoids; i++) {
        neighbour_position = Boid_Location[i];
        dist = distance(self_position, neighbour_position, 3);
        if (dist <= range) {
            allInRange[nInRange] = i;
            nInRange++;
        }
    }
    
    return nInRange;
}

// If there is a .3ds model, assigns each boid to
// hover around a randomly chosen vertex in the model.
void assignToModelVertices() {
    if (n_vertices > 0) {
        for (int i = 0; i < nBoids; ++i) {
            Boid_Model_Vertex[i] = rand() % n_vertices;
        }
    }
}

// Assigns an RGB value to every boid
void assignToColors() {
    for (int i = 0; i < nBoids; ++i) {
        Boid_Color[i][0] = (float)rand()/(float)(RAND_MAX);
        Boid_Color[i][1] = (float)rand()/(float)(RAND_MAX);
        Boid_Color[i][2] = (float)rand()/(float)(RAND_MAX);
    }
}

// Assigns initial values to the past locations of every boid
// as it's current location.
void assignPastLocations() {
    for (int i = 0; i < nBoids; ++i) {
        for (int j = 0; j < HISTORY; j++) {
            Boid_Past_Locations[i][j][0] = Boid_Location[i][0];
            Boid_Past_Locations[i][j][1] = Boid_Location[i][1];
            Boid_Past_Locations[i][j][2] = Boid_Location[i][2];
        }
    }
}

// Draws the trajectory for the given boid
void drawTrajectory(int i) {
    // Shift history down
    for (int j = HISTORY-1; j >= 1; j--) {
        Boid_Past_Locations[i][j][0] = Boid_Past_Locations[i][j-1][0];
        Boid_Past_Locations[i][j][1] = Boid_Past_Locations[i][j-1][1];
        Boid_Past_Locations[i][j][2] = Boid_Past_Locations[i][j-1][2];
    }
    
    // Update the most recent location
    Boid_Past_Locations[i][0][0] = Boid_Location[i][0];
    Boid_Past_Locations[i][0][1] = Boid_Location[i][1];
    Boid_Past_Locations[i][0][2] = Boid_Location[i][2];
    
    // Draw the trajectory as points of increasing red-ness
    glBegin(GL_POINTS);
    for (int j = HISTORY-1; j >=0; j--) {
        glColor4f((float)(HISTORY-j)/HISTORY, 0.0, 0.0, 1.0);
        glVertex3f(Boid_Past_Locations[i][j][0],
                   Boid_Past_Locations[i][j][1],
                   Boid_Past_Locations[i][j][2]);
    }
    glEnd();
}

// Distance between p1 and p2 in dim dimensions
float distance(float *p1, float *p2, int dim) {
    float sum = 0;
    for (int i = 0; i < dim; i++) {
        sum += (p1[i] - p2[i])*(p1[i] - p2[i]);
    }
    return sqrt(sum);
}

bool isLeader(int boidIdx) {
    for (int i = 0; i < nLeaders; ++i) {
        if (boidIdx == leaders[i]) {
            return true;
        }
    }
    return false;
}



