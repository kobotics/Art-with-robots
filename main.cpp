/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name: 

  main.cpp

Description:

  The main file that performs all haptics-relevant operation. Within a 
  asynchronous callback the graphics thread reads the position and sets
  the force. Within a synchronous callback the graphics thread gets the
  position and constructs graphics elements (e.g. force vector).

*******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <cstdio>
#include <cassert>

#include <HD/hd.h>

#include "helper.h"

#include <HDU/hduError.h>
#include <HDU/hduVector.h>

// Sample code to read in test cases:
#include <iostream>
#include <fstream>
#include <string>
using namespace std;

static double sphereRadius = 15;
static double mySphereRadius = 3;

/* Charge (positive/negative) */
int charge = 1;

static HHD ghHD = HD_INVALID_HANDLE;
static HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;

/* Glut callback functions used by helper.cpp */
void displayFunction(void);
void handleIdle(void);



hduVector3Dd forceField(hduVector3Dd pos, hduVector3Dd* shape, int shape_size);


hduVector3Dd* Shape;
int Shape_size = 0;

/* Haptic device record. */
struct DeviceDisplayState
{
    HHD m_hHD;
    hduVector3Dd position;
    hduVector3Dd force;
};

/*******************************************************************************
 Client callback used by the graphics main loop function.
 Use this callback synchronously.
 Gets data, in a thread safe manner, that is constantly being modified by the 
 haptics thread. 
*******************************************************************************/
HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserData)
{
    DeviceDisplayState *pDisplayState = 
        static_cast<DeviceDisplayState *>(pUserData);

    hdGetDoublev(HD_CURRENT_POSITION, pDisplayState->position);
    hdGetDoublev(HD_CURRENT_FORCE, pDisplayState->force);

    // execute this only once.
    return HD_CALLBACK_DONE;
}


/*******************************************************************************
 Graphics main loop function.
*******************************************************************************/
void displayFunction(void)
{
    // Setup model transformations.
    glMatrixMode(GL_MODELVIEW); 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();

    setupGraphicsState();
    drawAxes(sphereRadius*3.0);

    // Draw the fixed sphere.
    static const hduVector3Dd fixedSpherePosition(0, 0, 0);
    static const float fixedSphereColor[4] = {.2, .8, .8, .8};
    GLUquadricObj* pQuadObj = gluNewQuadric();
   // drawSphere(pQuadObj, fixedSpherePosition, fixedSphereColor, sphereRadius);

    // Get the current position of end effector.
    DeviceDisplayState state;
    hdScheduleSynchronous(DeviceStateCallback, &state,
                          HD_MIN_SCHEDULER_PRIORITY);

    // Draw a sphere to represent the haptic cursor and the dynamic 
    // charge.
    static const float dynamicSphereColor[4] = { .8, .2, .2, .8 };
    drawSphere(pQuadObj, 
               state.position,
               dynamicSphereColor,
               sphereRadius);    

	for (int i = 0; i < Shape_size; i++)
	{
		//glBegin(GL_POINTS); //starts drawing of points

		//glVertex3f(Shape[0][0], Shape[0][1], Shape[0][2]);//upper-right corner

		GLUquadricObj* pQuadObj1 = gluNewQuadric();
		drawSphere(pQuadObj1, Shape[i], fixedSphereColor, mySphereRadius);
		

		//glEnd();//end drawing of points
	}


    // Create the force vector.
    hduVector3Dd forceVector = 40.0 * forceField(state.position, Shape, Shape_size);
	//if (forceVector.magnitude() > 40.0) forceVector = normalize(forceVector) * 40.0;

    drawForceVector(pQuadObj,
                    state.position,
                    forceVector,
                    sphereRadius*.1);

    gluDeleteQuadric(pQuadObj);
  
    glPopMatrix();
    glutSwapBuffers();                      
}
                                
/*******************************************************************************
 Called periodically by the GLUT framework.
*******************************************************************************/
void handleIdle(void)
{
    glutPostRedisplay();

    if (!hdWaitForCompletion(gSchedulerCallback, HD_WAIT_CHECK_STATUS))
    {
        printf("The main scheduler callback has exited\n");
        printf("Press any key to quit.\n");
        getchar();
        exit(-1);
    }
}

/******************************************************************************
 Popup menu handler
******************************************************************************/
void handleMenu(int ID)
{
    switch(ID) 
    {
        case 0:
            exit(0);
            break;
        case 1:
            charge *= -1;
            break;
    }
}


/*******************************************************************************
 Given the position is space, calculates the (modified) coulomb force.
*******************************************************************************/
hduVector3Dd forceField(hduVector3Dd pos, hduVector3Dd* shape, int shape_size)
{
	double scale = 12.0;
	hduVector3Dd forceVec(0, 0, 0);
	for (int i = 0; i < shape_size; i++) {
		hduVector3Dd diff = pos - shape[i];
		double dist = diff.magnitude();


		// if two charges overlap...
		if (dist < sphereRadius*2.0)
		{
			// Attract the charge to the center of the sphere.
			hduVector3Dd unitPos = normalize(diff);
			return -scale*unitPos;
		}
		else
		{
			hduVector3Dd unitPos = normalize(diff);
			forceVec += -scale*unitPos / (dist*dist);
		}
	}
		forceVec *= charge;
		return forceVec;
	
}

/*******************************************************************************
 Main callback that calculates and sets the force.
*******************************************************************************/
HDCallbackCode HDCALLBACK CoulombCallback(void *data)
{
    HHD hHD = hdGetCurrentDevice();

    hdBeginFrame(hHD);

    hduVector3Dd pos;
    hdGetDoublev(HD_CURRENT_POSITION,pos);
    hduVector3Dd forceVec;
	forceVec = forceField(pos, Shape, Shape_size);
    hdSetDoublev(HD_CURRENT_FORCE, forceVec);
        
    hdEndFrame(hHD);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error during scheduler callback");
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Schedules the coulomb force callback.
*******************************************************************************/
void CoulombForceField()
{
    gSchedulerCallback = hdScheduleAsynchronous(
        CoulombCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }


    glutMainLoop(); // Enter GLUT main loop.
}

/******************************************************************************
 This handler gets called when the process is exiting. Ensures that HDAPI is
 properly shutdown
******************************************************************************/
void exitHandler()
{
    hdStopScheduler();
    hdUnschedule(gSchedulerCallback);

    if (ghHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(ghHD);
        ghHD = HD_INVALID_HANDLE;
    }
}









void parse(string fileName)
{
	   cout << "Parse called" << endl;
		ifstream stream1(fileName);		
		string line;
		double coords[3];
		cout << "At while" << endl;
		while (getline(stream1, line)) {
			Shape_size++;
		}
		Shape_size++;
		cout<<Shape_size << endl;
		Shape = new hduVector3Dd[Shape_size];

		ifstream stream(fileName);
		int j = 0;
		while (getline(stream, line)) {
			j++;
			size_t i = 2;

			for (int coord = 0; coord < 3; coord++) {
				//for each coordinate find corresponding substring n and convert to float
				string number;

				int count = 0;
				while (i < line.length() - 1 && line[i] != ' ') {
					i++;
					count++;
				}
				i++;
				count++;

				number = line.substr(i - count, count);
				coords[coord] = stod(number)*10;

			}
			hduVector3Dd vc(coords[0], coords[2]-35, coords[1]);
			Shape[j] = vc;
			

		}

		cout << "exited parse\n";
		return;
}

void print(hduVector3Dd* Shape, int Shape_size) {
	for (int i = 0; i < Shape_size; i++)
	{
		cout << "x = " << Shape[i][0] << " y = " << Shape[i][1] << " z = " << Shape[i][2] << "\n";
	}
}








/******************************************************************************
 Main function.
******************************************************************************/
int main(int argc, char* argv[])
{
    HDErrorInfo error;

    printf("Starting application\n");
    
    atexit(exitHandler);

	string file = "models/justcurves2.obj";

	glPointSize(10.0f);
	cout << "entering parse\n";
	parse(file);
	cout << "exiting parse\n";
	cout << "entering print\n";
	print(Shape, Shape_size);
	cout << "exiting print\n";

	glBegin(GL_POINTS);
	glVertex3f(2, 2, 0);//upper-right corner
	glEnd();


    // Initialize the device.  This needs to be called before any other
    // actions on the device are performed.
    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }

    printf("Found device %s\n",hdGetString(HD_DEVICE_MODEL_TYPE));
    
    hdEnable(HD_FORCE_OUTPUT);
    hdEnable(HD_MAX_FORCE_CLAMPING);

    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getchar();
        exit(-1);
    }
    
    initGlut(argc, argv);

    // Get the workspace dimensions.
    HDdouble maxWorkspace[6];
    hdGetDoublev(HD_MAX_WORKSPACE_DIMENSIONS, maxWorkspace);

    // Low/left/back point of device workspace.
    hduVector3Dd LLB(maxWorkspace[0], maxWorkspace[1], maxWorkspace[2]);
    // Top/right/front point of device workspace.
    hduVector3Dd TRF(maxWorkspace[3], maxWorkspace[4], maxWorkspace[5]);
    initGraphics(LLB, TRF);

    // Application loop.
    CoulombForceField();

	delete[] Shape;

    printf("Done\n");
    return 0;
}

/******************************************************************************/
