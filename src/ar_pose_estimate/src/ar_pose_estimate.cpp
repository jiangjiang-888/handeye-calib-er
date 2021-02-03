/*
 *  simpleLite.c
 *
 *  Some code to demonstrate use of ARToolKit.
 *
 *  Press '?' while running for help on available key commands.
 *
 *  Disclaimer: IMPORTANT:  This Daqri software is supplied to you by Daqri
 *  LLC ("Daqri") in consideration of your agreement to the following
 *  terms, and your use, installation, modification or redistribution of
 *  this Daqri software constitutes acceptance of these terms.  If you do
 *  not agree with these terms, please do not use, install, modify or
 *  redistribute this Daqri software.
 *
 *  In consideration of your agreement to abide by the following terms, and
 *  subject to these terms, Daqri grants you a personal, non-exclusive
 *  license, under Daqri's copyrights in this original Daqri software (the
 *  "Daqri Software"), to use, reproduce, modify and redistribute the Daqri
 *  Software, with or without modifications, in source and/or binary forms;
 *  provided that if you redistribute the Daqri Software in its entirety and
 *  without modifications, you must retain this notice and the following
 *  text and disclaimers in all such redistributions of the Daqri Software.
 *  Neither the name, trademarks, service marks or logos of Daqri LLC may
 *  be used to endorse or promote products derived from the Daqri Software
 *  without specific prior written permission from Daqri.  Except as
 *  expressly stated in this notice, no other rights or licenses, express or
 *  implied, are granted by Daqri herein, including but not limited to any
 *  patent rights that may be infringed by your derivative works or by other
 *  works in which the Daqri Software may be incorporated.
 *
 *  The Daqri Software is provided by Daqri on an "AS IS" basis.  DAQRI
 *  MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION
 *  THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE, REGARDING THE DAQRI SOFTWARE OR ITS USE AND
 *  OPERATION ALONE OR IN COMBINATION WITH YOUR PRODUCTS.
 *
 *  IN NO EVENT SHALL DAQRI BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL
 *  OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, REPRODUCTION,
 *  MODIFICATION AND/OR DISTRIBUTION OF THE DAQRI SOFTWARE, HOWEVER CAUSED
 *  AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE),
 *  STRICT LIABILITY OR OTHERWISE, EVEN IF DAQRI HAS BEEN ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Copyright 2015 Daqri LLC. All Rights Reserved.
 *  Copyright 2002-2015 ARToolworks, Inc. All Rights Reserved.
 *
 *  Author(s): Philip Lamb.
 *
 */

// ============================================================================
//	Includes
// ============================================================================

#include <stdio.h>
#include <string.h>
#ifdef _WIN32
#  define snprintf _snprintf
#  define _USE_MATH_DEFINES
#endif
#include <stdlib.h>					// malloc(), free()
#include <math.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include <AR/config.h>
#include <AR/video.h>
#include <AR/param.h>			// arParamDisp()
#include <AR/ar.h>
#include <AR/gsub_lite.h>
#include <ARUtil/time.h>

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include "common_pkg/task_cmd.h"
#include "common_pkg/ar_pose.h"
#include "common_pkg/ar_pose_info.h"

// ============================================================================
//	Constants
// ============================================================================

#define VIEW_SCALEFACTOR		1.0         // Units received from ARToolKit tracking will be multiplied by this factor before being used in OpenGL drawing.
#define VIEW_DISTANCE_MIN		40.0        // Objects closer to the camera than this will not be displayed. OpenGL units.
#define VIEW_DISTANCE_MAX		10000.0     // Objects further away from the camera than this will not be displayed. OpenGL units.

// ============================================================================
//	Global variables
// ============================================================================

// Preferences.
static int prefWindowed = TRUE;             // Use windowed (TRUE) or fullscreen mode (FALSE) on launch.
static int prefWidth = 640;					// Initial window width, also updated during program execution.
static int prefHeight = 480;                // Initial window height, also updated during program execution.
static int prefDepth = 32;					// Fullscreen mode bit depth.
static int prefRefresh = 0;					// Fullscreen mode refresh rate. Set to 0 to use default rate.

static int          gARTImageSavePlease = FALSE;

// Marker detection.
static ARHandle		*gARHandle = NULL;
static ARPattHandle	*gARPattHandle = NULL;
static long	        gCallCountMarkerDetect = 0;

// Transformation matrix retrieval.
static AR3DHandle	*gAR3DHandle = NULL;
static ARdouble		gPatt_width     = 150.0;	// Per-marker, but we are using only 1 marker.
static ARdouble		gPatt_trans[3][4];		// Per-marker, but we are using only 1 marker.
static int		gPatt_found = FALSE;	        // Per-marker, but we are using only 1 marker.
static int		gPatt_id;			// Per-marker, but we are using only 1 marker.

// Drawing.
static int gWindowW;
static int gWindowH;
static ARParamLT *gCparamLT = NULL;
static ARGL_CONTEXT_SETTINGS_REF gArglSettings = NULL;
static int gShowHelp = 1;
static int gShowMode = 1;
static int gDrawRotate = FALSE;
static float gDrawRotateAngle = 0;			// For use in drawing.


//param
static std::string g_camera_param;
static std::string g_patt_file;
static std::string g_video_conf;

static double g_patt_width;

static ros::Publisher g_pub_pose;

//const char* test_str = "hello world!";

// ============================================================================
//	Function prototypes.
// ============================================================================

static void print(const char *text, const float x, const float y, int calculateXFromRightEdge, int calculateYFromTopEdge);
static void drawBackground(const float width, const float height, const float x, const float y);
static void printHelpKeys();
static void printMode();

// ============================================================================
//	Functions
// ============================================================================

static double radian2Degree(double radian) {
  return radian / 3.1415926 * 180.0;
}

static double degree2Radian(double degree) {
  return degree / 180.0 * 3.1415926;
}

// Something to look at, draw a rotating colour cube.
static void DrawCube(void)
{
    // Colour cube data.
    int i;
	float fSize = 40.0f;
    const GLfloat cube_vertices [8][3] = {
        /* +z */ {0.5f, 0.5f, 0.5f}, {0.5f, -0.5f, 0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f},
        /* -z */ {0.5f, 0.5f, -0.5f}, {0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f} };
    const GLubyte cube_vertex_colors [8][4] = {
        {255, 255, 255, 255}, {255, 255, 0, 255}, {0, 255, 0, 255}, {0, 255, 255, 255},
        {255, 0, 255, 255}, {255, 0, 0, 255}, {0, 0, 0, 255}, {0, 0, 255, 255} };
    const GLubyte cube_faces [6][4] = { /* ccw-winding */
        /* +z */ {3, 2, 1, 0}, /* -y */ {2, 3, 7, 6}, /* +y */ {0, 1, 5, 4},
        /* -x */ {3, 0, 4, 7}, /* +x */ {1, 2, 6, 5}, /* -z */ {4, 5, 6, 7} };
      
    glPushMatrix(); // Save world coordinate system.
    glRotatef(gDrawRotateAngle, 0.0f, 0.0f, 1.0f); // Rotate about z axis.
    glScalef(fSize, fSize, fSize);
    glTranslatef(0.0f, 0.0f, 0.5f); // Place base of cube on marker surface.
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glColorPointer(4, GL_UNSIGNED_BYTE, 0, cube_vertex_colors);
    glVertexPointer(3, GL_FLOAT, 0, cube_vertices);
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_COLOR_ARRAY);
    for (i = 0; i < 6; i++) {
        glDrawElements(GL_TRIANGLE_FAN, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
    }
    glDisableClientState(GL_COLOR_ARRAY);
    glColor4ub(0, 0, 0, 255);
    for (i = 0; i < 6; i++) {
        glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_BYTE, &(cube_faces[i][0]));
    }
    glPopMatrix();    // Restore world coordinate system.
}

static void DrawCubeUpdate(float timeDelta)
{
	if (gDrawRotate) {
		gDrawRotateAngle += timeDelta * 45.0f; // Rotate cube at 45 degrees per second.
		if (gDrawRotateAngle > 360.0f) gDrawRotateAngle -= 360.0f;
	}
}

static void usage(char *com)
{
    ARLOG("Usage: %s [options]\n", com);
    ARLOG("Options:\n");
    ARLOG("  --vconf <video parameter for the camera>\n");
    ARLOG("  --cpara <camera parameter file for the camera>\n");
    ARLOG("  -cpara=<camera parameter file for the camera>\n");
    ARLOG("  --width w     Use display/window width of w pixels.\n");
    ARLOG("  --height h    Use display/window height of h pixels.\n");
    ARLOG("  --refresh f   Use display refresh rate of f Hz.\n");
    ARLOG("  --windowed    Display in window, rather than fullscreen.\n");
    ARLOG("  --fullscreen  Display fullscreen, rather than in window.\n");
    ARLOG("  -h -help --help: show this message\n");
    exit(0);
}

static int setupCamera(const char *cparam_name, const char *vconf, ARParamLT **cparamLT_p, ARHandle **arhandle, AR3DHandle **ar3dhandle)
{	
    ARParam			cparam;
    int				xsize, ysize;
    AR_PIXEL_FORMAT pixFormat;

    // Open the video path.
    if (arVideoOpen(vconf) < 0) {
    	ARLOGe("setupCamera(): Unable to open connection to camera.\n");
    	return (FALSE);
	}
	
    // Find the size of the window.
    if (arVideoGetSize(&xsize, &ysize) < 0) {
        ARLOGe("setupCamera(): Unable to determine camera frame size.\n");
        arVideoClose();
        return (FALSE);
    }
    ARLOGi("Camera image size (x,y) = (%d,%d)\n", xsize, ysize);
	
	// Get the format in which the camera is returning pixels.
	pixFormat = arVideoGetPixelFormat();
	if (pixFormat == AR_PIXEL_FORMAT_INVALID) {
    	ARLOGe("setupCamera(): Camera is using unsupported pixel format.\n");
        arVideoClose();
		return (FALSE);
	}
	
	// Load the camera parameters, resize for the window and init.
	if (cparam_name && *cparam_name) {
        if (arParamLoad(cparam_name, 1, &cparam) < 0) {
		    ARLOGe("setupCamera(): Error loading parameter file %s for camera.\n", cparam_name);
            arVideoClose();
            return (FALSE);
        }
    } else {
        arParamClearWithFOVy(&cparam, xsize, ysize, M_PI_4); // M_PI_4 radians = 45 degrees.
        ARLOGw("Using default camera parameters for %dx%d image size, 45 degrees vertical field-of-view.\n", xsize, ysize);
    }
    if (cparam.xsize != xsize || cparam.ysize != ysize) {
        ARLOGw("*** Camera Parameter resized from %d, %d. ***\n", cparam.xsize, cparam.ysize);
        arParamChangeSize(&cparam, xsize, ysize, &cparam);
    }
#ifdef DEBUG
    ARLOG("*** Camera Parameter ***\n");
    arParamDisp(&cparam);
#endif
    if ((*cparamLT_p = arParamLTCreate(&cparam, AR_PARAM_LT_DEFAULT_OFFSET)) == NULL) {
        ARLOGe("setupCamera(): Error: arParamLTCreate.\n");
        return (FALSE);
    }

    if ((*arhandle = arCreateHandle(*cparamLT_p)) == NULL) {
        ARLOGe("setupCamera(): Error: arCreateHandle.\n");
        return (FALSE);
    }
    if (arSetPixelFormat(*arhandle, pixFormat) < 0) {
        ARLOGe("setupCamera(): Error: arSetPixelFormat.\n");
        return (FALSE);
    }
	if (arSetDebugMode(*arhandle, AR_DEBUG_DISABLE) < 0) {
        ARLOGe("setupCamera(): Error: arSetDebugMode.\n");
        return (FALSE);
    }
	if ((*ar3dhandle = ar3DCreateHandle(&cparam)) == NULL) {
        ARLOGe("setupCamera(): Error: ar3DCreateHandle.\n");
        return (FALSE);
    }
	
	if (arVideoCapStart() != 0) {
    	ARLOGe("setupCamera(): Unable to begin camera data capture.\n");
		return (FALSE);		
	}
	
	return (TRUE);
}

static int setupMarker(const char *patt_name, int *patt_id, ARHandle *arhandle, ARPattHandle **pattHandle_p)
{	
    if ((*pattHandle_p = arPattCreateHandle()) == NULL) {
        ARLOGe("setupMarker(): Error: arPattCreateHandle.\n");
        return (FALSE);
    }
    
    // Loading only 1 pattern in this example.
    if ((*patt_id = arPattLoad(*pattHandle_p, patt_name)) < 0) {
      ARLOGe("setupMarker(): Error loading pattern file %s.\n", patt_name);
      arPattDeleteHandle(*pattHandle_p);
      return (FALSE);
    }
    
    arPattAttach(arhandle, *pattHandle_p);
	
    return (TRUE);
}

static void cleanup(void)
{
	arglCleanup(gArglSettings);
        gArglSettings = NULL;
	arPattDetach(gARHandle);
	arPattDeleteHandle(gARPattHandle);
	arVideoCapStop();
	ar3DDeleteHandle(&gAR3DHandle);
	arDeleteHandle(gARHandle);
        arParamLTFree(&gCparamLT);
	arVideoClose();
}

static void Keyboard(unsigned char key, int x, int y)
{
	int mode, threshChange = 0;
        AR_LABELING_THRESH_MODE modea;
	
	switch (key) {
		case 0x1B:						// Quit.
		case 'Q':
		case 'q':
			cleanup();
			exit(0);
			break;
		case ' ':
			gDrawRotate = !gDrawRotate;
			break;
		case 'X':
		case 'x':
            arGetImageProcMode(gARHandle, &mode);
            switch (mode) {
                case AR_IMAGE_PROC_FRAME_IMAGE:  mode = AR_IMAGE_PROC_FIELD_IMAGE; break;
                case AR_IMAGE_PROC_FIELD_IMAGE:
                default: mode = AR_IMAGE_PROC_FRAME_IMAGE; break;
            }
            arSetImageProcMode(gARHandle, mode);
			break;
		case 'C':
		case 'c':
			ARLOGe("*** Camera - %f (frame/sec)\n", (double)gCallCountMarkerDetect/arUtilTimer());
			gCallCountMarkerDetect = 0;
			arUtilTimerReset();
			break;
		case 'a':
		case 'A':
			arGetLabelingThreshMode(gARHandle, &modea);
            switch (modea) {
                case AR_LABELING_THRESH_MODE_MANUAL:        modea = AR_LABELING_THRESH_MODE_AUTO_MEDIAN; break;
                case AR_LABELING_THRESH_MODE_AUTO_MEDIAN:   modea = AR_LABELING_THRESH_MODE_AUTO_OTSU; break;
                case AR_LABELING_THRESH_MODE_AUTO_OTSU:     modea = AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE; break;
                case AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE: modea = AR_LABELING_THRESH_MODE_AUTO_BRACKETING; break;
                case AR_LABELING_THRESH_MODE_AUTO_BRACKETING:
                default: modea = AR_LABELING_THRESH_MODE_MANUAL; break;
            }
            arSetLabelingThreshMode(gARHandle, modea);
			break;
		case '-':
			threshChange = -5;
			break;
		case '+':
		case '=':
			threshChange = +5;
			break;
		case 'D':
		case 'd':
			arGetDebugMode(gARHandle, &mode);
			arSetDebugMode(gARHandle, !mode);
			break;
        case 's':
        case 'S':
            if (!gARTImageSavePlease) gARTImageSavePlease = TRUE;
            break;
		case '?':
		case '/':
            gShowHelp++;
            if (gShowHelp > 1) gShowHelp = 0;
			break;
        case 'm':
        case 'M':
            gShowMode = !gShowMode;
            break;
		default:
			break;
	}
	if (threshChange) {
		int threshhold;
		arGetLabelingThresh(gARHandle, &threshhold);
		threshhold += threshChange;
		if (threshhold < 0) threshhold = 0;
		if (threshhold > 255) threshhold = 255;
		arSetLabelingThresh(gARHandle, threshhold);
	}
}

static void transMat_process(double transMat[][4]) {
   //double transMat[3][4];
   //arUtilMatInv(iTransMat, transMat);

   float o_x = transMat[0][3];
   float o_y = transMat[1][3];
   float o_z = transMat[2][3];
   float x_x = transMat[0][0];
   float x_y = transMat[1][0];
   float x_z = transMat[2][0];
   //float dx = x_x - o_x;
   //float dy = x_y - o_y;
   float angle = atan2(x_y, x_x) / 3.1416f * 180.0f;

   static int cnt = 0;
   static double array[10][4];
   int index = cnt % 10;
   array[index][0] = o_x;
   array[index][1] = o_y;
   array[index][2] = o_z;
   array[index][3] = angle;
   ++cnt;
   cnt %= 10;

   int MAX_NUM = 10;
   float average_x = 0.0;
   float average_y = 0.0;
   float average_z = 0.0;
   float average_angle = 0.0;
   for (int i = 0; i < 10; i++) {
      average_x += array[i][0];
      average_y += array[i][1];
      average_z += array[i][2];
      average_angle += array[i][3];
   }   

   average_x /= MAX_NUM;
   average_y /= MAX_NUM;
   average_z /= MAX_NUM;
   average_angle /= MAX_NUM;
   
   ARLOG("x = %5.1f, y = %5.1f, z = %5.1f, angle = %5.1f\n", average_x, average_y, average_z, average_angle);
}

//static void publish_transofrm(double transmat[][4]) {
//  static tf::Transform tf_marker_to_camera;
//  const double millimeter_to_meter = 0.001;
//  tf_marker_to_camera.setOrigin(tf::Vector3(transmat[0][3] * millimeter_to_meter, transmat[1][3] * millimeter_to_meter, transmat[2][3] * millimeter_to_meter));
//  tf_marker_to_camera.setBasis(tf::Matrix3x3(transmat[0][0], transmat[0][1], transmat[0][2],\
//      transmat[1][0], transmat[1][1], transmat[1][2],\
//      transmat[2][0], transmat[2][1], transmat[2][2]));
//  static tf::TransformBroadcaster br;
//  br.sendTransform(tf::StampedTransform(tf_marker_to_camera, ros::Time::now(), "/camera1_link", "/marker_link"));
//}

static void transform_to_pose(const tf::Transform& tf, geometry_msgs::Pose& pose) {
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  pose.orientation.x = tf.getRotation().getX();
  pose.orientation.y = tf.getRotation().getY();
  pose.orientation.z = tf.getRotation().getZ();
  pose.orientation.w = tf.getRotation().getW();
}

static void publish_transofrm(double transmat[][4]) {
  static tf::Transform tf_marker_to_camera;
  const double millimeter_to_meter = 0.001;
  tf_marker_to_camera.setOrigin(tf::Vector3(transmat[0][3] * millimeter_to_meter, transmat[1][3] * millimeter_to_meter, transmat[2][3] * millimeter_to_meter));
  tf_marker_to_camera.setBasis(tf::Matrix3x3(transmat[0][0], transmat[0][1], transmat[0][2],\
      transmat[1][0], transmat[1][1], transmat[1][2],\
      transmat[2][0], transmat[2][1], transmat[2][2]));

//  double x = transmat[0][3] * millimeter_to_meter;
//  double y = transmat[1][3] * millimeter_to_meter;
//  double z = transmat[2][3] * millimeter_to_meter;

//  static tf::Transform tf_camera_to_marker;
//  tf_camera_to_marker =  tf_marker_to_camera.inverse();

//  double roll, pitch, yaw;
//  tf_marker_to_camera.getBasis().getRPY(roll, pitch, yaw);

//  tf_camera_to_marker.getBasis().getRPY(roll, pitch, yaw);

//  double x = tf_camera_to_marker.getOrigin().getX();
//  double y = tf_camera_to_marker.getOrigin().getY();
//  double z = tf_camera_to_marker.getOrigin().getZ();

  double x, y, z;
  double roll, pitch, yaw;

  x = tf_marker_to_camera.getOrigin().getX();
  y = tf_marker_to_camera.getOrigin().getY();
  z = tf_marker_to_camera.getOrigin().getZ();

  tf_marker_to_camera.getBasis().getRPY(roll, pitch, yaw);

//  roll = radian2Degree(roll);
//  pitch = radian2Degree(pitch);
//  yaw = radian2Degree(yaw);

//  roll = roll / 3.14159 * 180.0;
//  pitch = pitch / 3.14159 * 180.0;
//  yaw = yaw / 3.14159 * 180.0;

  geometry_msgs::Pose pose_marker_to_camera;
  transform_to_pose(tf_marker_to_camera, pose_marker_to_camera);

  g_pub_pose.publish(pose_marker_to_camera);

  printf("marker to camera, x = %4.3f y = %4.3f z = %4.3f roll = %3.2f pitch = %3.2f yaw = %3.2f\n",\
         x, y, z, radian2Degree(roll), radian2Degree(pitch), radian2Degree(yaw));
//  std::cout << "x:" << x  << " y:" << y << " z:" << z << " row:" << row << " pitch_angle:" << pitch << " yaw_angle:" << yaw << std::endl;



  static tf::TransformBroadcaster br;
//  br.sendTransform(tf::StampedTransform(tf_marker_to_camera, ros::Time::now(), "/camera_measure_link", "/marker_measure_link"));
  br.sendTransform(tf::StampedTransform(tf_marker_to_camera, ros::Time::now(), "/camera_measure_link", "/marker_measure_link"));
}

//static void copy_transmat(ARdouble transmat_src[][4], double transmat_dst[][4]) {
//  for (int i = 0; i < 3; i++) {
//    for (int j = 0; j < 4; j++) {
//      transmat_dst[i][j] = transmat_src[i][j];
//    }
//  }
//}

//static int transfer_data(Marker_Data marker_data) {
//  //static bool first = true;
//  if (send(g_sockfd, (char*)&marker_data, sizeof (Marker_Data), 0) < 0) {
//      printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
//      return 0;
//  }
//  return 1;
//}

//static void send_transmat(ARdouble gPatt_trans[][4], bool have_marker_data) {
//  static Marker_Data marker_data;
//  marker_data.have_marker_data = have_marker_data;
//  if (have_marker_data) {
//    copy_transmat(gPatt_trans, marker_data.marker_data);
//  }
//  transfer_data(marker_data);
//}

static void mainLoop(void) {
  static int imageNumber = 0;
	static int ms_prev;
	int ms;
	float s_elapsed;
	AR2VideoBufferT *image;
	ARdouble err;
  int j, k;

  static ros::Rate loop_rate(30.0);
	
	// Find out how long since mainLoop() last ran.
	ms = glutGet(GLUT_ELAPSED_TIME);
	s_elapsed = (float)(ms - ms_prev) * 0.001f;
	if (s_elapsed < 0.01f) return; // Don't update more often than 100 Hz.
	ms_prev = ms;
	
	// Update drawing.
	DrawCubeUpdate(s_elapsed);
	
	// Grab a video frame.
	if ((image = arVideoGetImage()) != NULL) {
        
        arglPixelBufferDataUpload(gArglSettings, image->buff);

        if (gARTImageSavePlease) {
            char imageNumberText[15];
            sprintf(imageNumberText, "image-%04d.jpg", imageNumber++);
            if (arVideoSaveImageJPEG(gARHandle->xsize, gARHandle->ysize, gARHandle->arPixelFormat, image->buff, imageNumberText, 75, 0) < 0) {
                ARLOGe("Error saving video image.\n");
            }
            gARTImageSavePlease = FALSE;
        }
		
		gCallCountMarkerDetect++; // Increment ARToolKit FPS counter.
		
		// Detect the markers in the video frame.
		if (arDetectMarker(gARHandle, image) < 0) {
			exit(-1);
		}
		
		// Check through the marker_info array for highest confidence
		// visible marker matching our preferred pattern.
		k = -1;
		for (j = 0; j < gARHandle->marker_num; j++) {
			if (gARHandle->markerInfo[j].id == gPatt_id) {
				if (k == -1) k = j; // First marker detected.
				else if (gARHandle->markerInfo[j].cf > gARHandle->markerInfo[k].cf) k = j; // Higher confidence marker detected.
			}
		}
		
		if (k != -1) {
			// Get the transformation between the marker and the real camera into gPatt_trans.
			err = arGetTransMatSquare(gAR3DHandle, &(gARHandle->markerInfo[k]), gPatt_width, gPatt_trans);
			gPatt_found = TRUE;
		} else {
			gPatt_found = FALSE;
		}

    if (gPatt_found)
      publish_transofrm(gPatt_trans);

//                //output
//                if (gPatt_found) {
//                  transMat_process(gPatt_trans);
//                }

//                //transfer marker data through TCP
//                send_transmat(gPatt_trans, gPatt_found);
		
		// Tell GLUT the display has changed.
		glutPostRedisplay();
	}

  ros::spinOnce();
  loop_rate.sleep();
}

//
//	This function is called on events when the visibility of the
//	GLUT window changes (including when it first becomes visible).
//
static void Visibility(int visible)
{
	if (visible == GLUT_VISIBLE) {
		glutIdleFunc(mainLoop);
	} else {
		glutIdleFunc(NULL);
	}
}

//
//	This function is called when the
//	GLUT window is resized.
//
static void Reshape(int w, int h)
{
    gWindowW = w;
    gWindowH = h;
    
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	
	// Call through to anyone else who needs to know about window sizing here.
}

//
// This function is called when the window needs redrawing.
//
static void Display(void)
{
  ARdouble p[16];
	ARdouble m[16];
	
	// Select correct buffer for this context.
	glDrawBuffer(GL_BACK);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers for new frame.
	
	arglDispImage(gArglSettings);
				
	// Projection transformation.
	arglCameraFrustumRH(&(gCparamLT->param), VIEW_DISTANCE_MIN, VIEW_DISTANCE_MAX, p);
	glMatrixMode(GL_PROJECTION);

#ifdef ARDOUBLE_IS_FLOAT
    glLoadMatrixf(p);
#else
    glLoadMatrixd(p);
#endif

	glMatrixMode(GL_MODELVIEW);
		
	glEnable(GL_DEPTH_TEST);

	// Viewing transformation.
	glLoadIdentity();
	// Lighting and geometry that moves with the camera should go here.
	// (I.e. must be specified before viewing transformations.)
	//none
	
	if (gPatt_found) {
	
		// Calculate the camera position relative to the marker.
		// Replace VIEW_SCALEFACTOR with 1.0 to make one drawing unit equal to 1.0 ARToolKit units (usually millimeters).
		arglCameraViewRH((const ARdouble (*)[4])gPatt_trans, m, VIEW_SCALEFACTOR);
#ifdef ARDOUBLE_IS_FLOAT
        glLoadMatrixf(m);
#else
        glLoadMatrixd(m);
#endif

		// All lighting and geometry to be drawn relative to the marker goes here.
		DrawCube();
	
	} // gPatt_found
	
	// Any 2D overlays go here.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, (GLdouble)gWindowW, 0, (GLdouble)gWindowH, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);

    //
    // Draw help text and mode.
    //
    if (gShowMode) {
        printMode();
    }
    if (gShowHelp) {
        if (gShowHelp == 1) {
            printHelpKeys();
        }
    }
	
	glutSwapBuffers();
}

static void init_param() {
  if (!ros::param::get("/ar_pose_estimate/camera_param", g_camera_param)) {
    ROS_ERROR("get camera param file failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/patt_file", g_patt_file)) {
    ROS_ERROR("get pattern file failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/patt_width", g_patt_width)) {
    ROS_ERROR("get patt width failed!");
  }

  gPatt_width = g_patt_width;

  if (!ros::param::get("/ar_pose_estimate/video_conf", g_video_conf)) {
    ROS_ERROR("get video conf failed!");
  }
}

int main_api(int argc, char** argv)
{  
    char  glutGamemode[32] = "";
    char  *vconf = NULL;
    char  *cparam_name = NULL;
    int   i;
    int   gotTwoPartOption;

    init_param();

//    vconf = g_video_conf.c_str();
//    cparam_name = g_camera_param.c_str();

    glutInit(&argc, argv);
    arUtilChangeToResourcesDirectory(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_BEST, NULL);

    std::cout << "argv:" << argv << std::endl;
    
//    i = 1; // argv[0] is name of app, so start at 1.
//    while (i < argc) {
//        gotTwoPartOption = FALSE;
//        // Look for two-part options first.
//        if ((i + 1) < argc) {
//            if (strcmp(argv[i], "--vconf") == 0) {
//                i++;
//                vconf = argv[i];
//                gotTwoPartOption = TRUE;
//            } else if (strcmp(argv[i], "--cpara") == 0) {
//                i++;
//                cparam_name = argv[i];
//                gotTwoPartOption = TRUE;
//            } else if (strcmp(argv[i],"--width") == 0) {
//                i++;
//                // Get width from second field.
//                if (sscanf(argv[i], "%d", &prefWidth) != 1) {
//                    ARLOGe("Error: --width option must be followed by desired width.\n");
//                }
//                gotTwoPartOption = TRUE;
//            } else if (strcmp(argv[i],"--height") == 0) {
//                i++;
//                // Get height from second field.
//                if (sscanf(argv[i], "%d", &prefHeight) != 1) {
//                    ARLOGe("Error: --height option must be followed by desired height.\n");
//                }
//                gotTwoPartOption = TRUE;
//            } else if (strcmp(argv[i],"--refresh") == 0) {
//                i++;
//                // Get refresh rate from second field.
//                if (sscanf(argv[i], "%d", &prefRefresh) != 1) {
//                    ARLOGe("Error: --refresh option must be followed by desired refresh rate.\n");
//                }
//                gotTwoPartOption = TRUE;
//            }
//        }
//        if (!gotTwoPartOption) {
//            // Look for single-part options.
//            if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "-h") == 0) {
//                usage(argv[0]);
//            } else if (strncmp(argv[i], "-cpara=", 7) == 0) {
//                cparam_name = &(argv[i][7]);
//            } else if (strcmp(argv[i], "--version") == 0 || strcmp(argv[i], "-version") == 0 || strcmp(argv[i], "-v") == 0) {
//                ARLOG("%s version %s\n", argv[0], AR_HEADER_VERSION_STRING);
//                exit(0);
//            } else if (strcmp(argv[i],"--windowed") == 0) {
//                prefWindowed = TRUE;
//            } else if (strcmp(argv[i],"--fullscreen") == 0) {
//                prefWindowed = FALSE;
//            } else {
//                ARLOGe("Error: invalid command line argument '%s'.\n", argv[i]);
//                usage(argv[0]);
//            }
//        }
//        i++;
//    }
    
	//
	// Video setup.
	//

        //ARLOG("cparam_name:%s\n", cparam_name);
//   std::string camera_param;
//   if (!ros::param::get("/ar_pose_estimate/camera_param", camera_param)) {
//     std::cout << "get camera param failed!" << std::endl;
//     return -1;
//   } else {
//     std::cout << "camera param path:" << camera_param << std::endl;
//   }
   //     cparam_name = "Data/camera_para.dat";

  if (!setupCamera(g_camera_param.c_str(), g_video_conf.c_str(), &gCparamLT, &gARHandle, &gAR3DHandle)) {
		ARLOGe("main(): Unable to set up AR camera.\n");
		exit(-1);
	}

	//
	// Graphics setup.
	//

	// Set up GL context(s) for OpenGL to draw into.
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	if (!prefWindowed) {
		if (prefRefresh) sprintf(glutGamemode, "%ix%i:%i@%i", prefWidth, prefHeight, prefDepth, prefRefresh);
		else sprintf(glutGamemode, "%ix%i:%i", prefWidth, prefHeight, prefDepth);
		glutGameModeString(glutGamemode);
		glutEnterGameMode();
	} else {
		glutInitWindowSize(prefRefresh, prefHeight);
		glutCreateWindow(argv[0]);
	}

	// Setup ARgsub_lite library for current OpenGL context.
	if ((gArglSettings = arglSetupForCurrentContext(&(gCparamLT->param), arVideoGetPixelFormat())) == NULL) {
		ARLOGe("main(): arglSetupForCurrentContext() returned error.\n");
		cleanup();
		exit(-1);
	}
  arglSetupDebugMode(gArglSettings, gARHandle);
	arUtilTimerReset();
	
	// Load marker(s).
//  std::string pattern_file;
//  if (!ros::param::get("/ar_pose_estimate/pattern_file", pattern_file)) {
//    std::cout << "get pattern file failed!" << std::endl;
//    return -2;
//  } else {
//    std::cout << "pattern file:" << pattern_file << std::endl;
//  }

  if (!setupMarker(g_patt_file.c_str(), &gPatt_id, gARHandle, &gARPattHandle)) {
		ARLOGe("main(): Unable to set up AR marker.\n");
		cleanup();
		exit(-1);
	}
	
	// Register GLUT event-handling callbacks.
	// NB: mainLoop() is registered by Visibility.
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutVisibilityFunc(Visibility);
	glutKeyboardFunc(Keyboard);

	glutMainLoop();

	return (0);
}

//
// The following functions provide the onscreen help text and mode info.
//

static void print(const char *text, const float x, const float y, int calculateXFromRightEdge, int calculateYFromTopEdge)
{
    int i, len;
    GLfloat x0, y0;
    
    if (!text) return;
    
    if (calculateXFromRightEdge) {
        x0 = gWindowW - x - (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (const unsigned char *)text);
    } else {
        x0 = x;
    }
    if (calculateYFromTopEdge) {
        y0 = gWindowH - y - 10.0f;
    } else {
        y0 = y;
    }
    glRasterPos2f(x0, y0);
    
    len = (int)strlen(text);
    for (i = 0; i < len; i++) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
}

static void drawBackground(const float width, const float height, const float x, const float y)
{
    GLfloat vertices[4][2];
    
    vertices[0][0] = x; vertices[0][1] = y;
    vertices[1][0] = width + x; vertices[1][1] = y;
    vertices[2][0] = width + x; vertices[2][1] = height + y;
    vertices[3][0] = x; vertices[3][1] = height + y;
    glLoadIdentity();
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glVertexPointer(2, GL_FLOAT, 0, vertices);
    glEnableClientState(GL_VERTEX_ARRAY);
    glColor4f(0.0f, 0.0f, 0.0f, 0.5f);	// 50% transparent black.
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Opaque white.
    //glLineWidth(1.0f);
    //glDrawArrays(GL_LINE_LOOP, 0, 4);
    glDisableClientState(GL_VERTEX_ARRAY);
    glDisable(GL_BLEND);
}

static void printHelpKeys()
{
    int i;
    GLfloat  w, bw, bh;
    const char *helpText[] = {
        "Keys:\n",
        " ? or /        Show/hide this help.",
        " q or [esc]    Quit program.",
        " d             Activate / deactivate debug mode.",
        " m             Toggle display of mode info.",
        " a             Toggle between available threshold modes.",
        " - and +       Switch to manual threshold mode, and adjust threshhold up/down by 5.",
        " x             Change image processing mode.",
        " c             Calulcate frame rate.",
    };
#define helpTextLineCount (sizeof(helpText)/sizeof(char *))
    
    bw = 0.0f;
    for (i = 0; i < helpTextLineCount; i++) {
        w = (float)glutBitmapLength(GLUT_BITMAP_HELVETICA_10, (unsigned char *)helpText[i]);
        if (w > bw) bw = w;
    }
    bh = helpTextLineCount * 10.0f /* character height */+ (helpTextLineCount - 1) * 2.0f /* line spacing */;
    drawBackground(bw, bh, 2.0f, 2.0f);
    
    for (i = 0; i < helpTextLineCount; i++) print(helpText[i], 2.0f, (helpTextLineCount - 1 - i)*12.0f + 2.0f, 0, 0);;
}

static void printMode()
{
    int len, thresh, line, mode, xsize, ysize;
    AR_LABELING_THRESH_MODE threshMode;
    ARdouble tempF;
    char text[256], *text_p;

    glColor3ub(255, 255, 255);
    line = 1;
    
    // Image size and processing mode.
    arVideoGetSize(&xsize, &ysize);
    arGetImageProcMode(gARHandle, &mode);
	if (mode == AR_IMAGE_PROC_FRAME_IMAGE) text_p = "full frame";
	else text_p = "even field only";
    snprintf(text, sizeof(text), "Processing %dx%d video frames %s", xsize, ysize, text_p);
    print(text, 2.0f,  (line - 1)*12.0f + 2.0f, 0, 1);
    line++;
    
    // Threshold mode, and threshold, if applicable.
    arGetLabelingThreshMode(gARHandle, &threshMode);
    switch (threshMode) {
        case AR_LABELING_THRESH_MODE_MANUAL: text_p = "MANUAL"; break;
        case AR_LABELING_THRESH_MODE_AUTO_MEDIAN: text_p = "AUTO_MEDIAN"; break;
        case AR_LABELING_THRESH_MODE_AUTO_OTSU: text_p = "AUTO_OTSU"; break;
        case AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE: text_p = "AUTO_ADAPTIVE"; break;
        case AR_LABELING_THRESH_MODE_AUTO_BRACKETING: text_p = "AUTO_BRACKETING"; break;
        default: text_p = "UNKNOWN"; break;
    }
    snprintf(text, sizeof(text), "Threshold mode: %s", text_p);
    if (threshMode != AR_LABELING_THRESH_MODE_AUTO_ADAPTIVE) {
        arGetLabelingThresh(gARHandle, &thresh);
        len = (int)strlen(text);
        snprintf(text + len, sizeof(text) - len, ", thresh=%d", thresh);
    }
    print(text, 2.0f,  (line - 1)*12.0f + 2.0f, 0, 1);
    line++;
    
    // Border size, image processing mode, pattern detection mode.
    arGetBorderSize(gARHandle, &tempF);
    snprintf(text, sizeof(text), "Border: %0.1f%%", tempF*100.0);
    arGetPatternDetectionMode(gARHandle, &mode);
    switch (mode) {
        case AR_TEMPLATE_MATCHING_COLOR: text_p = "Colour template (pattern)"; break;
        case AR_TEMPLATE_MATCHING_MONO: text_p = "Mono template (pattern)"; break;
        case AR_MATRIX_CODE_DETECTION: text_p = "Matrix (barcode)"; break;
        case AR_TEMPLATE_MATCHING_COLOR_AND_MATRIX: text_p = "Colour template + Matrix (2 pass, pattern + barcode)"; break;
        case AR_TEMPLATE_MATCHING_MONO_AND_MATRIX: text_p = "Mono template + Matrix (2 pass, pattern + barcode "; break;
        default: text_p = "UNKNOWN"; break;
    }
    len = (int)strlen(text);
    snprintf(text + len, sizeof(text) - len, ", Pattern detection mode: %s", text_p);
    print(text, 2.0f,  (line - 1)*12.0f + 2.0f, 0, 1);
    line++;
    
    // Window size.
    snprintf(text, sizeof(text), "Drawing into %dx%d window", gWindowW, gWindowH);
    print(text, 2.0f,  (line - 1)*12.0f + 2.0f, 0, 1);
    line++;
}

//camera_in_base_link
static void get_tf_camera_in_base(tf::Transform& tf_camera_in_base) {
  double camera_in_base_x;
  double camera_in_base_y;
  double camera_in_base_z;

  double camera_in_base_roll;
  double camera_in_base_pitch;
  double camera_in_base_yaw;

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_x", camera_in_base_x)) {
    ROS_ERROR("get param camera in base x failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_y", camera_in_base_y)) {
    ROS_ERROR("get param camera in base y failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_z", camera_in_base_z)) {
    ROS_ERROR("get param camera in base z failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_roll", camera_in_base_roll)) {
    ROS_ERROR("get param camera in base roll failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_pitch", camera_in_base_pitch)) {
    ROS_ERROR("get param camera in base pitch failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_yaw", camera_in_base_yaw)) {
    ROS_ERROR("get param camera in base yaw failed!");
  }

//  tf::Transform tf_camera_in_base;
  tf_camera_in_base.setOrigin(tf::Vector3(camera_in_base_x, camera_in_base_y, camera_in_base_z));
  tf::Quaternion q;
  q.setRPY(camera_in_base_roll / 180.0 * 3.1415926, camera_in_base_pitch / 180.0 * 3.1415926, camera_in_base_yaw / 180.0 * 3.1415926);
  tf_camera_in_base.setRotation(q);
}

static void get_tf_target_in_base(tf::Transform& tf_target_in_base) {
  double target_in_base_x;
  double target_in_base_y;
  double target_in_base_z;

  double target_in_base_roll;
  double target_in_base_pitch;
  double target_in_base_yaw;

  if (!ros::param::get("/ar_pose_estimate/target_in_base_x", target_in_base_x)) {
    ROS_ERROR("get param camera in base x failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_y", target_in_base_y)) {
    ROS_ERROR("get param camera in base y failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_z", target_in_base_z)) {
    ROS_ERROR("get param camera in base z failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_roll", target_in_base_roll)) {
    ROS_ERROR("get param camera in base roll failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_pitch", target_in_base_pitch)) {
    ROS_ERROR("get param camera in base pitch failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_yaw", target_in_base_yaw)) {
    ROS_ERROR("get param camera in base yaw failed!");
  }

  tf_target_in_base.setOrigin(tf::Vector3(target_in_base_x, target_in_base_y, target_in_base_z));
  tf::Quaternion q;
  q.setRPY(target_in_base_roll / 180.0 * 3.1415926, target_in_base_pitch / 180.0 * 3.1415926, target_in_base_yaw / 180.0 * 3.1415926);
  tf_target_in_base.setRotation(q);
}

static void pub_static_transform(tf::Transform& transform, const std::string& parent_frame, const std::string& child_frame) {
  static tf2_ros::StaticTransformBroadcaster br;
  static geometry_msgs::TransformStamped transform_stamped;

  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = parent_frame;
  transform_stamped.child_frame_id = child_frame;

  transform_stamped.transform.translation.x = transform.getOrigin().getX();
  transform_stamped.transform.translation.y = transform.getOrigin().getY();
  transform_stamped.transform.translation.z = transform.getOrigin().getZ();

  transform_stamped.transform.rotation.x = transform.getRotation().getX();
  transform_stamped.transform.rotation.y = transform.getRotation().getY();
  transform_stamped.transform.rotation.z = transform.getRotation().getZ();
  transform_stamped.transform.rotation.w = transform.getRotation().getW();

  br.sendTransform(transform_stamped);
}

//static void transform_to_pose(const tf::Transform& tf, geometry_msgs::Pose& pose) {
//  pose.position.x = tf.getOrigin().x();
//  pose.position.y = tf.getOrigin().y();
//  pose.position.z = tf.getOrigin().z();
//  pose.orientation.x = tf.getRotation().getX();
//  pose.orientation.y = tf.getRotation().getY();
//  pose.orientation.z = tf.getRotation().getZ();
//  pose.orientation.w = tf.getRotation().getW();
//}

static void init_transform() {
  //pub camera transform
  tf::Transform tf_camera_in_base;
  get_tf_camera_in_base(tf_camera_in_base);
  std::string parent_frame = "/base_link";
  std::string child_frame = "/camera_measure_link";
  pub_static_transform(tf_camera_in_base, parent_frame, child_frame);

  //pub target transform
  tf::Transform tf_target_in_base;
  parent_frame = "/base_link";
  child_frame = "/target_measure_link";
  get_tf_target_in_base(tf_target_in_base);
  pub_static_transform(tf_target_in_base, parent_frame, child_frame);
  return ;
}

bool get_target_in_marker(geometry_msgs::Pose& target_in_marker) {
  static tf::TransformListener listener;
  tf::StampedTransform tf_target_to_marker;

  listener.waitForTransform("/marker_meausre_link", "/target_measure_link", ros::Time::now(), ros::Duration(0.5));
  int cnt = 3;
  while (cnt--) {
    try {
      listener.lookupTransform("/marker_measure_link", "/target_measure_link", ros::Time(0), tf_target_to_marker);
      break;
    } catch(tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      if (cnt == 0) {
        std::cout << "max times tried, can't get target in marker!" << std::endl;
        return false;
      }
      ros::Duration(0.5).sleep();
    }
  }

  transform_to_pose(tf_target_to_marker, target_in_marker);
  return true;
}

//static void get_target_pose_in_marker(geometry_msgs::Pose& pose) {
//  std::string frame_marker = "";
//  std::string frame_target = "";
//}

#include <fstream>

static bool record_target_pose_(int index, geometry_msgs::Pose& pose) {
  geometry_msgs::Pose target_in_marker;
  if (!get_target_in_marker(target_in_marker)) {
    ROS_ERROR("get target pose in marker failed!");
    return false;
  }

  pose = target_in_marker;

  tf::Matrix3x3 mat;
  mat.setRotation(tf::Quaternion(target_in_marker.orientation.x, target_in_marker.orientation.y, target_in_marker.orientation.z, target_in_marker.orientation.w));

  double roll, pitch, yaw;
  mat.getRPY(roll, pitch, yaw);

  roll = radian2Degree(roll);
  pitch = radian2Degree(pitch);
  yaw = radian2Degree(yaw);

  double x, y, z;
  x = target_in_marker.position.x;
  y = target_in_marker.position.y;
  z = target_in_marker.position.z;

  printf("\n-------------------\n");
  printf("target in marker, x = %4.3f y = %4.3f z = %4.3f roll = %3.2f pitch = %3.2f yaw = %3.2f\n", x, y, z, roll, pitch, yaw);
  printf("\n-------------------\n");

  bool record_target_pose;
  if (!ros::param::get("/ar_pose_estimate/record_target_pose", record_target_pose)) {
    ROS_ERROR("get param record target pose failed!");
    return false;
  }

  #if 1
      std::string file_path;
      if (ros::param::get("/ar_pose_estimate/save_file_path", file_path)) {
         std::ofstream fout(file_path.c_str(), std::ios::app);
         fout << "index:" << index << " x:" << target_in_marker.position.x << " y:" << target_in_marker.position.y \
              << " z:" << target_in_marker.position.z << " roll:" << roll << " pitch:" << pitch << " yaw:" << yaw << std::endl;
         fout.close();
      } else {
        std::cout << "get save file path failed!" << std::endl;
      }
  #endif
  return true;
}

// camera in marker
// marker in base link

bool record_target_pose(common_pkg::ar_pose_infoRequest& req, common_pkg::ar_pose_infoResponse& res) {
  res.success = record_target_pose_(req.cmd, res.pose);
  return res.success;
}

//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
void sigint_handler(int) {
  ROS_INFO("shutting down!");
  ros::shutdown();
}

//for calibrate

static void init_marker_in_base() {
  double marker_in_base_x = 0.0;
  double marker_in_base_y = 0.0;
  double marker_in_base_z = 0.0;

  double marker_in_base_roll = 0.0;
  double marker_in_base_pitch = 0.0;
  double marker_in_base_yaw = 0.0;

  if (!ros::param::get("/ar_pose_estimate/marker_in_base_x", marker_in_base_x)) {
    ROS_ERROR("get param marker_in_base_x failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/marker_in_base_y", marker_in_base_y)) {
    ROS_ERROR("get param marker_in_base_y failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/marker_in_base_z", marker_in_base_z)) {
    ROS_ERROR("get param marker_in_base_z failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/marker_in_base_roll", marker_in_base_roll)) {
    ROS_ERROR("get param marker_in_base_roll failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/marker_in_base_pitch", marker_in_base_pitch)) {
    ROS_ERROR("get param camera in marker pitch failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/marker_in_base_yaw", marker_in_base_yaw)) {
    ROS_ERROR("get param camera in marker yaw failed!");
  }

  std::string parent_frame_2 = "/base_link";
  std::string child_frame_2 = "/marker_measure_link";

  tf::Transform tf_marker_to_base;
  tf_marker_to_base.setOrigin(tf::Vector3(marker_in_base_x, marker_in_base_y, marker_in_base_z));
  tf::Quaternion q_2;
  q_2.setRPY( degree2Radian(marker_in_base_roll), degree2Radian(marker_in_base_pitch), degree2Radian(marker_in_base_yaw));
  tf_marker_to_base.setRotation(q_2);
  pub_static_transform(tf_marker_to_base, parent_frame_2, child_frame_2);
}

static void init_target_in_base() {
  double marker_in_base_x = 0.0;
  double marker_in_base_y = 0.0;
  double marker_in_base_z = 0.0;

  double marker_in_base_roll = 0.0;
  double marker_in_base_pitch = 0.0;
  double marker_in_base_yaw = 0.0;

  if (!ros::param::get("/ar_pose_estimate/target_in_base_x", marker_in_base_x)) {
    ROS_ERROR("get param target_in_base_x failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_y", marker_in_base_y)) {
    ROS_ERROR("get param target_in_base_y failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_z", marker_in_base_z)) {
    ROS_ERROR("get param target_in_base_z failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_roll", marker_in_base_roll)) {
    ROS_ERROR("get param target_in_base_roll failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_pitch", marker_in_base_pitch)) {
    ROS_ERROR("get param camera in marker pitch failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/target_in_base_yaw", marker_in_base_yaw)) {
    ROS_ERROR("get param camera in marker yaw failed!");
  }

  std::string parent_frame_2 = "/base_link";
  std::string child_frame_2 = "/target_measure_link";

  tf::Transform tf_marker_to_base;
  tf_marker_to_base.setOrigin(tf::Vector3(marker_in_base_x, marker_in_base_y, marker_in_base_z));
  tf::Quaternion q_2;
  q_2.setRPY( degree2Radian(marker_in_base_roll), degree2Radian(marker_in_base_pitch), degree2Radian(marker_in_base_yaw));
  tf_marker_to_base.setRotation(q_2);
  pub_static_transform(tf_marker_to_base, parent_frame_2, child_frame_2);
}

static void init_camera_in_base() {
  double camera_in_base_x = 0.0;
  double camera_in_base_y = 0.0;
  double camera_in_base_z = 0.0;

  double camera_in_base_roll = 0.0;
  double camera_in_base_pitch = 0.0;
  double camera_in_base_yaw = 0.0;

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_x", camera_in_base_x)) {
    ROS_ERROR("get param camera_in_base_x failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_y", camera_in_base_y)) {
    ROS_ERROR("get param camera_in_base_y failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_z", camera_in_base_z)) {
    ROS_ERROR("get param camera_in_base_z failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_roll", camera_in_base_roll)) {
    ROS_ERROR("get param camera_in_base_roll failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_pitch", camera_in_base_pitch)) {
    ROS_ERROR("get param camera_in_base_pitch failed!");
  }

  if (!ros::param::get("/ar_pose_estimate/camera_in_base_yaw", camera_in_base_yaw)) {
    ROS_ERROR("get param camera_in_base_yaw failed!");
  }

  std::string parent_frame_2 = "/base_link";
  std::string child_frame_2 = "/camera_measure_link";

  tf::Transform tf_marker_to_base;
  tf_marker_to_base.setOrigin(tf::Vector3(camera_in_base_x, camera_in_base_y, camera_in_base_z));
  tf::Quaternion q_2;
  q_2.setRPY( degree2Radian(camera_in_base_roll), degree2Radian(camera_in_base_pitch), degree2Radian(camera_in_base_yaw));
  tf_marker_to_base.setRotation(q_2);
  pub_static_transform(tf_marker_to_base, parent_frame_2, child_frame_2);
}

bool calibrate_camera_in_base() {
  //-----------------------
  static tf::TransformListener listener;
  tf::StampedTransform tf_camera_to_base;

  listener.waitForTransform("/base_link", "/camera_measure_link", ros::Time::now(), ros::Duration(2.0));

  int cnt = 3;
  while (cnt--) {
    try {
      listener.lookupTransform("/base_link", "/camera_measure_link", ros::Time(0), tf_camera_to_base);

      double roll, pitch, yaw;
      tf_camera_to_base.getBasis().getRPY(roll, pitch, yaw);

      double x = tf_camera_to_base.getOrigin().x();
      double y = tf_camera_to_base.getOrigin().y();
      double z = tf_camera_to_base.getOrigin().z();
      printf("\n------------------------------\n");
      printf("x = %4.3f y = %4.3f z = %4.3f roll = %3.2f pitch = %3.2f yaw = %3.2f\n", x, y, z, radian2Degree(roll), radian2Degree(pitch), radian2Degree(yaw));
      printf("\n------------------------------\n");
     // std::cout << "camera in base, x:" << x << " y:" << y << " z:" << z << " roll:" << radian2Degree(roll) << " pitch:" << radian2Degree(pitch) << " yaw:" << radian2Degree(yaw) << std::endl;
      return true;
    } catch(tf::TransformException& ex) {
      ROS_ERROR("%s", ex.what());
      if (cnt == 0) {
        std::cout << "max times tried, can't get charge link in odom!" << std::endl;
        return false;
      }
      ros::Duration(1.0).sleep();
    }
  }
}

bool calibrate_camera_pose(common_pkg::task_cmdRequest& req, common_pkg::task_cmdResponse& res) {
  return calibrate_camera_in_base();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "ar_pose_estimate");

  ros::NodeHandle nh;

  signal(SIGINT, sigint_handler);

  ros::ServiceServer record_target_server = nh.advertiseService("/ar_pose_estimate/record_target_pose", record_target_pose);

  g_pub_pose = nh.advertise<geometry_msgs::Pose>("/ar_pose_estimate/marker_to_camera", 1);

  init_target_in_base();

  bool calibrate_camera_pose_mode = false;
  if (!ros::param::get("/ar_pose_estimate/calibrate_camera_pose_mode", calibrate_camera_pose_mode)) {
    ROS_ERROR("get param calibrate camera failed!");
  }

  if (calibrate_camera_pose_mode) {
    std::cout << "calibrate camera pose mode on" << std::endl;
    init_marker_in_base();
    static ros::ServiceServer calibrate_camera_in_base_server = nh.advertiseService("/ar_pose_estimate/calibrate_camera_pose", calibrate_camera_pose);
  } else {
    init_camera_in_base();
    std::cout << "calibrate camera pose mode off" << std::endl;
  }


// init_transform();
// ros::NodeHandle nh;
// g_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("ar_pose", 1);
  main_api(argc, argv);
  ros::spin();
  return 0;
}
