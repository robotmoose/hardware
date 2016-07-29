/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


// This file utilizes the Kinect microphone data collecting example micview.c included with
//     libfreenect to implement sound source localization using the Kinect's microphone array.

#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_audio.h>
#include "ofxKinectExtras.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "Shader.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "xcor_td.h"
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <deque>

#include <iostream>
#include <fstream>
std::ofstream myfile;

pthread_t freenect_thread;
volatile int die = 0;

static freenect_context* f_ctx;
static freenect_device* f_dev;

typedef struct {
	int32_t* buffers[4];
	int max_samples;
	int current_idx;  // index to the oldest data in the buffer (equivalently, where the next new data will be placed)
	int new_data;
} capture;

capture state;

// Approximate linear coordinates of the Kinect's built in microphones relative to the RGB camera's position (in meters).
//     Source: http://www.mathworks.com/help/audio/examples/live-direction-of-arrival-estimation-with-a-linear-microphone-array.html
//static const double MIC_POSITIONS[4] = {-0.088, 0.042, 0.078, 0.11};
//     Source: http://giampierosalvi.blogspot.com/2013/12/ms-kinect-microphone-array-geometry.html
static const double MIC_POSITIONS[4] = {0.113, -0.036, -.076, -0.113};

static const double SOUND_SPEED = 343.0; // Sound speed in meters per second.
static const double SAMPLE_FREQUENCY = 16000.0; // 16 kHz per channel.
static const int NUMSAMPLES_XCOR = 256*16; // Width of the sliding window.
int xcor_counter = 0; // Counts up to NUMSAMPLES_XCOR/2 to trigger xcor. 

// The microphones will be almost nearly in sync, so only need to compute the xcor for a short window.
static const int MAX_LAG = std::round(fabs(MIC_POSITIONS[0] - MIC_POSITIONS[3])/SOUND_SPEED*SAMPLE_FREQUENCY);
static const int XCOR_WIDTH = 2*MAX_LAG;

static const bool KINECT_1473 = true; // Need to upload special firmware if using Kinect Model #1473

// Create arrays to store microphone data for xcor. This allows us to collect data and do calculations simultaneously.
int32_t ** xcor_data = new int32_t* [4];

std::deque<int32_t> mic1_d, mic2_d, mic3_d, mic4_d; // Store microphone data streams

int paused = 0;

pthread_mutex_t audiobuf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t audiobuf_cond = PTHREAD_COND_INITIALIZER;

// Function Prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown);
void* freenect_threadfunc(void* arg);
double findAngle();
void drawAngle();
void cleanup();

static float angle = 0;

// Shaders
// const GLchar* vertexShaderSource = "#version 330 core\n"
//     "layout (location = 0) in vec3 position;\n"
//     // "layout (location = 1) in vec3 color;\n"
//     // "layout (location = 2) in vec2 texCoord;\n"
//     // "out vec3 ourColor;\n"
//     // "out vec2 TexCoord;\n"
//     "uniform mat4 transform;\n"
//     "void main()\n"
//     "{\n"
//     "gl_Position = transform * vec4(position.x, position.y, position.z, 1.0);\n"
//     // "ourColor = color;\n"
//     // "TexCoord = vec2(texCoord.x, 1.0 - texCoord.y);\n"
//     "}\n\0";
// const GLchar* fragmentShaderSource = "#version 330 core\n"
//     "out vec4 color;\n"
//     "void main()\n"
//     "{\n"
//     "color = vec4(1.0f, 0.5f, 0.2f, 1.0f);\n"
//     "}\n\0";

int main(int argc, char** argv) {

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

	if(KINECT_1473) {
		// Need to upload audio firmware for Kinect model #1473
		freenect_set_fw_address_nui(f_ctx, ofxKinectExtras::getFWData1473(), ofxKinectExtras::getFWSize1473());
		freenect_set_fw_address_k4w(f_ctx, ofxKinectExtras::getFWDatak4w(), ofxKinectExtras::getFWSizek4w());	
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_INFO);
	freenect_select_subdevices(f_ctx, FREENECT_DEVICE_AUDIO);

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);
	if (nr_devices < 1) {
		freenect_shutdown(f_ctx);
		return 1;
	}

	int user_device_number = 0;
	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return 1;
	}

	for(int i=0; i<4; ++i) {
		xcor_data[i] = new int32_t[NUMSAMPLES_XCOR];
	}

	freenect_set_user(f_dev, &state);

	freenect_set_audio_in_callback(f_dev, in_callback);
	freenect_start_audio(f_dev);

	int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		freenect_shutdown(f_ctx);
		return 1;
	}
	printf("This is the Kinect DOA viewer. Press 'esc' to exit.\n");

	// ***** Begin Graphics Setup ***** //
	glfwInit();
	// Set required options for GLFW
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Needed for OSX compatibility. 
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	GLFWwindow* window = glfwCreateWindow(800, 600, "DOA Estimate", nullptr, nullptr);
	if(window == nullptr) {
		printf("Failed to create GLFW window!\n");
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	glfwSetKeyCallback(window, key_callback);

	glewExperimental = GL_TRUE;
	if(glewInit() != GLEW_OK) {
		printf("Failed to initialize GLEW!\n");
		return -1;
	}

	int width, height;
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0,0, width, height);
	
	Shader shader("../shaders/transformations.vs", "../shaders/transformations.frag");

    GLfloat vertices[] = {
    	0.0075f, 0.75f, 0.0f, // Top right of stem
    	0.0075f, 0.0f, 0.0f, // Bottom right of stem
    	-0.0075f, 0.0f, 0.0f, // Bottom left of stem
    	-0.0075f, 0.75f, 0.0f, // Top left of stem
    	0.025f, 0.75f, 0.0f, // Right bottom arrow tip
    	-0.025f, 0.75f, 0.0f, // Left bottom arrow tip
    	0.0f, 0.8f, 0.0f // Tip of arrow
    };
    GLuint indices[] = {  // Note that we start from 0!
        0, 1, 3,  // First Triangle
        1, 2, 3,  // Second Triangle
        4, 5, 6   // Third Triangle
    };
    GLuint VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    // Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0); // Note that this is allowed, the call to glVertexAttribPointer registered VBO as the currently bound vertex buffer object so afterwards we can safely unbind

    glBindVertexArray(0); // Unbind VAO (it's always a good thing to unbind any buffer/array to prevent strange bugs), remember: do NOT unbind the EBO, keep it bound to this VAO

	while(!glfwWindowShouldClose(window)) {
		glfwPollEvents();

        // Render
        // Clear the colorbuffer
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw our first triangle
        shader.Use();

        // Create Transformations
		glm::mat4 transform;
		transform = glm::rotate(transform, glm::radians(angle), glm::vec3(0.0, 0.0, 1.0));
		//transform = glm::translate(transform, glm::vec3(0.5f, -0.5f, 0.0f));

		GLint transformLoc = glGetUniformLocation(shader.Program, "transform");
		glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(transform));

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, 9, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        // Swap the screen buffers
        glfwSwapBuffers(window);
	}

    // Properly de-allocate all resources once they've outlived their purpose
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    // Terminate GLFW, clearing any resources allocated by GLFW.
    glfwTerminate();
	die = 1;
	pthread_exit(NULL);
	
	// ***** End Graphics Setup ***** //
	return 0;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode) {
	if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GL_TRUE);
	}
}

void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown) {
	pthread_mutex_lock(&audiobuf_mutex);
	if(mic1_d.size() < NUMSAMPLES_XCOR) { // Still filling up buffers.
		for(int i=0; i<num_samples; ++i) {
			mic1_d.push_back(mic1[i]);
			mic2_d.push_back(mic2[i]);
			mic3_d.push_back(mic3[i]);
			mic4_d.push_back(mic4[i]);
		}
	}
	else { // buffers full.
		for(int i=0; i<num_samples; ++i) {
			mic1_d.pop_front();
			mic1_d.push_back(mic1[i]);
			mic2_d.pop_front();
			mic2_d.push_back(mic2[i]);
			mic3_d.pop_front();
			mic3_d.push_back(mic3[i]);
			mic4_d.pop_front();
			mic4_d.push_back(mic4[i]);
		}
	}
	xcor_counter += num_samples;
	// Perform the DOA analysis if we have collected the appropriate number of samples.
	if(xcor_counter >= NUMSAMPLES_XCOR/2 && mic1_d.size() >= NUMSAMPLES_XCOR) { // xcors overlap by 50%.

		for(int i=0; i<NUMSAMPLES_XCOR; ++i) {
			xcor_data[0][i] = mic1_d[i];
			xcor_data[1][i] = mic2_d[i];
			xcor_data[2][i] = mic3_d[i];
			xcor_data[3][i] = mic4_d[i];
		}

		xcor_counter=0;

		pthread_cond_signal(&audiobuf_cond);
		pthread_mutex_unlock(&audiobuf_mutex);
		angle = findAngle();
		printf("The estimated angle to the source is %f degrees\n", angle);
	}
	else {
		pthread_cond_signal(&audiobuf_cond);
		pthread_mutex_unlock(&audiobuf_mutex);		
	}
}

void* freenect_threadfunc(void* arg) {
	while(!die && freenect_process_events(f_ctx) >= 0) {
		// If we did anything else in the freenect thread, it might go here.
	}
	freenect_stop_audio(f_dev);
	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);
	cleanup();
	return NULL;
}

// Returns the estimated angle to the dominant sound source in degrees,
double findAngle() {
	std::vector<std::pair<int32_t,double>> lags_and_x;
	lags_and_x.reserve(8);

	// Determine the lags between the microphone pairs.
	// Ideally, this would be done in a loop.
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[0], xcor_data[1], NUMSAMPLES_XCOR, XCOR_WIDTH).second, 
		fabs(MIC_POSITIONS[0]-MIC_POSITIONS[1])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[0], xcor_data[2], NUMSAMPLES_XCOR, XCOR_WIDTH).second, 
		fabs(MIC_POSITIONS[0]-MIC_POSITIONS[2])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[0], xcor_data[3], NUMSAMPLES_XCOR, XCOR_WIDTH).second, 
		fabs(MIC_POSITIONS[0]-MIC_POSITIONS[3])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[1], xcor_data[2], NUMSAMPLES_XCOR, XCOR_WIDTH).second, 
		fabs(MIC_POSITIONS[1]-MIC_POSITIONS[2])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[1], xcor_data[3], NUMSAMPLES_XCOR, XCOR_WIDTH).second, 
		fabs(MIC_POSITIONS[1]-MIC_POSITIONS[3])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[2], xcor_data[3], NUMSAMPLES_XCOR, XCOR_WIDTH).second, 
		fabs(MIC_POSITIONS[2]-MIC_POSITIONS[3])));

	// std::vector<double> angles;
	// for(int i=0; i<6; ++i) {
	// 	angles.push_back(asin(lags_and_x[i].first*SOUND_SPEED/(SAMPLE_FREQUENCY*lags_and_x[i].second))*180.0/M_PI);
	// 	printf("%f\n", lags_and_x[i].first*SOUND_SPEED/(SAMPLE_FREQUENCY*lags_and_x[i].second));
	// 	//printf("angle %d: %f\n", i, angles[i]);
	// }
	// std::sort(angles.begin(), angles.end());
	// for(int i=0; i<6; ++i) {
	// 	printf("angle %d: %f\n", i, angles[i]);
	// }

	// for(int i=0; i<5; ++i) {
	// 	printf("lag %d: %d\n", i, lags_and_x[i].first);
	// }

	// Determine the median lag
	std::sort(lags_and_x.begin(), lags_and_x.end(), 
		[](const std::pair<int32_t,double> &a, const std::pair<int32_t, double> & b) {
			return b.first < a.first;
	});

	// Determine the angle using the median lag.
	double sin_angle = lags_and_x[3].first*SOUND_SPEED/(SAMPLE_FREQUENCY*lags_and_x[3].second);
	// Clamp the angle.
	if(sin_angle > 1) return 90.0;
	else if(sin_angle < -1) return -90.0;
	else return asin(sin_angle)*180/M_PI;
}

void drawAngle() {
}

void cleanup() {
	for(int i=0; i<4; ++i) {
		delete [] xcor_data[i];
	}
	delete [] xcor_data;
}

// void DrawMicData() {
// 	if (paused)
// 		return;
// 	pthread_mutex_lock(&audiobuf_mutex);
// 	while(!state.new_data)
// 		pthread_cond_wait(&audiobuf_cond, &audiobuf_mutex);
// 	state.new_data = 0;
// 	// Draw:
// 	glClear(GL_COLOR_BUFFER_BIT);
// 	glMatrixMode(GL_MODELVIEW);
// 	glLoadIdentity();

// 	float xIncr = (float)win_w / state.max_samples;
// 	float x = 0.;
// 	int i;
// 	int base_idx = state.current_idx;

// 	// Technically, we should hold the lock until we're done actually drawing
// 	// the lines, but this is sufficient to ensure that the drawings align
// 	// provided we don't reallocate buffers.
// 	pthread_mutex_unlock(&audiobuf_mutex);

// 	// This is kinda slow.  It should be possible to compile each sample
// 	// window into a glCallList, but that's overly complex.
// 	int mic;
// 	for(mic = 0; mic < 4; mic++) {
// 		glBegin(GL_LINE_STRIP);
// 		glColor4f(1.0f, 1.0f, 1.0f, 0.7f);
// 		for(x = 0, i = 0; i < state.max_samples; i++) {
// 			glVertex3f(x, ((float)win_h * (float)(2*mic + 1) / 8. ) + (float)(state.buffers[mic][(base_idx + i) % state.max_samples]) * ((float)win_h/4) /2147483647. , 0);
// 			x += xIncr;
// 		}
// 		glEnd();
// 	}
// 	glutSwapBuffers();
// }

// void Reshape(int w, int h) {
// 	win_w = w;
// 	win_h = h;
// 	glViewport(0, 0, w, h);
// 	glMatrixMode(GL_PROJECTION);
// 	glLoadIdentity();
// 	glOrtho(0.0, (float)w, (float)h, 0.0, -1.0, 1.0);
// 	glMatrixMode(GL_MODELVIEW);
// 	glLoadIdentity();
// }

// void Keyboard(unsigned char key, int x, int y) {
// 	if(key == 'q') {
// 		die = 1;
// 		pthread_exit(NULL);
// 	}
// 	if(key == 32) {
// 		paused = !paused;
// 	}
// }