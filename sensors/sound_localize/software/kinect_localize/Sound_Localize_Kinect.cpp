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

#include <cstdio>
#include <pthread.h>

// Libraries needed for OpenGL graphics
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "SOIL.h"
#include "Shader.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Libraries needed for DSP
#include <deque>
#include "Kinect_DOA.h"

// ********** Constants and Variables ********** //

// ***** Freenect variables ***** //
pthread_t freenect_thread;
volatile int die = 0;

pthread_mutex_t audiobuf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t audiobuf_cond = PTHREAD_COND_INITIALIZER;

static freenect_context* f_ctx;
static freenect_device* f_dev;
// ********** // 

// ***** Constants and Variables needed for DOA Estimation ***** // 
Kinect_DOA kinect_DOA;
int xcor_counter = 0; // Counts up to NUMSAMPLES_XCOR/2 to trigger xcor. 
std::deque<int32_t> mic1_d, mic2_d, mic3_d, mic4_d; // Store microphone data streams
static float angle = 0;

// ********** // 

static const bool KINECT_1473 = true; // Need to upload special firmware if using Kinect Model #1473

const GLuint WIDTH = 800, HEIGHT = 600;

// ******************** // 

// Function Prototypes
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode);
void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown);
void* freenect_threadfunc(void* arg);

int main() {
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

	freenect_set_user(f_dev, &mic1_d);

	freenect_set_audio_in_callback(f_dev, in_callback);
	freenect_start_audio(f_dev);

	int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		freenect_shutdown(f_ctx);
		return 1;
	}
	printf("This is the Kinect DOA viewer. Press 'esc' or 'q' to exit.\n");

	// ***** Begin Graphics Setup ***** //
	glfwInit();
	// Set required options for GLFW
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // Needed for OSX compatibility. 
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);

	GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "DOA Estimate", nullptr, nullptr);
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

	// glViewport(0,0, WIDTH, HEIGHT);

	// // Load in the background
	// GLuint background_texture;
	// glGenTextures(1, &background_texture);
	// glBindTexture(GL_TEXTURE_2D, background_texture);

 //    // Set texture filtering
 //    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
 //    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// int width, height;
	// unsigned char* background_image = SOIL_load_image("../background.jpg", &width, &height, 0, SOIL_LOAD_RGB);
	// glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, background_image);
	// glGenerateMipmap(GL_TEXTURE_2D);
	// SOIL_free_image_data(background_image);
	// glBindTexture(GL_TEXTURE_2D, 0); // Unbind the texture

	Shader shader("../shaders/transformations.vs", "../shaders/transformations.frag");

	// GLfloat background_vertices[] = {
	// 	0.0f, 0.0f,
	// 	0.0f, 1.0f,
	// 	1.0f, 0.0f,
	// 	1.0f, 1.0f
	// };
	// GLuint background_indices[] = {
	// 	0, 1, 3,
	// 	1, 2, 3
	// };

    GLfloat arrow_vertices[] = {
    	0.0075f, 0.75f, 0.0f, // Top right of stem
    	0.0075f, 0.0f, 0.0f, // Bottom right of stem
    	-0.0075f, 0.0f, 0.0f, // Bottom left of stem
    	-0.0075f, 0.75f, 0.0f, // Top left of stem
    	0.025f, 0.75f, 0.0f, // Right bottom arrow tip
    	-0.025f, 0.75f, 0.0f, // Left bottom arrow tip
    	0.0f, 0.8f, 0.0f // Tip of arrow
    };
    GLuint arrow_indices[] = {
        0, 1, 3,  // First Triangle
        1, 2, 3,  // Second Triangle
        4, 5, 6   // Third Triangle
    };
    GLuint VBO[2], VAO[2], EBO[2];
    glGenVertexArrays(2, VAO);
    glGenBuffers(2, VBO);
    glGenBuffers(2, EBO);
    // Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
    glBindVertexArray(VAO[1]);

    glBindBuffer(GL_ARRAY_BUFFER, VBO[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(arrow_vertices), arrow_vertices, GL_DYNAMIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(arrow_indices), arrow_indices, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0); // Note that this is allowed, the call to glVertexAttribPointer registered VBO as the currently bound vertex buffer object so afterwards we can safely unbind

    glBindVertexArray(0); // Unbind VAO (it's always a good thing to unbind any buffer/array to prevent strange bugs), remember: do NOT unbind the EBO, keep it bound to this VAO
    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // // Setup Background Texture
    // glBindVertexArray(VAO[0]);

    // glBindBuffer(GL_ARRAY_BUFFER, VBO[0]);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(background_vertices), background_vertices, GL_STATIC_DRAW);

    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO[0]);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(background_indices), background_indices, GL_STATIC_DRAW);

	while(!glfwWindowShouldClose(window)) {
		glfwPollEvents();

        // Render
        // Clear the colorbuffer
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw our first triangle
        shader.Use();

  //       glBindTexture(GL_TEXTURE_2D, background_texture);
  //       glBindVertexArray(VAO[0]);
  //       glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO[0]);
  //       glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
 	// //	glBindVertexArray(0);
 	// 	glBindTexture(GL_TEXTURE_2D, 0);

        // Create Transformations
		glm::mat4 transform;
		transform = glm::rotate(transform, glm::radians(angle), glm::vec3(0.0, 0.0, 1.0));
		//transform = glm::translate(transform, glm::vec3(0.5f, -0.5f, 0.0f));

		GLint transformLoc = glGetUniformLocation(shader.Program, "transform");
		glUniformMatrix4fv(transformLoc, 1, GL_FALSE, glm::value_ptr(transform));

        glBindVertexArray(VAO[1]);
        //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO[1]);
        glDrawElements(GL_TRIANGLES, 9, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        // Swap the screen buffers
        glfwSwapBuffers(window);
	}

    // Properly de-allocate all resources once they've outlived their purpose
    glDeleteVertexArrays(2, VAO);
    glDeleteBuffers(2, VBO);
    glDeleteBuffers(2, EBO);
    // Terminate GLFW, clearing any resources allocated by GLFW.
    glfwTerminate();
	die = 1;
	pthread_exit(NULL);
	
	// ***** End Graphics Setup ***** //
	return 0;
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mode) {
	if((key == GLFW_KEY_ESCAPE || key == GLFW_KEY_Q) && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window, GL_TRUE);
	}
}

void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown) {
	pthread_mutex_lock(&audiobuf_mutex);
	if(mic1_d.size() < kinect_DOA.NUMSAMPLES_XCOR) { // Still filling up buffers.
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
	if(xcor_counter >= kinect_DOA.NUMSAMPLES_XCOR/2 && mic1_d.size() >= kinect_DOA.NUMSAMPLES_XCOR) { // xcors overlap by 50%.

		for(int i=0; i<kinect_DOA.NUMSAMPLES_XCOR; ++i) {
			kinect_DOA.xcor_data[0][i] = mic1_d[i];
			kinect_DOA.xcor_data[1][i] = mic2_d[i];
			kinect_DOA.xcor_data[2][i] = mic3_d[i];
			kinect_DOA.xcor_data[3][i] = mic4_d[i];
		}

		xcor_counter=0;

		pthread_cond_signal(&audiobuf_cond);
		pthread_mutex_unlock(&audiobuf_mutex);
		angle = kinect_DOA.findAngle();
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
	return NULL;
}