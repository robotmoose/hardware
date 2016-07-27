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
#include <string.h>
#include <signal.h>
#include <pthread.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "xcor_td.h"
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>
#include <deque>

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
//const double MIC_POSITIONS[4] = {-0.088, 0.042, 0.078, 0.11};

//     Source: http://giampierosalvi.blogspot.com/2013/12/ms-kinect-microphone-array-geometry.html
const double MIC_POSITIONS[4] = {0.113, -0.036, -.076, -0.113};

const double SOUND_SPEED = 343.0; // Sound speed in meters per second.
const double SAMPLE_FREQUENCY = 16000.0; // 16 kHz per channel.
const int NUMSAMPLES_XCOR = 256*32; // Width of the sliding window.
// Since microphones are so close together, only need to compute the xcor near the center of the window.
const int XCOR_WIDTH = 32;
int xcor_counter = 0; // Counts up to NUMSAMPLES_XCOR/2 to trigger xcor. 

// Create arrays to store microphone data for xcor. This allows us to collect data and do calculations simultaneously.
int32_t ** xcor_data = new int32_t* [4];

int32_t sig1[256*16];
int32_t sig2[256*16];

std::deque<int32_t> mic1_d;
std::deque<int32_t> mic2_d;
std::deque<int32_t> mic3_d;
std::deque<int32_t> mic4_d;

int paused = 0;

pthread_mutex_t audiobuf_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t audiobuf_cond = PTHREAD_COND_INITIALIZER;

int win_h, win_w;

double findAngle() {
	double angle = 0;
	std::vector<std::pair<int32_t,double>> lags_and_x;
	lags_and_x.reserve(8);
	std::vector<double> angles;

	// Determine the lags between the microphone pairs.
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[0], xcor_data[1], NUMSAMPLES_XCOR, XCOR_WIDTH).second, fabs(MIC_POSITIONS[0]-MIC_POSITIONS[1])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[0], xcor_data[2], NUMSAMPLES_XCOR, XCOR_WIDTH).second, fabs(MIC_POSITIONS[0]-MIC_POSITIONS[2])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[0], xcor_data[3], NUMSAMPLES_XCOR, XCOR_WIDTH).second, fabs(MIC_POSITIONS[0]-MIC_POSITIONS[3])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[1], xcor_data[2], NUMSAMPLES_XCOR, XCOR_WIDTH).second, fabs(MIC_POSITIONS[1]-MIC_POSITIONS[2])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[1], xcor_data[3], NUMSAMPLES_XCOR, XCOR_WIDTH).second, fabs(MIC_POSITIONS[1]-MIC_POSITIONS[3])));
	lags_and_x.push_back(std::make_pair(xcor_td(xcor_data[2], xcor_data[3], NUMSAMPLES_XCOR, XCOR_WIDTH).second, fabs(MIC_POSITIONS[2]-MIC_POSITIONS[3])));

	for(int i=0; i<6; ++i) {
		angles.push_back(-asin(lags_and_x[i].first*SOUND_SPEED/(SAMPLE_FREQUENCY*lags_and_x[i].second))*180.0/M_PI);
		//printf("%f\n", -asin(lags_and_x[i].first*SOUND_SPEED/(SAMPLE_FREQUENCY*lags_and_x[i].second))*180.0/M_PI);
		//printf("angle %d: %f\n", i, angles[i]);
	}
	//std::sort(angles.begin(), angles.end());
	// for(int i=0; i<6; ++i) {
	// 	printf("angle %d: %f\n", i, angles[i]);
	// }

	for(int i=0; i<5; ++i) {
		printf("lag %d: %d\n", i, lags_and_x[i].first);
	}
	// Determine the median lag
	// std::sort(lags_and_x.begin(), lags_and_x.end(), [](const std::pair<int32_t,double> &a, const std::pair<int32_t, double> & b) {
	// 	return b.first < a.first;
	// });
	//angle = -asin(lags_and_x[3].first*SOUND_SPEED/(SAMPLE_FREQUENCY*lags_and_x[3].second))*180/M_PI;


	//angles[0] = -asin(lags[0]*SOUND_SPEED/(SAMPLE_FREQUENCY*(MIC_POSITIONS[0]-MIC_POSITIONS[1])))*180/M_PI;


	return angles[3];
}

void in_callback(freenect_device* dev, int num_samples,
                 int32_t* mic1, int32_t* mic2,
                 int32_t* mic3, int32_t* mic4,
                 int16_t* cancelled, void *unknown) {
	pthread_mutex_lock(&audiobuf_mutex);
	capture* c = (capture*)freenect_get_user(dev);
	if(mic1_d.size() < NUMSAMPLES_XCOR) {
		for(int i=0; i<num_samples; ++i) {
			mic1_d.push_back(mic1[i]);
			mic2_d.push_back(mic2[i]);
			mic3_d.push_back(mic3[i]);
			mic4_d.push_back(mic4[i]);
		}
	}
	else {
		for(int i=0; i<num_samples; ++i) {
			mic1_d.push_back(mic1[i]);
			mic1_d.pop_front();
			mic2_d.push_back(mic2[i]);
			mic2_d.pop_front();
			mic3_d.push_back(mic3[i]);
			mic3_d.pop_front();
			mic4_d.push_back(mic4[i]);
			mic4_d.pop_front();
		}
	}
	if(num_samples < c->max_samples - c->current_idx) {
		memcpy(&(c->buffers[0][c->current_idx]), mic1, num_samples*sizeof(int32_t));
		memcpy(&(c->buffers[1][c->current_idx]), mic2, num_samples*sizeof(int32_t));
		memcpy(&(c->buffers[2][c->current_idx]), mic3, num_samples*sizeof(int32_t));
		memcpy(&(c->buffers[3][c->current_idx]), mic4, num_samples*sizeof(int32_t));
	} else {
		int first = c->max_samples - c->current_idx;
		int left = num_samples - first;
		memcpy(&(c->buffers[0][c->current_idx]), mic1, first*sizeof(int32_t));
		memcpy(&(c->buffers[1][c->current_idx]), mic2, first*sizeof(int32_t));
		memcpy(&(c->buffers[2][c->current_idx]), mic3, first*sizeof(int32_t));
		memcpy(&(c->buffers[3][c->current_idx]), mic4, first*sizeof(int32_t));
		memcpy(c->buffers[0], &mic1[first], left*sizeof(int32_t));
		memcpy(c->buffers[1], &mic2[first], left*sizeof(int32_t));
		memcpy(c->buffers[2], &mic3[first], left*sizeof(int32_t));
		memcpy(c->buffers[3], &mic4[first], left*sizeof(int32_t));
	}
	c->current_idx = (c->current_idx + num_samples) % c->max_samples;
	c->new_data = 1;
	xcor_counter += num_samples;
	if(xcor_counter >= NUMSAMPLES_XCOR/2 && mic1_d.size() >= NUMSAMPLES_XCOR) { // xcors overlap by 50%.


		// for(int i=0; i<NUMSAMPLES_XCOR; ++i) {
		// 	myfile << std::to_string(state.buffers[0][i]) << "\n";
		// }

		// memcpy(xcor_data[0], mic1, NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[1], mic2, NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[2], mic3, NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[3], mic4, NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[0], &(c->buffers[0]), NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[1], &(c->buffers[1]), NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[2], &(c->buffers[2]), NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[3], &(c->buffers[3]), NUMSAMPLES_XCOR*sizeof(int32_t));

		// memcpy(xcor_data[0], &state.buffers[0], NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[1], &state.buffers[1], NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[2], &state.buffers[2], NUMSAMPLES_XCOR*sizeof(int32_t));
		// memcpy(xcor_data[3], &state.buffers[3], NUMSAMPLES_XCOR*sizeof(int32_t));

		for(int i=0; i<NUMSAMPLES_XCOR; ++i) {
			xcor_data[0][i] = mic1_d[i];
			xcor_data[1][i] = mic2_d[i];
			xcor_data[2][i] = mic3_d[i];
			xcor_data[3][i] = mic4_d[i];
		}

		// for(int i=0; i<NUMSAMPLES_XCOR; ++i) {
		// 	myfile << std::to_string(xcor_data[0][i]) << "\n";
		// }
		xcor_counter=0;

		pthread_cond_signal(&audiobuf_cond);
		pthread_mutex_unlock(&audiobuf_mutex);
		printf("The estimated angle to the source is %f degrees\n", findAngle());
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

	for(int i=0; i<4; ++i) {
		delete [] xcor_data[i];
	}
	delete [] xcor_data;

	return NULL;
}

void DrawMicData() {
	if (paused)
		return;
	pthread_mutex_lock(&audiobuf_mutex);
	while(!state.new_data)
		pthread_cond_wait(&audiobuf_cond, &audiobuf_mutex);
	state.new_data = 0;
	// Draw:
	glClear(GL_COLOR_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	float xIncr = (float)win_w / state.max_samples;
	float x = 0.;
	int i;
	int base_idx = state.current_idx;

	// Technically, we should hold the lock until we're done actually drawing
	// the lines, but this is sufficient to ensure that the drawings align
	// provided we don't reallocate buffers.
	pthread_mutex_unlock(&audiobuf_mutex);

	// This is kinda slow.  It should be possible to compile each sample
	// window into a glCallList, but that's overly complex.
	int mic;
	for(mic = 0; mic < 4; mic++) {
		glBegin(GL_LINE_STRIP);
		glColor4f(1.0f, 1.0f, 1.0f, 0.7f);
		for(x = 0, i = 0; i < state.max_samples; i++) {
			glVertex3f(x, ((float)win_h * (float)(2*mic + 1) / 8. ) + (float)(state.buffers[mic][(base_idx + i) % state.max_samples]) * ((float)win_h/4) /2147483647. , 0);
			x += xIncr;
		}
		glEnd();
	}
	glutSwapBuffers();
}

void Reshape(int w, int h) {
	win_w = w;
	win_h = h;
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, (float)w, (float)h, 0.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Keyboard(unsigned char key, int x, int y) {
	if(key == 'q') {
		die = 1;
		pthread_exit(NULL);
	}
	if(key == 32) {
		paused = !paused;
	}
}

int main(int argc, char** argv) {

	// Dummy signals for debugging xcor
	// for(int i=0; i<NUMSAMPLES_XCOR/2-30; ++i) {
	// 	sig1[i] = 0;
	// }

	// for(int i=0; i<NUMSAMPLES_XCOR/2-40; ++i) {
	// 	sig2[i] = 0;
	// }

	// for(int i=NUMSAMPLES_XCOR/2-30; i<NUMSAMPLES_XCOR-20; ++i) {
	// 	sig1[i] = 1;
	// }

	// for(int i=NUMSAMPLES_XCOR/2-40; i<NUMSAMPLES_XCOR-30; ++i) {
	// 	sig2[i] = 1;
	// }
	// for(int i=NUMSAMPLES_XCOR-20; i<NUMSAMPLES_XCOR; ++i) {
	// 	sig1[i] = 0;
	// }
	// 	for(int i=NUMSAMPLES_XCOR-30; i<NUMSAMPLES_XCOR; ++i) {
	// 	sig2[i] = 0;
	// }

	// printf("%d\n", xcor_td(sig1, sig2, NUMSAMPLES_XCOR, XCOR_WIDTH).second);
	// return 0;
	// myfile.open("myfile.csv");
	// printf("%ld", mic4_d.size());


	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

	// Need to upload audio firmware for Kinect model #1473
	freenect_set_fw_address_nui(f_ctx, ofxKinectExtras::getFWData1473(), ofxKinectExtras::getFWSize1473());
	freenect_set_fw_address_k4w(f_ctx, ofxKinectExtras::getFWDatak4w(), ofxKinectExtras::getFWSizek4w());	

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


	state.max_samples = 256 * 60;
	state.current_idx = 0;
	state.buffers[0] = (int32_t*)malloc(state.max_samples * sizeof(int32_t));
	state.buffers[1] = (int32_t*)malloc(state.max_samples * sizeof(int32_t));
	state.buffers[2] = (int32_t*)malloc(state.max_samples * sizeof(int32_t));
	state.buffers[3] = (int32_t*)malloc(state.max_samples * sizeof(int32_t));
	memset(state.buffers[0], 0, state.max_samples * sizeof(int32_t));
	memset(state.buffers[1], 0, state.max_samples * sizeof(int32_t));
	memset(state.buffers[2], 0, state.max_samples * sizeof(int32_t));
	memset(state.buffers[3], 0, state.max_samples * sizeof(int32_t));
	freenect_set_user(f_dev, &state);

	freenect_set_audio_in_callback(f_dev, in_callback);
	freenect_start_audio(f_dev);

	int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		freenect_shutdown(f_ctx);
		return 1;
	}
	printf("This is the libfreenect microphone waveform viewer.  Press 'q' to quit or spacebar to pause/unpause the view.\n");

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_ALPHA );
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Microphones");
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	Reshape(800, 600);
	glutReshapeFunc(Reshape);
	glutDisplayFunc(DrawMicData);
	glutIdleFunc(DrawMicData);
	glutKeyboardFunc(Keyboard);

	glutMainLoop();

	return 0;
}