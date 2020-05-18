/*
 * assignment2_drums
 * ECS7012 Music and Audio Programming
 *
 * Second assignment, to create a sequencer-based
 * drum machine which plays sampled drum sounds in loops.
 *
 * This code runs on the Bela embedded audio platform (bela.io).
 *
 * Andrew McPherson, Becky Stewart and Victor Zappi
 * 2015-2020
 */

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <libgen.h>
#include <signal.h>
#include <getopt.h>
#include <libraries/sndfile/sndfile.h>
#include <Bela.h>
#include "drums.h"

using namespace std;


/* Drum samples are pre-loaded in these buffers. Length of each
 * buffer is given in gDrumSampleBufferLengths.
 */
float *gDrumSampleBuffers[NUMBER_OF_DRUMS];
int gDrumSampleBufferLengths[NUMBER_OF_DRUMS];

/* Patterns indicate which drum(s) should play on which beat.
 * Each element of gPatterns is an array, whose length is given
 * by gPatternLengths.
 */
int *gPatterns[NUMBER_OF_PATTERNS];
int gPatternLengths[NUMBER_OF_PATTERNS];

// Handle Ctrl-C by requesting that the audio rendering stop
void interrupt_handler(int var)
{
	gShouldStop = true;
}

// Print usage information
void usage(const char * processName)
{
	cerr << "Usage: " << processName << " [options]" << endl;

	Bela_usage();

	cerr << "   --help [-h]:                Print this menu\n";
}

int initDrums() {
	/* Load drums from WAV files */
	SNDFILE *sndfile ;
	SF_INFO sfinfo ;
	char filename[64];

	for(int i = 0; i < NUMBER_OF_DRUMS; i++) {
		snprintf(filename, 64, "./drum%d.wav", i);

		if (!(sndfile = sf_open (filename, SFM_READ, &sfinfo))) {
			printf("Couldn't open file %s\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gDrumSampleBuffers[j]);
			return 1;
		}

		if (sfinfo.channels != 1) {
			printf("Error: %s is not a mono file\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gDrumSampleBuffers[j]);
			return 1;
		}

		gDrumSampleBufferLengths[i] = sfinfo.frames;
		gDrumSampleBuffers[i] = (float *)malloc(gDrumSampleBufferLengths[i] * sizeof(float));
		if(gDrumSampleBuffers[i] == NULL) {
			printf("Error: couldn't allocate buffer for %s\n", filename);

			/* Free already loaded sounds */
			for(int j = 0; j < i; j++)
				free(gDrumSampleBuffers[j]);
			return 1;
		}

		int subformat = sfinfo.format & SF_FORMAT_SUBMASK;
		int readcount = sf_read_float(sndfile, gDrumSampleBuffers[i], gDrumSampleBufferLengths[i]);

		/* Pad with zeros in case we couldn't read whole file */
		for(int k = readcount; k < gDrumSampleBufferLengths[i]; k++)
			gDrumSampleBuffers[i][k] = 0;

		if (subformat == SF_FORMAT_FLOAT || subformat == SF_FORMAT_DOUBLE) {
			double	scale ;
			int 	m ;

			sf_command (sndfile, SFC_CALC_SIGNAL_MAX, &scale, sizeof (scale)) ;
			if (scale < 1e-10)
				scale = 1.0 ;
			else
				scale = 32700.0 / scale ;
			printf("Scale = %f\n", scale);

			for (m = 0; m < gDrumSampleBufferLengths[i]; m++)
				gDrumSampleBuffers[i][m] *= scale;
		}

		sf_close(sndfile);
	}

	return 0;
}

void cleanupDrums() {
	for(int i = 0; i < NUMBER_OF_DRUMS; i++)
		free(gDrumSampleBuffers[i]);
}

void initPatterns() {
	int pattern0[16] = {0x05, 0x04, 0x14, 0, 0x06, 0, 0x04, 0x01, 0x04, 0x01, 0x05, 0, 0x06, 0, 0x04, 0x10};
	int pattern1[32] = {0x09, 0, 0x04, 0x01, 0x06, 0, 0x04, 0x01,
		 0x05, 0, 0x04, 0x01, 0x06, 0, 0x04, 0x03,
		 0x09, 0, 0x21, 0, 0x06, 0, 0x21, 0,
		 0x04, 0x01, 0x21, 0x01, 0x06, 0, 0x20, 0};
	//My pattern
	int pattern2[16] = {0x05, 0x00, 0x50, 0x01, 0x04, 0x40, 0x01, 0x40, 0x04, 0x40, 0x40, 0x01, 0x04, 0x40, 0x01, 0x50};
	int pattern3[16] = {0x11, 0, 0x10, 0x01, 0x12, 0x40, 0x04, 0x40, 0x11, 0x42, 0x50, 0x01, 0x12, 0x21, 0x30, 0x20};
	int pattern4[32] = {0x81, 0x80, 0x80, 0x80, 0x01, 0x80, 0x80, 0x80, 0x81, 0, 0, 0, 0x41, 0x80, 0x80, 0x80,
		0x81, 0x80, 0x80, 0, 0x41, 0, 0x80, 0x80, 0x81, 0x80, 0x80, 0x80, 0xC1, 0, 0, 0};
	int pattern5[16] = {0x81, 0x02, 0, 0x81, 0x0A, 0, 0xA1, 0x10, 0xA2, 0x11, 0x46, 0x41, 0xC5, 0x81, 0x81, 0x89};
	
	
	gPatternLengths[0] = 16;
	gPatterns[0] = (int *)malloc(gPatternLengths[0] * sizeof(int));
	memcpy(gPatterns[0], pattern0, gPatternLengths[0] * sizeof(int));

	gPatternLengths[1] = 32;
	gPatterns[1] = (int *)malloc(gPatternLengths[1] * sizeof(int));
	memcpy(gPatterns[1], pattern1, gPatternLengths[1] * sizeof(int));

	gPatternLengths[2] = 16;
	gPatterns[2] = (int *)malloc(gPatternLengths[2] * sizeof(int));
	memcpy(gPatterns[2], pattern2, gPatternLengths[2] * sizeof(int));

	gPatternLengths[3] = 16;
	gPatterns[3] = (int *)malloc(gPatternLengths[3] * sizeof(int));
	memcpy(gPatterns[3], pattern3, gPatternLengths[3] * sizeof(int));

	gPatternLengths[4] = 32;
	gPatterns[4] = (int *)malloc(gPatternLengths[4] * sizeof(int));
	memcpy(gPatterns[4], pattern4, gPatternLengths[4] * sizeof(int));

	gPatternLengths[5] = 16;
	gPatterns[5] = (int *)malloc(gPatternLengths[5] * sizeof(int));
	memcpy(gPatterns[5], pattern5, gPatternLengths[5] * sizeof(int));
}

void cleanupPatterns() {
	for(int i = 0; i < NUMBER_OF_PATTERNS; i++)
		free(gPatterns[i]);
}

int main(int argc, char *argv[])
{
	BelaInitSettings  settings;	// Standard audio settings

	struct option customOptions[] =
	{
		{"help", 0, NULL, 'h'},
		{NULL, 0, NULL, 0}
	};

	// Set default settings
	Bela_defaultSettings(&settings);
	settings.setup = setup;
	settings.render = render;
	settings.cleanup = cleanup;

	// Parse command-line arguments
	while (1) {
		int c;
		if ((c = Bela_getopt_long(argc, argv, "hf:", customOptions, &settings)) < 0)
				break;
		switch (c) {
		case 'h':
				usage(basename(argv[0]));
				exit(0);
		case '?':
		default:
				usage(basename(argv[0]));
				exit(1);
		}
	}

	// Load the drum sounds and the patterns
    if(initDrums()) {
    	printf("Unable to load drum sounds. Check that you have all the WAV files!\n");
    	return -1;
    }
    initPatterns();

	// Initialise the PRU audio device
	if(Bela_initAudio(&settings, 0) != 0) {
		cout << "Error: unable to initialise audio" << endl;
		return -1;
	}


	// Start the audio device running
	if(Bela_startAudio()) {
		cout << "Error: unable to start real-time audio" << endl;
		return -1;
	}

	// Set up interrupt handler to catch Control-C
	signal(SIGINT, interrupt_handler);
	signal(SIGTERM, interrupt_handler);

	// Run until told to stop
	while(!gShouldStop) {
		usleep(100000);
	}

	// Stop the audio device and sensor thread
	Bela_stopAudio();

	// Clean up any resources allocated for audio
	Bela_cleanupAudio();

	// Clean up the drums
	cleanupPatterns();
	cleanupDrums();

	// All done!
	return 0;
}
