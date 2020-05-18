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


#include <Bela.h>
#include <cmath>
#include "drums.h"

// Define the number of playable voices
#define NUMBER_OF_VOICES 16

//Sample rate. Re-calculated in setup()
float Fs = 44100;

/* Drum samples are pre-loaded in these buffers. Length of each
 * buffer is given in gDrumSampleBufferLengths.
 */
extern float *gDrumSampleBuffers[NUMBER_OF_DRUMS];
extern int gDrumSampleBufferLengths[NUMBER_OF_DRUMS];

/* Whether or not the loop is playing. */
int gIsPlaying = 1;

/* Two arrays, one holding each read pointer, the
 * other saying which buffer each read pointer corresponds to.
 */
int gReadPointers[NUMBER_OF_VOICES];
int gDrumBufferForReadPointer[NUMBER_OF_VOICES];

/* Patterns indicate which drum(s) should play on which beat.
 * Each element of gPatterns is an array, whose length is given
 * by gPatternLengths.
 */
extern int *gPatterns[NUMBER_OF_PATTERNS];
extern int gPatternLengths[NUMBER_OF_PATTERNS];

/* These variables indicate which pattern we're playing, and
 * where within the pattern we currently are.
 */
int gCurrentPattern = 0;
int gCurrentIndexInPattern = 0;

/* These variables are used in determining when the next event should be started.
 */
int gEventIntervalMilliseconds = 200;
int gEventIntervalSamples = -1;
int gSamplesSinceLastEvent = -1;

/* This indicates whether we should play the samples backwards.
 */
int gPlaysBackwards = 0;

/* For bonus step only: these variables help implement a fill
 * (temporary pattern) which is triggered by tapping the board.
 */
int gShouldPlayFill = 0;
int gPreviousPattern = 0;

/*Variables used in determining and debouncing button presses.
*/
int pausePlayButtonPin = 0;
int pausePlayButtonLastValue = 0;
int pausePlayButtonDebounceCounter = 0;
float maxDebounceCountSeconds = 0.02; //Debounce interval of 20ms.
int maxDebounceCountSamples = 882; // This is also recalculated in setup() using the system sample rate.
enum {
	kStateOpen = 0,
	kStateJustClosed,
	kStateClosed,
	kStateJustOpen
};
int pausePlayButtonDebounceState = kStateOpen;

/*Variables used in flashing the LED.
*/
int gLedPin = 1;
int gLedState = 0;
float gLedOnSeconds = 0.1; //Led flashes on for 100ms.
int gLedOnSamples = 441; //Recalculated in setup() using the system's sample rate.
int gSamplesSinceLEDOn = 0;

//Potentiometer pin.
int potentiometerPin = 0;

//Accelerometer pins
int accelXPin = 2;
int accelYPin = 3;
int accelZPin = 4;

//Moving average filters
#define MOVING_AVERAGE_ORDER 501 //Need quite a high order on this to stop the drum fills' samples playing in reverse.
float X_prev[MOVING_AVERAGE_ORDER];
int X_prev_readPtr = 0;
float Y_prev[MOVING_AVERAGE_ORDER];
int Y_prev_readPtr = 0;
float Z_prev[MOVING_AVERAGE_ORDER];
int Z_prev_readPtr = 0;

//HPF coefficients.
float a_0 = 0, a_1 = 0, a_2 = 0;
float b_0 = 0, b_1 = 0, b_2 = 0;
//Input state variables
float x0 = 0, x1 = 0, x2 = 0;
//First filter output state variables. Same as second filter's input state.
float y10 = 0, y11 = 0, y12 = 0;
//Second filter output state variables.
float y20 = 0, y21 = 0, y22 = 0;

//Used so the accelerometer HPF doesn't go wild and set off a drum fill at the start of the program.
unsigned int gSamplesSinceStartOfProgram = 0;
int gFirstNSamples = 1000;

/**
 * Calculate second order high pass filter coefficients for use in detecting accelerometer hits.
 */
void calculate_hpf_coefficients(float T, float w, float q)
{
	//HPF coefficients:
	a_0 = (4/(T*T)) + ((2*w)/(q*T)) + (w*w);
	a_1 = (2*w*w) - (8/(T*T));
	a_2 = (4/(T*T)) - ((2*w)/(q*T)) + (w*w);
	b_0 = 4/(T*T);
	b_1 = -8/(T*T);
	b_2 = 4/(T*T);
}
/**
 * Use a fourth order high pass filter to detect taps in the Z direction.
 * Code to do this has been taken fromAssignment 1.
 * These x's/y's do not refer to acceleration values along those axes,
 * but to the filter input/output states.
 */
void detectAccelerometerTapAndStartDrumFill(float z_val) {
	/**
     * FILTER 1
     */
	float x0 = z_val;
	float y10 = ((x0*b_0 + x1*b_1 + x2*b_2) - (y11*a_1 + y12*a_2))/a_0;
	/**
     * FILTER 2
     * Remember input to second filter is output of first filter - y1n
     */
	float y20 = ((y10*b_0 + y11*b_1 + y12*b_2) - (y21*a_1 + y22*a_2))/a_0;
	//output of filter 2 is the filtered Z value.
	float filteredZ = y20;
	
	//update the first filter's input/output state for the next iteration.
	x2 = x1;
	x1 = x0;
	y12 = y11;
	y11 = y10;
	//update the second filter's output state.
	y22 = y21;
	y21 = y20;
	
	//If the HPF output is above a certain threshold (experimentally found), then it's been tapped.
	//Looking after first N samples as the HPF gets a bit funky at the start.
	if (filteredZ > 0.25 && gSamplesSinceStartOfProgram > gFirstNSamples) {
		if (gIsPlaying && !gShouldPlayFill) {
			rt_printf("Tapped! %.2f\n", filteredZ);
			
			//Keep track of the previous drum pattern so we can return to it after the fill has finished.
			gPreviousPattern = gCurrentPattern;
	
			//Change the currently playing pattern to a drum fill
			gCurrentPattern = FILL_PATTERN;
			
			//Set the drum fill state variable.
			gShouldPlayFill = 1;
			
			//Because some drum loops are longer than the fill, make sure the fill is in range.
			// This only really works nicely if all loops are 8/16/32 events long.
			gCurrentIndexInPattern = gCurrentIndexInPattern % gPatternLengths[gCurrentPattern];
		}
	}
	gSamplesSinceStartOfProgram++;
}

// setup() is called once before the audio rendering starts.
// Use it to perform any initialisation and allocation which is dependent
// on the period size or sample rate.
//
// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.

bool setup(BelaContext *context, void *userData)
{
	//Initialise the voice arrays.
	for (int voice=0; voice<NUMBER_OF_VOICES; voice++) {
		//A value of -1 indicates the voice is not being used for a drum sound.
		gReadPointers[voice] = -1;
		gDrumBufferForReadPointer[voice] = -1;
	}
	
	//Work out variables that require the system's sample rate.
	Fs = context->audioSampleRate;
	maxDebounceCountSamples = maxDebounceCountSeconds*Fs;
	gEventIntervalSamples = 0.001*gEventIntervalMilliseconds*Fs;
	gLedOnSamples = gLedOnSeconds*Fs;
	
	//Set this to be the maximum value so we start off by playing a drum.
	gSamplesSinceLastEvent = gEventIntervalSamples;
	
	// Initialise GPIO pins
	pinMode(context, 0, pausePlayButtonPin, INPUT);
	pinMode(context, 0, gLedPin, OUTPUT);
	
	//HPF with cutoff of 10Hz, remembering the analog sampling rate is 22.05kHz.
	calculate_hpf_coefficients(1/22050.0, 10*2*M_PI, 1/sqrt(2));
	
	return true;
}

/**
 * Update the current pattern based on accelerometer values, 
 * and ensure that we're at a valid point within the pattern.
 */
void detectOrientationAndUpdatePattern(float x_acc, float y_acc, float z_acc) {
	gCurrentPattern = getPatternFromAccelerationValues(x_acc, y_acc, z_acc);
	
	//If the pattern's changed, make sure we're still at a valid point in the pattern
	//so that the beat carries on. This works nicely when every beat is a multiple of 4 in length.
	gCurrentIndexInPattern = gCurrentIndexInPattern % gPatternLengths[gCurrentPattern];
}

/**
 * Simple moving average filter to low pass filter the board's orientation.
 * Uses a circular buffer for each axis.
 */
float movingAverage(char axis, float newVal) {
	float sum = 0.0;
	switch (axis) {
		case 'X':
			X_prev[X_prev_readPtr] = newVal;
			X_prev_readPtr = (X_prev_readPtr + 1) % MOVING_AVERAGE_ORDER;
			for (int i=0; i<MOVING_AVERAGE_ORDER; i++) {
				sum += X_prev[i];
			}
			return sum/MOVING_AVERAGE_ORDER;
		case 'Y':
			Y_prev[Y_prev_readPtr] = newVal;
			Y_prev_readPtr = (Y_prev_readPtr + 1) % MOVING_AVERAGE_ORDER;
			for (int i=0; i<MOVING_AVERAGE_ORDER; i++) {
				sum += Y_prev[i];
			}
			return sum/MOVING_AVERAGE_ORDER;
			
		case 'Z':
			Z_prev[Z_prev_readPtr] = newVal;
			Z_prev_readPtr = (Z_prev_readPtr + 1) % MOVING_AVERAGE_ORDER;
			for (int i=0; i<MOVING_AVERAGE_ORDER; i++) {
				sum += Z_prev[i];
			}
			return sum/MOVING_AVERAGE_ORDER;
	}
	rt_printf("Unexpected Axis for movingAverage(): %c\n", axis);
	return -1;
}

/**
 * Read potentiometer to determine the event interval size (and therefore tempo)
 */
 void readPotentiometerAndSetTempo(BelaContext *context, int n) {
	float potentiometerVal_V = analogRead(context, n/2, potentiometerPin);
	//Val ranges from 0:(3.3/4.096)
	//Map this value from 50ms-300ms
	gEventIntervalMilliseconds = map(potentiometerVal_V, 0, 3.3/4.096, 50, 300);
	gEventIntervalSamples = 0.001 * Fs * gEventIntervalMilliseconds;
 }

/**
 * Keep track of the button's state using a state machine to aid with debouncing.
 */
void handleButtonPressedStartStop(BelaContext *context, int n) {
	int buttonValue = digitalRead(context, n/2, pausePlayButtonPin);
	
	if(pausePlayButtonDebounceState == kStateOpen) {
   		// Button is open. Could be pressed whenever.
   		if (buttonValue == 0 && pausePlayButtonLastValue == 1) {
   			//Button pressed! Pause / Resume the drum loop.
   			gIsPlaying = 1 - gIsPlaying;
   			pausePlayButtonDebounceState = kStateJustClosed;
   		}
   	}
   	else if(pausePlayButtonDebounceState == kStateJustClosed) {
   		// Button was just pressed, wait for debounce counter.
   		pausePlayButtonDebounceCounter++;
   		if (pausePlayButtonDebounceCounter >= maxDebounceCountSamples) {
   			pausePlayButtonDebounceState = kStateClosed;
   			pausePlayButtonDebounceCounter = 0;
   		}
   	}
   	else if(pausePlayButtonDebounceState == kStateClosed) {
   		// Button is pressed, wait for re-opening.
   		if (buttonValue == 1 && pausePlayButtonLastValue == 0) {
   			pausePlayButtonDebounceState = kStateJustOpen;
   		}
   	}
   	else if(pausePlayButtonDebounceState == kStateJustOpen) {
   		// Button was just released, wait for debounce counter
   		pausePlayButtonDebounceCounter++;
   		if (pausePlayButtonDebounceCounter >= maxDebounceCountSamples) {
   			pausePlayButtonDebounceState = kStateOpen;
   			pausePlayButtonDebounceCounter = 0;
   		}
   	}

   	// Update the previous button value
   	pausePlayButtonLastValue = buttonValue;
}

void playNextEventAndFlashLedIfNecessary(BelaContext *context, int n) {
	gSamplesSinceLastEvent++;
	
	if (gLedState == 1) {
		//LED stays on for gLedOnSamples
		if (gSamplesSinceLEDOn >= gLedOnSamples) {
			gLedState = 0;
			digitalWrite(context, n, gLedPin, gLedState);
		} else {
			gSamplesSinceLEDOn++;
		}
	}
	
	if (gSamplesSinceLastEvent >= gEventIntervalSamples) {
		startNextEvent();
		
		//Blink the LED every fourth event.
		//A remainder of 1 is used here as the event index
		//counter has already been updated in startNextEvent().
		if (gCurrentIndexInPattern % 4 == 1) {
			gLedState = 1;
			gSamplesSinceLEDOn = 0;
		} else {
			gLedState = 0;
		}
		digitalWrite(context, n, gLedPin, gLedState);
	}
}

// render() is called regularly at the highest priority by the audio engine.
// Input and output are given from the audio hardware and the other
// ADCs and DACs (if available). If only audio is available, numMatrixFrames
// will be 0.
void render(BelaContext *context, void *userData)
{
	for(unsigned int n = 0; n < context->audioFrames; n++) {
		//If the button's been pressed, then start/stop the loop playback.
		handleButtonPressedStartStop(context, n);
		
		// Read in the accelerometer values at half the audio sampling rate.
		float x_accel = analogRead(context, n/2, accelXPin);
		float y_accel = analogRead(context, n/2, accelYPin);
		float z_accel = analogRead(context, n/2, accelZPin);
		
		//Detect a hit of the accelerometer in the z direction (Before it's been LPF'd).
		detectAccelerometerTapAndStartDrumFill(z_accel);
		
		//Perform simple moving moving average filter on accelerometer values.
		x_accel = movingAverage('X', x_accel);
		y_accel = movingAverage('Y', y_accel);
		z_accel = movingAverage('Z', z_accel);
		
		if (gIsPlaying) {
			//If we're not playing a fill, detect orientation, and update the current pattern accordingly.
			if (!gShouldPlayFill) detectOrientationAndUpdatePattern(x_accel, y_accel, z_accel);
			
			//Set the tempo based on the potentiometer reading.
			readPotentiometerAndSetTempo(context, n);
			
			//Work out if the next drum event needs to be played and whether the LED should be on or off.
			playNextEventAndFlashLedIfNecessary(context, n);
		}
		
		float out = 0.0;
		/**
		 * Read the next sample for each voice that is currently reading from a drum buffer.
		 * We want to do this even if the pattern is not playing, so we finish off any drum
		 * sounds that were in the middle of playback.
		 */
		for (int voice=0; voice<NUMBER_OF_VOICES; voice++) {
			//Get a pointer to the readPointer for each voice.
			int *voiceReadPointer = &gReadPointers[voice];
			if (*voiceReadPointer >= 0) {
				//this voice is currently reading from a drum buffer. So let's read a sample!
				
				//Which drum buffer is this voice reading from
				int drumBuffer = gDrumBufferForReadPointer[voice];
				
				//Length of this voice's drum sound
				int lengthOfDrumSample = gDrumSampleBufferLengths[drumBuffer];
				
				//Always play the samples forward for the drum fill.
				if (!gPlaysBackwards) {
					//Get the current sample from the drum buffer and add it to the audio output.
					out += gDrumSampleBuffers[drumBuffer][*voiceReadPointer];
				}
				else {
					//In reverse, we just need to take the sample that's the same distance from the end
					// as voiceReadPointer is from the beginning.
					out += gDrumSampleBuffers[drumBuffer][lengthOfDrumSample-(*voiceReadPointer)-1];
				}
				
				//increment current voice's readPointer for the next loop.
				(*voiceReadPointer)++;
				
				//If this voice's readPointer has reached the end of a loop,
				//then set it back to -1 (not reading a drum buffer)
				if (*voiceReadPointer >= lengthOfDrumSample) {
					*voiceReadPointer = -1;
					gDrumBufferForReadPointer[voice] = -1;
				}
			}
		}
		
		for(unsigned int channel = 0; channel < context->audioOutChannels; channel++) {
			audioWrite(context, n, channel, out);
		}
	}
	
}

/** 
 * Start playing a particular drum sound given by drumIndex.
 */
void startPlayingDrum(int drumIndex) {
	for (int voice=0; voice < NUMBER_OF_VOICES; voice++) {
		//Look through all the voices to find a free one.
		if (gReadPointers[voice] == -1) {
			//found a free buffer! Start using it.
			gDrumBufferForReadPointer[voice] = drumIndex;
			gReadPointers[voice] = 0;
			return;
		}
	}
}

/**
 * Start playing the next event in the pattern
 */
void startNextEvent() {
	for (int drum=0; drum<NUMBER_OF_DRUMS; drum++) {
		//if this drum is contained in the current event of the chosen pattern, play it.
		int currentEvent = gPatterns[gCurrentPattern][gCurrentIndexInPattern];
		if (eventContainsDrum(currentEvent, drum) == 1) {
			startPlayingDrum(drum);
		}
	}
	
	//Increment the index in the current pattern.
	gCurrentIndexInPattern++;
	//If we've reached the end, wrap round to the beginning, and stop playing a drum fill if necessary.
	if (gCurrentIndexInPattern >= gPatternLengths[gCurrentPattern]) {
		if (gShouldPlayFill) {
			gShouldPlayFill = 0;
			gCurrentPattern = gPreviousPattern;
		}
		gCurrentIndexInPattern = 0;
	}
	
	gSamplesSinceLastEvent = 0;
}

/**
 * Returns whether the given event contains the given drum sound
 */
int eventContainsDrum(int event, int drum) {
	if(event & (1 << drum))
		return 1;
	return 0;
}

// cleanup_render() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in initialise_render().

void cleanup(BelaContext *context, void *userData)
{

}

/**
 * Returns drum pattern index 0:4 depending on accelerometer values.
 * 
 * Acceleration values for each orientation were experimentally found and then
 * suitable ranges constructed.
 * E.g. a zero y acceleration was recorded as 0.45, and ±g as 0.45±0.2
 *		Therefore 0.45±0.1 are chosen as the cutoffs for each of those two orientations.
 */
int getPatternFromAccelerationValues(float x_accel, float y_accel, float z_accel) {
	
	//If it's upside down, play the currently playing beat backwards.
	if (z_accel < 0.25) {
		gPlaysBackwards = true;
		return gCurrentPattern;
	}
	gPlaysBackwards = false;
	
	if (y_accel < 0.35) return 1;
	
	if (y_accel > 0.55) return 2;
	
	if (x_accel < 0.31) return 3;
	
	if (x_accel > 0.51) return 4;
	
	return 0;
}
