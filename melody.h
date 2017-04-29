#include "pitches.h"

// tempo for the melody expressed in beats per minute (BPM)
int tempo = 120;

// Piezzo element connected to Arduino pin 12 and ground
int buzzerPin = 11;

// Array with the notes in the melody (see pitches.h for reference)
int melody[] = {
  NOTE_C3,NOTE_D3,NOTE_E3,
  NOTE_F3,NOTE_F3,NOTE_F3,
  NOTE_F3,NOTE_E3,NOTE_D3,
  NOTE_E3,NOTE_E3,NOTE_E3,
  NOTE_E3,NOTE_D3,NOTE_C3,
  NOTE_D3,NOTE_D3,NOTE_D3,
  NOTE_D3,NOTE_E3,NOTE_D3,
  NOTE_C3,NOTE_C3,NOTE_C3
};

// Array with the note durations: a quarter note has a duration of 4, half note 2 etc.
int durations[]  = {
  8,8,4,
  4,8,8,
  8,8,4,
  4,8,8,
  8,8,4,
  4,8,8,
  8,8,4,
  4,8,8
};
