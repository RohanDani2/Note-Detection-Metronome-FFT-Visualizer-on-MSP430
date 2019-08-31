#ifndef SOUND_H_
#define SOUND_H_

#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

typedef enum {playing, notPlaying} NoteStatus;

// In this enum, we are simply giving a name to each note so that we can work with them easier.
// We are creating a new type using this enum called basic_note_t.
// Whenever we say basic_note_t x, we mean x is simply a number that represent a certain basic note.
typedef enum {  note_silent,
                note_c0, note_d0, note_e0, note_f0, note_g0, note_a0, note_b0,
                note_f0F,
                note_b0F,
                note_c1, note_d1, note_e1, note_f1, note_g1, note_a1, note_b1,
                note_c2, note_d2, note_e2, note_f2, note_g2, note_a2, note_b2,
                note_c3, note_d3, note_e3, note_f3, note_g3, note_a3, note_b3,
                note_c4, note_d4, note_e4, note_f4, note_g4, note_a4, note_b4,
                note_c5, note_d5, note_e5, note_f5, note_g5, note_a5, note_b5,
                note_f5S,
                note_f4S,
                note_g4F,
                note_c6 }
basic_note_t;

/* This structure consists of a basic note and its length. Imagine a music sheet, where each note tells you the type (C4) and its length (quarter-note).
 * We call this structure song_note_t, because it contains the information for a note in a song. In our structure the length is specified in terms of ms.
 * For example if we say song_note_t myNote = {note_c4, 100}, it means myNote should be played as the note C4 for 100 ms which is equal to 1/16th note in rhythm.
 */
typedef struct {
    basic_note_t  note_name;
    unsigned int  note_length; // note length in ms
} song_note_t;

// This function configures the pin that drives the buzzer with a PWM
void InitSound();

// These two functions generate the PWM that makes the note

// blocking version
// It plays the whole note while blocking all other operations
void PlayNote_blocking(song_note_t songNote);


// non-blocking version
// It returns true if the note has played for the required length (duration), otherwise it returns false.
bool PlayNote_nonblocking(song_note_t songNote);

#endif /* SOUND_H_ */
