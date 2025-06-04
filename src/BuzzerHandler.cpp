/**
 * @file BuzzerHandler.cpp
 * @brief Implementation of BuzzerHandler for audio notifications and music
 */

#include "BuzzerHandler.h"

// =====================================================================
// SONG DATA - Popular melodies and notification sounds
// =====================================================================

// Startup melody - Ascending scale
const int BuzzerHandler::startupMelody[] = {
  NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4, NOTE_C5
};
const int BuzzerHandler::startupDurations[] = {4, 4, 4, 4, 4, 4, 4, 2};

// Launch detected - Exciting fanfare
const int BuzzerHandler::launchMelody[] = {
  NOTE_C5, NOTE_G4, NOTE_C5, NOTE_G4, NOTE_C5, NOTE_E5, NOTE_G5
};
const int BuzzerHandler::launchDurations[] = {8, 8, 8, 8, 4, 4, 2};

// Apogee detected - Victory fanfare
const int BuzzerHandler::apogeeMelody[] = {
  NOTE_C5, NOTE_C5, NOTE_C5, NOTE_C5, NOTE_G4, NOTE_A4, NOTE_C5
};
const int BuzzerHandler::apogeeDurations[] = {8, 8, 8, 2, 4, 4, 2};

// Error warning - Alert pattern
const int BuzzerHandler::errorMelody[] = {
  NOTE_A5, REST, NOTE_A5, REST, NOTE_A5, REST, NOTE_A5
};
const int BuzzerHandler::errorDurations[] = {8, 8, 8, 8, 8, 8, 4};

// Super Mario Bros Theme (opening) - Popular classic
const int BuzzerHandler::marioMelody[] = {
  NOTE_E5, NOTE_E5, REST, NOTE_E5, REST, NOTE_C5, NOTE_E5, REST,
  NOTE_G5, REST, REST, REST, NOTE_G4, REST, REST, REST,
  NOTE_C5, REST, REST, NOTE_G4, REST, REST, NOTE_E4, REST,
  REST, NOTE_A4, REST, NOTE_B4, REST, NOTE_AS4, NOTE_A4, REST,
  NOTE_G4, NOTE_E5, NOTE_G5, NOTE_A5, REST, NOTE_F5, NOTE_G5,
  REST, NOTE_E5, REST, NOTE_C5, NOTE_D5, NOTE_B4
};
const int BuzzerHandler::marioDurations[] = {
  8, 4, 8, 4, 8, 8, 4, 8,
  4, 8, 8, 8, 4, 8, 8, 8,
  4, 8, 8, 4, 8, 8, 4, 8,
  8, 4, 8, 4, 8, 8, 4, 8,
  8, 8, 8, 4, 8, 8, 4,
  8, 4, 8, 8, 8, 4
};

// Star Wars Imperial March - Epic classic
const int BuzzerHandler::starWarsMelody[] = {
  NOTE_A4, NOTE_A4, NOTE_A4, NOTE_F4, NOTE_C5,
  NOTE_A4, NOTE_F4, NOTE_C5, NOTE_A4, REST,
  NOTE_E5, NOTE_E5, NOTE_E5, NOTE_F5, NOTE_C5,
  NOTE_GS4, NOTE_F4, NOTE_C5, NOTE_A4, REST
};
const int BuzzerHandler::starWarsDurations[] = {
  4, 4, 4, -4, 16,
  4, -4, 16, 2, 8,
  4, 4, 4, -4, 16,
  4, -4, 16, 2, 4
};

// Tetris Theme (Korobeiniki) - Addictive game music
const int BuzzerHandler::tetrisMelody[] = {
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4,
  NOTE_A4, NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_C5, NOTE_A4, NOTE_A4, REST,
  NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_E5, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5,
  NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5,
  NOTE_C5, NOTE_A4, NOTE_A4
};
const int BuzzerHandler::tetrisDurations[] = {
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  -4, 8, 4, 4,
  4, 4, 8, 4,
  -4, 8, 4, 8, 8,
  -4, 8, 4, 8, 8,
  4, 8, 8, 4, 4,
  4, 4, 4
};

// Nokia Ringtone - Iconic mobile sound
const int BuzzerHandler::nokiaMelody[] = {
  NOTE_E5, NOTE_D5, NOTE_FS4, NOTE_GS4,
  NOTE_CS5, NOTE_B4, NOTE_D4, NOTE_E4,
  NOTE_B4, NOTE_A4, NOTE_CS4, NOTE_E4,
  NOTE_A4
};
const int BuzzerHandler::nokiaDurations[] = {
  8, 8, 4, 4,
  8, 8, 4, 4,
  8, 8, 4, 4,
  2
};

// =====================================================================
// IMPLEMENTATION
// =====================================================================

BuzzerHandler::BuzzerHandler(int pin) : buzzerPin(pin), playing(false), 
  lastNoteTime(0), currentNoteIndex(0), currentMelody(nullptr), 
  currentDurations(nullptr), currentMelodyLength(0), currentTempo(120) {
}

void BuzzerHandler::initialize() {
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);
  playing = false;
}

void BuzzerHandler::playSong(SongType song) {
  switch (song) {
    case SongType::STARTUP:
      playStartupMelody();
      break;
    case SongType::LAUNCH_DETECTED:
      playLaunchDetected();
      break;
    case SongType::APOGEE_DETECTED:
      playApogeeDetected();
      break;
    case SongType::ERROR_WARNING:
      playErrorWarning();
      break;
    case SongType::MARIO_THEME:
      playMarioTheme();
      break;
    case SongType::STAR_WARS:
      playStarWars();
      break;
    case SongType::TETRIS:
      playTetris();
      break;
    case SongType::NOKIA_RINGTONE:
      playNokiaRingtone();
      break;
  }
}

void BuzzerHandler::playStartupMelody() {
  playMelody(startupMelody, startupDurations, 
             sizeof(startupMelody)/sizeof(startupMelody[0]), 120);
}

void BuzzerHandler::playLaunchDetected() {
  playMelody(launchMelody, launchDurations, 
             sizeof(launchMelody)/sizeof(launchMelody[0]), 140);
}

void BuzzerHandler::playApogeeDetected() {
  playMelody(apogeeMelody, apogeeDurations, 
             sizeof(apogeeMelody)/sizeof(apogeeMelody[0]), 130);
}

void BuzzerHandler::playErrorWarning() {
  playMelody(errorMelody, errorDurations, 
             sizeof(errorMelody)/sizeof(errorMelody[0]), 180);
}

void BuzzerHandler::playMarioTheme() {
  playMelody(marioMelody, marioDurations, 
             sizeof(marioMelody)/sizeof(marioMelody[0]), 144);
}

void BuzzerHandler::playStarWars() {
  playMelody(starWarsMelody, starWarsDurations, 
             sizeof(starWarsMelody)/sizeof(starWarsMelody[0]), 120);
}

void BuzzerHandler::playTetris() {
  playMelody(tetrisMelody, tetrisDurations, 
             sizeof(tetrisMelody)/sizeof(tetrisMelody[0]), 144);
}

void BuzzerHandler::playNokiaRingtone() {
  playMelody(nokiaMelody, nokiaDurations, 
             sizeof(nokiaMelody)/sizeof(nokiaMelody[0]), 120);
}

void BuzzerHandler::playMelody(const int melody[], const int durations[], int length, int tempo) {
  int wholenote = (60000 * 4) / tempo;
  
  for (int thisNote = 0; thisNote < length; thisNote++) {
    int divider = durations[thisNote];
    int noteDuration = 0;
    
    if (divider > 0) {
      // Regular note
      noteDuration = wholenote / divider;
    } else if (divider < 0) {
      // Dotted notes (negative duration)
      noteDuration = wholenote / abs(divider);
      noteDuration *= 1.5; // Increase duration by half for dotted notes
    }
    
    // Play the note for 90% of the duration, leaving 10% as pause
    if (melody[thisNote] != REST) {
      tone(buzzerPin, melody[thisNote], noteDuration * 0.9);
    }
    
    // Wait for the specified duration before playing the next note
    delay(noteDuration);
    
    // Stop the waveform generation before the next note
    noTone(buzzerPin);
  }
}

void BuzzerHandler::playNote(int frequency, int duration) {
  if (frequency > 0) {
    tone(buzzerPin, frequency, duration);
  }
  delay(duration);
  noTone(buzzerPin);
}

void BuzzerHandler::playTone(int frequency, int duration) {
  if (frequency > 0) {
    tone(buzzerPin, frequency, duration);
  } else {
    delay(duration);
  }
}

void BuzzerHandler::stopTone() {
  noTone(buzzerPin);
  playing = false;
}

bool BuzzerHandler::isPlaying() {
  return playing;
} 