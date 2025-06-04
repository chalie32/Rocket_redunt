/**
 * @file BuzzerHandler.h
 * @brief Buzzer handler for audio notifications and music playback
 * 
 * Provides various melodies and notification sounds for different flight events:
 * - Startup melody
 * - Launch detection
 * - Apogee detection  
 * - Error/warning sounds
 * - Popular music themes
 */

#ifndef BUZZER_HANDLER_H
#define BUZZER_HANDLER_H

#include <Arduino.h>

// Note frequency definitions
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define REST     0

// Song types
enum class SongType {
  STARTUP,
  LAUNCH_DETECTED,
  APOGEE_DETECTED,
  ERROR_WARNING,
  MARIO_THEME,
  STAR_WARS,
  TETRIS,
  NOKIA_RINGTONE
};

class BuzzerHandler {
public:
  BuzzerHandler(int pin);
  void initialize();
  
  // Main playback functions
  void playSong(SongType song);
  void playStartupMelody();
  void playLaunchDetected();
  void playApogeeDetected();
  void playErrorWarning();
  void playMarioTheme();
  void playStarWars();
  void playTetris();
  void playNokiaRingtone();
  
  // Utility functions
  void playTone(int frequency, int duration);
  void stopTone();
  bool isPlaying();
  
private:
  int buzzerPin;
  bool playing;
  unsigned long lastNoteTime;
  int currentNoteIndex;
  const int* currentMelody;
  const int* currentDurations;
  int currentMelodyLength;
  int currentTempo;
  
  // Helper functions
  void playMelody(const int melody[], const int durations[], int length, int tempo);
  void playNote(int frequency, int duration);
  
  // Song data arrays
  static const int startupMelody[];
  static const int startupDurations[];
  static const int launchMelody[];
  static const int launchDurations[];
  static const int apogeeMelody[];
  static const int apogeeDurations[];
  static const int errorMelody[];
  static const int errorDurations[];
  static const int marioMelody[];
  static const int marioDurations[];
  static const int starWarsMelody[];
  static const int starWarsDurations[];
  static const int tetrisMelody[];
  static const int tetrisDurations[];
  static const int nokiaMelody[];
  static const int nokiaDurations[];
};

#endif // BUZZER_HANDLER_H 