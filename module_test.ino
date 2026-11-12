#include "display.h"
#include "speaker.h"
#include <Arduino.h>
#include <stdlib.h>
#include <string.h>

struct FaceName { const char* name; face_t value; };
const FaceName faces[]={
{"idle",FACE_IDLE},
{"happy",FACE_HAPPY},
{"shy",FACE_SHY},
{"angry",FACE_ANGRY},
{"waking",FACE_WAKING},
{"listening",FACE_LISTENING},
{"confused",FACE_CONFUSED},
{"sitting",FACE_SITTING},
{"study",FACE_STUDY},
{"lifted",FACE_LIFTED},
{"flipped",FACE_FLIPPED},
{"edge",FACE_EDGE},
{"low_battery",FACE_LOW_BATTERY},
{"sleep",FACE_SLEEP},
{"charging",FACE_CHARGING},
{"no_connection",FACE_NO_CONNECTION},
{"error",FACE_ERROR},
{"ack",FACE_ACK}
};
struct SoundName { const char* name; sound_t value; };
const SoundName sounds[]={
{"bark",SOUND_BARK},
{"bark_short",SOUND_BARK_SHORT},
{"confused",SOUND_CONFUSED},
{"ack",SOUND_ACK},
{"timer_done",SOUND_TIMER_DONE},
{"low_battery",SOUND_LOW_BATTERY},
{"wake",SOUND_WAKE},
{"hello",SOUND_HELLO},
{"listening",SOUND_LISTENING},
{"study_start",SOUND_STUDY_START},
{"pause",SOUND_PAUSE},
{"resume",SOUND_RESUME},
{"charging",SOUND_CHARGING},
{"flipped",SOUND_FLIPPED},
{"upright",SOUND_UPRIGHT},
{"angry",SOUND_ANGRY},
{"forgive",SOUND_FORGIVE},
{"happy",SOUND_HAPPY}
};
bool displayOK=false, speakerOK=false;
void help() {
  Serial.println("face NAME | sound NAME | timer SECONDS | volume 0..100 | stop | status | help");
  Serial.println("Faces:");
  for (const auto &f:faces) Serial.println(f.name);
  Serial.println("Sounds:");
  for (const auto &s:sounds) Serial.println(s.name);
}
bool parseNumber(const char *p, uint32_t limit, uint32_t &out) {
  if (!*p) return false;
  out=0;
  for (;*p;p++) { if (*p<'0'||*p>'9') return false; out=out*10+(*p-'0'); if(out>limit)return false; }
  return true;
}
void command(String line) {
  line.trim(); line.toLowerCase();
  const char *p=line.c_str(); uint32_t n;
  if (!line.length()) return;
  if (line=="help") { help(); return; }
  if (line=="stop") { speaker_stop(); return; }
  if (line=="status") { Serial.println(speaker_is_playing()?"Speaker busy":"Speaker idle"); return; }
  if (strncmp(p,"face ",5)==0) {
    for (const auto &f:faces) if(strcmp(p+5,f.name)==0) { display_set_face(f.value);return; }
  }
  if (strncmp(p,"sound ",6)==0) {
    for (const auto &s:sounds) if(strcmp(p+6,s.name)==0) { speaker_play(s.value);return; }
  }
  if (strncmp(p,"timer ",6)==0 && parseNumber(p+6,5999,n)) { display_show_timer(n); return; }
  if (strncmp(p,"volume ",7)==0 && parseNumber(p+7,100,n)) { speaker_set_volume(n); return; }
  Serial.println("Unknown command. Type help.");
}
void setup() {
  Serial.begin(115200);
  displayOK=display_init(); speakerOK=speaker_init();
  Serial.println(displayOK?"Display ready":"Display init failed");
  Serial.println(speakerOK?"Speaker ready":"Speaker init failed");
  speaker_set_volume(10);
  help();
}
void loop() {
  static String line; static bool overflow=false;
  for (int i=0;i<64 && Serial.available();i++) {
    char c=(char)Serial.read();
    if(c=='\n'||c=='\r') {
      if(!overflow)command(line);else Serial.println("Command too long.");
      line="";overflow=false;
    } else if(!overflow) { if(line.length()<48)line+=c;else overflow=true; }
  }
  display_update();
  delay(1);
}
