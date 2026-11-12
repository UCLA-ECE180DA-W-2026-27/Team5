#include "state_machine.h"
#include "display.h"
#include "speaker.h"
#include <Arduino.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

namespace {
enum Mode { IDLE, WAKING, LISTENING, ACK, CONFUSED, SITTING, SPEAKING,
            SHY, HAPPY, ANGRY, SLEEP, STUDY };
enum View { V_NONE, V_IDLE, V_WAKING, V_LISTENING, V_ACK, V_CONFUSED,
  V_SITTING, V_SPEAKING, V_SHY, V_HAPPY, V_ANGRY, V_SLEEP, V_STUDY,
  V_FLIPPED, V_LIFTED, V_EDGE, V_ERROR, V_CHARGING, V_LOW_BATTERY, V_OFFLINE };
enum Action { A_NONE, A_SIT, A_SPEAK, A_STUDY, A_PAUSE, A_RESUME, A_STOP };
Mode mode=IDLE;
View view=V_NONE;
bool flipped=false, lifted=false, edge=false, fault=false, offline=false;
bool lowBattery=false, docked=false, angry=false, faceVisible=false;
bool study=false, manualPause=false, frozen=false;
bool bench=true, initialized=false, modulesOK=false, batterySleep=false;
int faceDirection=0;
uint32_t modeSince=0, viewSince=0, remainingMs=0, runSince=0;
uint32_t lastSeconds=UINT32_MAX, sessionCounter=0, session=0;
uint32_t motionCounter=0, activeSit=0;
Action pending=A_NONE;
uint32_t pendingMinutes=0;
bool audioQuietSeen=false;
uint32_t audioQuietSince=0;
robot_motion_handler_t motionHandler=nullptr;
void (*legacySitHandler)(void)=nullptr;
robot_motion_t lastMotion=ROBOT_MOVE_STOP;
bool motionKnown=false;

bool unsafe() { return flipped||lifted||edge||fault||offline||lowBattery||docked; }
uint32_t remainingNow() {
  if(!study) return 0;
  if(frozen) return remainingMs;
  uint32_t elapsed=millis()-runSince;
  return elapsed>=remainingMs ? 0 : remainingMs-elapsed;
}
void cancelStudy() {
  study=false; manualPause=false; frozen=false; remainingMs=0;
  lastSeconds=UINT32_MAX;
}
void freezeStudy() {
  if(study && !frozen) { remainingMs=remainingNow(); frozen=true; }
}
void thawStudy() {
  if(study && frozen && !manualPause && !unsafe()) { runSince=millis(); frozen=false; }
}
void requestMotion(robot_motion_t request, bool force=false) {
  if(!force && motionKnown && request==lastMotion) return;
  lastMotion=request; motionKnown=true;
  uint32_t id=++motionCounter;
  if(!id) id=++motionCounter;
  if(request==ROBOT_MOVE_SIT) activeSit=id;
  if(motionHandler) motionHandler(request,id);
  else if(request==ROBOT_MOVE_SIT && legacySitHandler) legacySitHandler();
  else {
    Serial.print("SIMULATED motion request: "); Serial.print((int)request);
    Serial.print(" id="); Serial.println(id);
  }
}
void resetAudioWait() { audioQuietSeen=false; audioQuietSince=0; }
void play(sound_t sound) { resetAudioWait(); speaker_play(sound); }
bool audioDrained() {
  if(speaker_is_playing()) { audioQuietSeen=false; return false; }
  if(!audioQuietSeen) { audioQuietSeen=true; audioQuietSince=millis(); return false; }
  // One I2S block is 16 ms; allow buffered output to drain before next sound.
  return millis()-audioQuietSince>=40;
}
void enter(Mode next) {
  mode=next; modeSince=millis();
  if(next!=SHY)faceDirection=0;
  if(next!=LISTENING) session=0;
  if(next!=ACK) { pending=A_NONE; pendingMinutes=0; }
  if(next!=SITTING) activeSit=0;
  resetAudioWait();
}
void restore() {
  thawStudy();
  enter(angry ? ANGRY : study ? STUDY : IDLE);
}
View selectedView() {
  if(flipped) return V_FLIPPED;
  if(lifted) return V_LIFTED;
  if(edge) return V_EDGE;
  if(fault) return V_ERROR;
  if(docked) return V_CHARGING;
  if(lowBattery) return V_LOW_BATTERY;
  if(offline) return V_OFFLINE;
  switch(mode) {
    case WAKING:return V_WAKING; case LISTENING:return V_LISTENING;
    case ACK:return V_ACK; case CONFUSED:return V_CONFUSED;
    case SITTING:return V_SITTING; case SPEAKING:return V_SPEAKING;
    case SHY:return V_SHY; case HAPPY:return V_HAPPY;
    case ANGRY:return V_ANGRY; case SLEEP:return V_SLEEP; case STUDY:return V_STUDY;
    default:return V_IDLE;
  }
}
void render() {
  View next=selectedView();
  if(next!=view) {
    view=next; viewSince=millis(); batterySleep=false; lastSeconds=UINT32_MAX;
    face_t face=FACE_IDLE;
    switch(view) {
      case V_WAKING:face=FACE_WAKING;break;
      case V_LISTENING:face=FACE_LISTENING;break;
      case V_ACK:face=FACE_ACK;break;
      case V_CONFUSED:face=FACE_CONFUSED;break;
      case V_SITTING:face=FACE_SITTING;break;
      case V_SPEAKING:case V_HAPPY:face=FACE_HAPPY;break;
      case V_SHY:face=FACE_SHY;break;
      case V_ANGRY:face=FACE_ANGRY;break;
      case V_SLEEP:face=FACE_SLEEP;break;
      case V_STUDY:face=FACE_STUDY;break;
      case V_FLIPPED:face=FACE_FLIPPED;break;
      case V_LIFTED:face=FACE_LIFTED;break;
      case V_EDGE:face=FACE_EDGE;break;
      case V_ERROR:face=FACE_ERROR;break;
      case V_CHARGING:face=FACE_CHARGING;break;
      case V_LOW_BATTERY:face=FACE_LOW_BATTERY;break;
      case V_OFFLINE:face=FACE_NO_CONNECTION;break;
      default:break;
    }
    display_set_face(face);
  }
  if(view==V_LOW_BATTERY && !batterySleep && millis()-viewSince>=3500) {
    display_set_face(FACE_SLEEP); batterySleep=true;
  }
  if(view==V_STUDY) {
    uint32_t seconds=(remainingNow()+999)/1000;
    if(seconds!=lastSeconds) { display_show_timer(seconds); lastSeconds=seconds; }
  }
  // Normal idle means simulated Free Roam, not proof of physical locomotion.
  robot_motion_t motion=ROBOT_MOVE_STOP;
  if(!unsafe()) {
    if(mode==IDLE && !angry)motion=ROBOT_MOVE_FREE_ROAM;
    else if(mode==SITTING)motion=ROBOT_MOVE_SIT;
    else if(mode==SHY && faceDirection)motion=faceDirection<0?ROBOT_MOVE_TURN_LEFT:ROBOT_MOVE_TURN_RIGHT;
  }
  requestMotion(motion);
}
void invalidate() {
  session=0; pending=A_NONE; pendingMinutes=0; activeSit=0;
  speaker_stop(); resetAudioWait();
  requestMotion(ROBOT_MOVE_STOP);
}
bool normalAllowed() { return !unsafe() && !angry && !study; }
void startStudy(uint32_t minutes) {
  cancelStudy(); study=true; remainingMs=minutes*60000UL; runSince=millis();
  enter(STUDY); play(SOUND_STUDY_START);
}
void execute(Action action,uint32_t minutes=0) {
  switch(action) {
    case A_SIT:
      enter(SITTING); play(SOUND_BARK);
      requestMotion(ROBOT_MOVE_SIT,true); break;
    case A_SPEAK:enter(SPEAKING);play(SOUND_BARK);break;
    case A_STUDY:startStudy(minutes);break;
    case A_PAUSE:
      freezeStudy(); manualPause=true; enter(STUDY);play(SOUND_PAUSE);break;
    case A_RESUME:
      manualPause=false;thawStudy();enter(STUDY);play(SOUND_RESUME);break;
    case A_STOP:
      cancelStudy();invalidate();enter(angry?ANGRY:IDLE);break;
    default:restore();break;
  }
  render();
}
bool number(const char *p,uint32_t max,uint32_t &n) {
  if(!*p) return false;
  n=0;
  for(;*p;p++) {
    if(*p<'0'||*p>'9')return false;
    uint32_t d=*p-'0';
    if(d>max || n>(max-d)/10)return false;
    n=n*10+d;
  }
  return true;
}
bool normalize(const char *input,char *out,size_t capacity) {
  if(!input) return false;
  while(isspace((unsigned char)*input))input++;
  size_t length=strlen(input);
  while(length && isspace((unsigned char)input[length-1]))length--;
  if(!length || length>=capacity)return false;
  for(size_t i=0;i<length;i++)out[i]=(char)tolower((unsigned char)input[i]);
  out[length]=0;return true;
}
bool parseAction(const char *text,Action &action,uint32_t &minutes) {
  minutes=0;
  if(!strcmp(text,"sit"))action=A_SIT;
  else if(!strcmp(text,"speak")||!strcmp(text,"bark"))action=A_SPEAK;
  else if(!strcmp(text,"pause"))action=A_PAUSE;
  else if(!strcmp(text,"resume"))action=A_RESUME;
  else if(!strcmp(text,"stop"))action=A_STOP;
  else if(!strncmp(text,"study ",6) && number(text+6,99,minutes) && minutes>0)action=A_STUDY;
  else return false;
  return true;
}
bool actionAllowed(Action action) {
  if(action==A_STOP)return true;
  if(unsafe()||angry)return false;
  if(action==A_PAUSE)return study&&!manualPause;
  if(action==A_RESUME)return study&&manualPause;
  return !study;
}
void rejected(const char *reason) { Serial.print("Ignored: ");Serial.println(reason); }
void beginFeedback(bool understood,Action action=A_NONE,uint32_t minutes=0) {
  session=0;enter(understood?ACK:CONFUSED);
  pending=understood?action:A_NONE;pendingMinutes=minutes;
  play(understood?SOUND_ACK:SOUND_CONFUSED);render();
}
void help() {
  Serial.println("listen; voice sit | voice speak | voice study 1 | voice pause | voice resume | voice stop");
  Serial.println("Listening results: ack | confused (within 5 seconds). voice RESULT uses the current session.");
  Serial.println("Safety: flip/upright, lifted/lowered, edge/edge_clear, error/error_clear");
  Serial.println("Link: disconnected/connected; power: lowbattery/battery_ok, charging/undocked");
  Serial.println("Interactions: face, face_left, face_right, face_lost, insult, sorry");
  Serial.println("stop cancels actions/timer, but NEVER clears safety, battery, charging or anger locks.");
  Serial.println("Bench only: sit, sit_done, study N, pause, resume, speak, wake, normal, happy, sleepy, shy");
  Serial.println("Bench only: sound NAME, bark_short, wow, duang, ohyeah, laugh, mambo, beep, tone");
  Serial.println("Utilities: status, volume 0..100, mute, bench on/off, help");
}
} // namespace

bool state_machine_init(void) {
  modulesOK=display_init();bool audioOK=speaker_init();modulesOK=modulesOK&&audioOK;
  speaker_set_volume(10);speaker_stop();
  flipped=lifted=edge=offline=lowBattery=docked=angry=faceVisible=false;
  fault=!modulesOK;cancelStudy();view=V_NONE;motionKnown=false;session=0;pending=A_NONE;activeSit=0;
  initialized=true;enter(WAKING);render();
  Serial.println(modulesOK?"Controller ready. Motion/sensors require integration.":"Module initialization failed; ERROR locked.");
  help();return modulesOK;
}
void state_machine_set_motion_handler(robot_motion_handler_t handler) {
  motionHandler=handler;motionKnown=false;
  // New driver starts stopped. Next update applies the current policy.
  requestMotion(ROBOT_MOVE_STOP,true);
}
void state_machine_set_sit_handler(void (*handler)(void)) {legacySitHandler=handler;}
void state_machine_enable_bench(bool enabled) {bench=enabled;}
uint32_t state_machine_active_motion(void) {return activeSit;}
uint32_t state_machine_listening_session(void) {
  return mode==LISTENING && !unsafe() && millis()-modeSince<5000 ? session:0;
}
uint32_t state_machine_begin_listening(void) {
  if(!initialized||unsafe()||angry) {rejected("Listening blocked by safety or anger.");return 0;}
  if(mode==LISTENING && millis()-modeSince<5000)return session;
  // Study stays frozen during recognition and feedback; restore it afterward.
  freezeStudy();invalidate();enter(LISTENING);
  session=++sessionCounter;if(!session)session=++sessionCounter;
  play(SOUND_LISTENING);render();return session;
}
bool state_machine_voice(uint32_t token,const char *text) {
  if(!token || token!=state_machine_listening_session()) {rejected("Stale/inactive listening session.");return false;}
  char command[49];if(!normalize(text,command,sizeof(command)))return false;
  if(!strcmp(command,"confused")) {beginFeedback(false);return true;}
  if(!strcmp(command,"ack")) {beginFeedback(true);return true;}
  Action action;uint32_t minutes;
  if(!parseAction(command,action,minutes) || !actionAllowed(action)) {
    beginFeedback(false);return false;
  }
  if(action==A_STOP) {execute(A_STOP);return true;} // Never delay a stop for feedback.
  beginFeedback(true,action,minutes);return true;
}
void state_machine_event(robot_event_t event) {
  bool before=unsafe(), changed=false, recoverFlip=false;
  View previousView=view;
  // Harmless repeated events must not restart playback or state timers.
  switch(event) {
    case ROBOT_EVENT_FLIPPED:if(flipped)return;freezeStudy();flipped=true;changed=true;break;
    case ROBOT_EVENT_UPRIGHT:if(!flipped)return;flipped=false;changed=true;recoverFlip=true;break;
    case ROBOT_EVENT_LIFTED:if(lifted)return;freezeStudy();lifted=true;changed=true;break;
    case ROBOT_EVENT_LOWERED:if(!lifted)return;lifted=false;changed=true;break;
    case ROBOT_EVENT_EDGE:if(edge)return;freezeStudy();edge=true;changed=true;break;
    case ROBOT_EVENT_EDGE_CLEAR:if(!edge)return;edge=false;changed=true;break;
    case ROBOT_EVENT_ERROR:if(fault)return;freezeStudy();fault=true;changed=true;break;
    case ROBOT_EVENT_ERROR_CLEAR:if(!fault||!modulesOK||speaker_has_error())return;fault=false;changed=true;break;
    case ROBOT_EVENT_DISCONNECTED:if(offline)return;freezeStudy();offline=true;changed=true;break;
    case ROBOT_EVENT_CONNECTED:if(!offline)return;offline=false;changed=true;break;
    case ROBOT_EVENT_LOW_BATTERY:if(lowBattery)return;cancelStudy();lowBattery=true;changed=true;break;
    case ROBOT_EVENT_BATTERY_OK:if(!lowBattery)return;lowBattery=false;changed=true;break;
    case ROBOT_EVENT_CHARGING:if(docked)return;cancelStudy();docked=true;changed=true;break;
    case ROBOT_EVENT_UNDOCKED:if(!docked)return;docked=false;changed=true;break;
    case ROBOT_EVENT_FACE_LOST:faceVisible=false;return;
    case ROBOT_EVENT_FACE_LEFT:
    case ROBOT_EVENT_FACE_RIGHT:
    case ROBOT_EVENT_FACE:
      if(faceVisible || !normalAllowed() || mode!=IDLE)return;
      faceVisible=true;enter(SHY);
      faceDirection=event==ROBOT_EVENT_FACE_LEFT?-1:event==ROBOT_EVENT_FACE_RIGHT?1:0;
      play(SOUND_HELLO);render();return;
    case ROBOT_EVENT_INSULT:
      if(unsafe()||study||angry)return;
      angry=true;invalidate();enter(ANGRY);play(SOUND_ANGRY);render();return;
    case ROBOT_EVENT_APOLOGY:
      if(!angry || unsafe())return;
      angry=false;enter(HAPPY);play(SOUND_FORGIVE);render();return;
    default:return;
  }
  if(!changed)return;
  // Cancel queued recognition/action requests on EVERY safety change.
  invalidate();
  if(unsafe()) {
    freezeStudy();enter(angry?ANGRY:IDLE);render();
    if(view!=previousView) {
      if(view==V_FLIPPED)play(SOUND_FLIPPED);
      else if(view==V_LOW_BATTERY)play(SOUND_LOW_BATTERY);
      else if(view==V_CHARGING)play(SOUND_CHARGING);
    }
  } else {
    restore();render();
    if(before && recoverFlip)play(SOUND_UPRIGHT);
  }
}
void state_machine_motion_done(uint32_t id,bool success) {
  if(!id || id!=activeSit || mode!=SITTING || unsafe()) {rejected("Stale motion completion.");return;}
  activeSit=0;
  if(!success) {state_machine_event(ROBOT_EVENT_ERROR);return;}
  restore();render();
}
void state_machine_update(void) {
  if(!initialized)return;
  if(speaker_has_error() && !fault)state_machine_event(ROBOT_EVENT_ERROR);
  uint32_t elapsed=millis()-modeSince;
  if(!unsafe()) {
    if(mode==WAKING && elapsed>=3300) {restore();play(SOUND_WAKE);}
    else if(mode==LISTENING && elapsed>=5000) {invalidate();restore();}
    else if(mode==ACK || mode==CONFUSED) {
      bool drained=audioDrained();
      if(elapsed>=8000 && !drained) {state_machine_event(ROBOT_EVENT_ERROR);}
      else if(elapsed>=3000 && drained) {
        Action action=pending;uint32_t minutes=pendingMinutes;
        if(mode==ACK && action!=A_NONE && actionAllowed(action))execute(action,minutes);
        else restore();
      }
    } else if(mode==SPEAKING) {
      bool drained=audioDrained();
      if(elapsed>=8000 && !drained)state_machine_event(ROBOT_EVENT_ERROR);
      else if(elapsed>=300 && drained)restore();
    } else if(mode==SITTING) {
      if(!motionHandler && !legacySitHandler && elapsed>=2000)
        state_machine_motion_done(activeSit,true); // Bench motion simulation.
      else if(elapsed>=15000)state_machine_event(ROBOT_EVENT_ERROR);
    } else if(mode==SHY || mode==HAPPY) {
      bool drained=audioDrained();
      if(elapsed>=12000 && !drained)state_machine_event(ROBOT_EVENT_ERROR);
      else if(elapsed>=3000 && drained)restore();
    }
    if(study && !frozen && remainingNow()==0) {
      cancelStudy();enter(HAPPY);play(SOUND_TIMER_DONE);
    }
  }
  render();display_update();
}

void state_machine_command(const char *input) {
  char line[49];if(!normalize(input,line,sizeof(line)))return;
  if(!strcmp(line,"help")) {help();return;}
  if(!strcmp(line,"status")) {
    const char *views[]={"NONE","IDLE","WAKING","LISTENING","ACK","CONFUSED",
      "SITTING","SPEAKING","SHY","HAPPY","ANGRY","SLEEP","STUDY",
      "FLIPPED","LIFTED","EDGE","ERROR","CHARGING","LOW_BATTERY","NO_CONNECTION"};
    const char *modes[]={"IDLE","WAKING","LISTENING","ACK","CONFUSED","SITTING",
      "SPEAKING","SHY","HAPPY","ANGRY","SLEEP","STUDY"};
    Serial.print("View=");Serial.print(views[view]);Serial.print(" Mode=");Serial.println(modes[mode]);
    Serial.print("Flags flip/lift/edge/error/offline/low/docked/angry: ");
    Serial.print(flipped);Serial.print(" ");Serial.print(lifted);Serial.print(" ");
    Serial.print(edge);Serial.print(" ");Serial.print(fault);Serial.print(" ");
    Serial.print(offline);Serial.print(" ");Serial.print(lowBattery);Serial.print(" ");
    Serial.print(docked);Serial.print(" ");Serial.println(angry);
    Serial.print("Timer seconds=");Serial.print((remainingNow()+999)/1000);
    Serial.print(" paused=");Serial.print(frozen);Serial.print(" manual=");Serial.println(manualPause);
    Serial.print("Session=");Serial.print(state_machine_listening_session());
    Serial.print(" Sit request=");Serial.println(activeSit);return;
  }
  if(!strcmp(line,"mute")) {speaker_stop();return;}
  if(!strncmp(line,"volume ",7)) {
    uint32_t pct;if(number(line+7,100,pct))speaker_set_volume((uint8_t)pct);
    else rejected("Use volume 0..100.");
    return;
  }
  if(!strcmp(line,"bench off")) {bench=false;Serial.println("Bench shortcuts disabled.");return;}
  if(!strcmp(line,"bench on")) {
    if(bench)Serial.println("Bench shortcuts already enabled.");
    else rejected("Re-enable bench through setup API, not production input.");
    return;
  }
  struct EventName {const char *name;robot_event_t event;};
  const EventName events[]={
    {"flip",ROBOT_EVENT_FLIPPED},{"upright",ROBOT_EVENT_UPRIGHT},
    {"lifted",ROBOT_EVENT_LIFTED},{"lowered",ROBOT_EVENT_LOWERED},
    {"edge",ROBOT_EVENT_EDGE},{"edge_clear",ROBOT_EVENT_EDGE_CLEAR},
    {"error",ROBOT_EVENT_ERROR},{"error_clear",ROBOT_EVENT_ERROR_CLEAR},
    {"disconnected",ROBOT_EVENT_DISCONNECTED},{"connected",ROBOT_EVENT_CONNECTED},
    {"lowbattery",ROBOT_EVENT_LOW_BATTERY},{"battery_ok",ROBOT_EVENT_BATTERY_OK},
    {"charging",ROBOT_EVENT_CHARGING},{"undocked",ROBOT_EVENT_UNDOCKED},
    {"face",ROBOT_EVENT_FACE},{"face_left",ROBOT_EVENT_FACE_LEFT},
    {"face_right",ROBOT_EVENT_FACE_RIGHT},{"face_lost",ROBOT_EVENT_FACE_LOST},
    {"insult",ROBOT_EVENT_INSULT},{"angry",ROBOT_EVENT_INSULT},{"sorry",ROBOT_EVENT_APOLOGY}};
  for(const auto &entry:events) if(!strcmp(line,entry.name)) {
    if(!bench) {rejected("Use typed sensor events in production.");return;}
    state_machine_event(entry.event);return;
  }
  if(!strcmp(line,"listen")) {state_machine_begin_listening();return;}
  if(!strncmp(line,"voice ",6)) {state_machine_voice(state_machine_listening_session(),line+6);return;}
  if(!strcmp(line,"ack")||!strcmp(line,"confused")) {state_machine_voice(state_machine_listening_session(),line);return;}
  if(!strcmp(line,"stop")) {execute(A_STOP);return;}
  if(!bench) {rejected("Bench shortcut disabled. Use a session-bound voice result.");return;}
  if(!strcmp(line,"sit_done")) {state_machine_motion_done(activeSit,true);return;}
  if(unsafe()||angry) {rejected("Safety/anger lock active.");return;}
  Action action;uint32_t minutes;
  if(parseAction(line,action,minutes)) {
    if(action==A_SIT && mode==SITTING)return;
    if(actionAllowed(action)) {
      if(mode==ACK && action!=A_STOP) {pending=action;pendingMinutes=minutes;}
      else execute(action,minutes);
    } else rejected("Action not valid in the current study state.");return;
  }
  if(study) {rejected("Study Mode: use pause, resume, stop, or listen for a voice command.");return;}
  if(!strcmp(line,"wake")) {invalidate();enter(WAKING);render();return;}
  if(!strcmp(line,"normal")) {invalidate();restore();render();return;}
  if(!strcmp(line,"happy")||!strcmp(line,"shy")||!strcmp(line,"sleepy")) {
    enter(!strcmp(line,"happy")?HAPPY:!strcmp(line,"shy")?SHY:SLEEP);
    if(mode==HAPPY)play(SOUND_HAPPY);
    render();return;
  }
  struct SoundName {const char *name;sound_t sound;};
  const SoundName sounds[]={
    {"ack",SOUND_ACK},{"confused",SOUND_CONFUSED},{"bark",SOUND_BARK},
    {"bark_short",SOUND_BARK_SHORT},{"timer_done",SOUND_TIMER_DONE},
    {"low_battery",SOUND_LOW_BATTERY},{"wake",SOUND_WAKE},{"hello",SOUND_HELLO},
    {"listening",SOUND_LISTENING},{"study_start",SOUND_STUDY_START},
    {"pause",SOUND_PAUSE},{"resume",SOUND_RESUME},{"charging",SOUND_CHARGING},
    {"flipped",SOUND_FLIPPED},{"upright",SOUND_UPRIGHT},{"angry",SOUND_ANGRY},
    {"forgive",SOUND_FORGIVE},{"happy",SOUND_HAPPY},
    {"wow",SOUND_WOW},{"duang",SOUND_FLIPPED},{"ohyeah",SOUND_TIMER_DONE},
    {"laugh",SOUND_HAPPY},{"mambo",SOUND_WAKE},{"beep",SOUND_BEEP},{"tone",SOUND_TONE}};
  const char *name=!strncmp(line,"sound ",6)?line+6:line;
  for(const auto &s:sounds)if(!strcmp(name,s.name)) {play(s.sound);return;}
  rejected("Unknown command. Type help.");
}
