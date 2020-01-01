/*
 * You can render following FSM diagram with https://www.planttext.com/

@startuml
title Roll Blind

state Idle
state RollUp
state RollDown
state CloseUp
state CloseDown

[*] --> Idle

Idle --> RollUp : IRKeyUp
Idle -> RollDown : IRKeyDown

RollUp -> Idle : IRKeyOK
RollUp -> Idle : TimeoutUp
RollUp --> CloseUp : SensorUp

CloseUp -> Idle : SlowTimeout
CloseDown -> Idle : SlowTimeout

note "All transitions to action state must add\na timed transition back to Idle state" as N1

RollDown --> Idle : IRKeyOK
RollDown --> Idle : TimeoutDown
RollDown --> CloseDown : SensorDown


@enduml

 */

#include <Fsm.h>
#include <IRremote.h>
#include <L298N.h>

#define IR_PIN        13

#define SENSOR_TOP    9
#define SENSOR_BOTTOM 8
#define SENSOR_ON     true
#define SENSOR_OFF    false

#define MOTOR_EN      10
#define MOTOR_IN1     12
#define MOTOR_IN2     11

#define IR_KEY_UP         0xFF629D
#define IR_KEY_DOWN       0xFFA857
#define IR_KEY_ASTERIX    0xFF42BD
#define IR_KEY_HASH       0xFF52AD
#define IR_KEY_1          0xFF6897
#define IR_KEY_3          0xFFB04F

#define TIMEOUT_UP    32000
#define TIMEOUT_DOWN  29000

#define TIMEOUT_CLOSEDOWN  1000
#define TIMEOUT_CLOSEUP    1000

// FSM events
#define EV_ROLL_UP          1
#define EV_ROLL_DOWN        2
#define EV_UNKNOWN_COMMAND	3
#define EV_AT_TOP           4
#define EV_AT_BOTTOM        5


class ObstacleSensor {
public:
  ObstacleSensor(uint8_t _pin) {
    pin_ = _pin;
    force_ = false;
  }

  // call bind() from setup()
  void bind(Fsm* _fsm, int _ev_clear, int _ev_cover) {
    pinMode(pin_, INPUT);
    fsm_ = _fsm;
    ev_raise_ = _ev_clear;
    ev_fall_ = _ev_cover;
  }

  void enable() {
    force_ = false;
    update();
  }

  void disable(bool state) {
    force_ = true;
    state_ = state;
  }

  bool blind() const {
    return state_;
  }

  // call update() in loop()
  void update() {
    if (force_) return;
    bool newstate = (digitalRead(pin_) ==  LOW);
    if (newstate ^ state_) {
      state_ = newstate;
      int ev = newstate ? ev_raise_ : ev_fall_;
      if (ev) fsm_->trigger(ev);
    }
  }

protected:
  uint8_t pin_;
  bool state_;
  bool force_;
  Fsm* fsm_;
  int ev_raise_;
  int ev_fall_;
};

L298N motor(MOTOR_EN, MOTOR_IN1, MOTOR_IN2);
ObstacleSensor at_top(SENSOR_TOP), at_bottom(SENSOR_BOTTOM);
IRrecv irrecv(IR_PIN);
decode_results results;

void on_idle_enter() {
    Serial.print("OK ENTER IDLE "); Serial.println(millis());
    motor.stop();
}

void on_rollup_enter() {
    if (!at_top.blind()) {
      Serial.print("ERR ALREADY AT TOP "); Serial.println(millis());
      return;
    }
    Serial.print("OK START ROLL-UP "); Serial.println(millis());
    motor.setSpeed(255);  // set max available speed
    motor.forward();
}

void on_rolldown_enter() {
    if (at_bottom.blind()) {
      Serial.print("ERR ALREADY AT BOTTOM "); Serial.println(millis());
      return;
    }
    Serial.print("OK START ROLL-DOWN "); Serial.println(millis());
    motor.setSpeed(255);  // set max available speed
    motor.backward();
}


void on_slowup_enter() {
    Serial.print("OK START ROLL-CLOSE-UP "); Serial.println(millis());
}

void on_slowdown_enter() {
    Serial.print("OK START ROLL-CLOSE-DOWN "); Serial.println(millis());
}


State state_idle(&on_idle_enter, NULL, NULL);
State state_rollup(&on_rollup_enter, NULL, NULL);
State state_rolldown(&on_rolldown_enter, NULL, NULL);

State state_slowup(&on_slowup_enter, NULL, NULL);
State state_slowdown(&on_slowdown_enter, NULL, NULL);


Fsm fsm_roblin(&state_idle);

void cmd_rollup() {
    if (!at_top.blind()) {
      Serial.println("OK ALREADY AT TOP");
      return;
    }
    fsm_roblin.trigger(EV_ROLL_UP);
}

void cmd_rolldown() {
    if (at_bottom.blind()) {
      Serial.println("OK ALREADY AT BOTTOM");
      return;
    }
    fsm_roblin.trigger(EV_ROLL_DOWN);
}

void cmd_stop() {
    if (!motor.isMoving()) {
      Serial.println("ERR NOT ROLLING");
      return;
    }
    fsm_roblin.trigger(EV_UNKNOWN_COMMAND);
}


void setup() {
  Serial.begin(57600);
  delay(1000);
  Serial.println("OK ROLLER-BLIND 1.0\n");

  motor.stop(); // stop motor

  // bind sensors to superposition
  at_top.bind(&fsm_roblin,    0,            EV_AT_TOP);
  at_bottom.bind(&fsm_roblin, EV_AT_BOTTOM, 0);

  irrecv.enableIRIn();

  fsm_roblin.add_transition(&state_idle, &state_rollup, EV_ROLL_UP, NULL);
  fsm_roblin.add_transition(&state_idle, &state_rolldown, EV_ROLL_DOWN, NULL);

  fsm_roblin.add_timed_transition(&state_rolldown, &state_idle, TIMEOUT_DOWN, NULL);
  fsm_roblin.add_timed_transition(&state_rollup, &state_idle, TIMEOUT_UP, NULL);

  fsm_roblin.add_transition(&state_rollup, &state_idle, EV_UNKNOWN_COMMAND, NULL);
  fsm_roblin.add_transition(&state_rolldown, &state_idle, EV_UNKNOWN_COMMAND, NULL);

  fsm_roblin.add_transition(&state_rollup, &state_slowup, EV_AT_TOP, NULL);
  fsm_roblin.add_transition(&state_rolldown, &state_slowdown, EV_AT_BOTTOM, NULL);

  fsm_roblin.add_timed_transition(&state_slowup, &state_idle, TIMEOUT_CLOSEDOWN, NULL);
  fsm_roblin.add_timed_transition(&state_slowdown, &state_idle, TIMEOUT_CLOSEUP, NULL);

}

void loop() {

  if ( irrecv.decode( &results )) {

    irrecv.resume();
    if (results.value == 0xFFFFFFFF) return;

    Serial.print("MSG IR-BUTTON-PRESS ");
    Serial.println( results.value, HEX );

    if (results.value == IR_KEY_UP) {
      cmd_rollup();
    } else if (results.value == IR_KEY_DOWN) {
      cmd_rolldown();
    } else if (results.value == IR_KEY_ASTERIX) {
      Serial.println( "MSG FORCE SENSORS OFF" );
      at_top.disable(SENSOR_ON);
      at_bottom.disable(SENSOR_OFF);
    } else if (results.value == IR_KEY_HASH) {
      Serial.println( "MSG FORCE SENSORS ON" );
      at_top.enable();
      at_bottom.enable();
    } else if (results.value == IR_KEY_1) {
      Serial.println( "MSG FAKE SENSOR TOP" );
      fsm_roblin.trigger(EV_AT_TOP);
    } else if (results.value == IR_KEY_3) {
      Serial.println( "MSG FAKE SENSOR BOTTOM" );
      fsm_roblin.trigger(EV_AT_BOTTOM);
    } else {
      cmd_stop();
    }
  }
  
  while (Serial.available() > 0) {
    String request = Serial.readStringUntil('\n');
    request.trim();
    if (request == "UP") {
      cmd_rollup();
    } else if (request == "DN") {
      cmd_rolldown();
    } else {
      cmd_stop();
    }
  }

  // check sensors
  at_top.update();
  at_bottom.update();

  fsm_roblin.run_machine();
  delay(50);
  
}

