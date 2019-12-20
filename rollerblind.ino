/*
 * You can render following FSM diagram with https://www.planttext.com/

@startuml
title Roll Blind

state Idle
state RollUp
state RollDown
state SlowUp
state SlowDown

[*] --> Idle

Idle --> RollUp : IRKeyUp
Idle -> RollDown : IRKeyDown

RollUp -> Idle : IRKeyOK
RollUp -> Idle : TimeoutUp
RollUp --> SlowUp : SensorUp

SlowUp -> Idle : SlowTimeout
SlowDown -> Idle : SlowTimeout

note "All transitions to action state must add\na timed transition back to Idle state" as N1

RollDown --> Idle : IRKeyOK
RollDown --> Idle : TimeoutDown
RollDown --> SlowDown : SensorDown


@enduml

 */

#include <Fsm.h>
#include <IRremote.h>
#include <L298N.h>

#define SENSOR_TOP    11
#define SENSOR_BOTTOM 10

#define MOTOR_EN      3
#define MOTOR_IN1     2
#define MOTOR_IN2     4

#define IR_KEY_UP     0xFF629D
#define IR_KEY_DOWN   0xFFA857

// FSM events
#define EV_ROLL_UP          1
#define EV_ROLL_DOWN        2
#define EV_UNKNOWN_COMMAND	3
#define EV_AT_TOP               4
#define EV_AT_BOTTOM            5


class ObstacleSensor {
public:
  ObstacleSensor(uint8_t _pin) {
    pin_ = _pin;
  }

  // call bind() from setup()
  void bind(Fsm* _fsm, int _ev_clear, int _ev_cover) {
    pinMode(pin_, INPUT);
    fsm_ = _fsm;
    ev_raise_ = _ev_clear;
    ev_fall_ = _ev_cover;
  }

  bool blind() const {
    return state_;
  }

  // call update() in loop()
  void update() {
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
  Fsm* fsm_;
  int ev_raise_;
  int ev_fall_;
};

L298N motor(MOTOR_EN, MOTOR_IN1, MOTOR_IN2);
ObstacleSensor at_top(SENSOR_TOP), at_bottom(SENSOR_BOTTOM);
IRrecv irrecv(12);
decode_results results;

void on_idle_enter() {
    Serial.println("OK ENTER IDLE");
    motor.stop();
}

void on_rollup_enter() {
    Serial.println("OK START ROLL-UP");
    motor.forward();
}

void on_rolldown_enter() {
    if (at_bottom.blind()) {
      Serial.println("ERR ALREADY AT BOTTOM");
      return;
    }
    Serial.println("OK START ROLL-DOWN");
    motor.backward();
}

State state_idle(&on_idle_enter, NULL, NULL);
State state_rollup(&on_rollup_enter, NULL, NULL);
State state_rolldown(&on_rolldown_enter, NULL, NULL);

Fsm fsm_roblin(&state_idle);

void cmd_rollup() {
    if (!at_top.blind()) {
      Serial.println("ERR ALREADY AT TOP");
      return;
    }
    fsm_roblin.trigger(EV_ROLL_UP);
}

void cmd_rolldown() {
    if (at_bottom.blind()) {
      Serial.println("ERR ALREADY AT BOTTOM");
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
  Serial.println("OK ROLLER-BLIND 1.0\n");

  motor.stop();         // stop motor
  motor.setSpeed(255);  // set max available speed

  // bind sensors to superposition
  at_top.bind(&fsm_roblin,    0,            EV_AT_TOP);
  at_bottom.bind(&fsm_roblin, EV_AT_BOTTOM, 0);

  irrecv.enableIRIn();

  fsm_roblin.add_transition(&state_idle, &state_rollup, EV_ROLL_UP, NULL);
  fsm_roblin.add_transition(&state_idle, &state_rolldown, EV_ROLL_DOWN, NULL);

  fsm_roblin.add_timed_transition(&state_rolldown, &state_idle, 10001, NULL);
  fsm_roblin.add_timed_transition(&state_rollup, &state_idle, 10002, NULL);

  fsm_roblin.add_transition(&state_rollup, &state_idle, EV_UNKNOWN_COMMAND, NULL);
  fsm_roblin.add_transition(&state_rolldown, &state_idle, EV_UNKNOWN_COMMAND, NULL);

  fsm_roblin.add_transition(&state_rollup, &state_idle, EV_AT_TOP, NULL);
  fsm_roblin.add_transition(&state_rolldown, &state_idle, EV_AT_BOTTOM, NULL);

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
  delay(100);
  
}

