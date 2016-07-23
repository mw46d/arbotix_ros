/* 
  ArbotiX Firmware for ROS driver
  Copyright (c) 2008-2012 Vanadium Labs LLC.  All right reserved.
 
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/ 

/* Build Configuration */
#undef USE_BASE            // Enable support for a mobile base

#define CONTROLLER_COUNT    5
/* Hardware Constructs */
#include <cm900_ax12.h>
#include <cm900_BioloidController.h>
BioloidController controllers[CONTROLLER_COUNT];

#include "ros.h"

#include <Servo.h>
Servo servos[10];
int servo_vals[10];           // in millis

const unsigned char SERVO_PIN_MAP[10] = {
    10,                       // PIN 10 works as servo/PWM.
    2,                       // XXX: Not sure about the others!!
    3,
    6,
    7,
    8,
    9,
    10,
    13,
    14
};

#ifdef USE_BASE
  #include <Motors2.h>
  Motors2 drive = Motors2();
  #include <EncodersAB.h>
  #include "diff_controller.h"
#endif

Dynamixel Dxl(1);

/* Register Storage */
unsigned char baud = 7;         // ?
unsigned char ret_level = 1;    // ?
unsigned char alarm_led = 0;    // ?

/* Pose & Sequence Structures */
typedef struct{
  unsigned char pose;           // index of pose to transition to 
  int time;                     // time for transition
} sp_trans_t;
int poses[30][AX12_MAX_SERVOS]; // poses [index][servo_id-1]
sp_trans_t sequence[50];        // sequence
int seqPos;                     // step in current sequence

#include "user_hooks.h"

void setup(){
#ifdef USE_BASE
    drive.init();
    Encoders.Begin();
    setupPID();
#endif

    Dxl.begin(3);
    Dxl.writeWord(BROADCAST_ID, 32, 60);
    Dxl.writeWord(BROADCAST_ID, 30, 512);
    Dxl.writeWord(1, 30, 696);
    Dxl.writeWord(2, 30, 820); // 776);
    Dxl.writeWord(10, 30, 192); // 248);
    Dxl.writeWord(3, 30, 560);
    Dxl.writeWord(4, 30, 312);
    delay(4000);
    Dxl.writeWord(BROADCAST_ID, 32, 1023);

    userSetup();
    pinMode(BOARD_LED_PIN, OUTPUT);     // status LED
    Serial2.begin(115200);
}

/*
 * Handle Write Requests to ArbotiX Registers
 */
unsigned char handleWrite(){
    int addr = params[0];                   // address to write
    int bytes = length - 3;                 // # of bytes left to write
    int k = 1;                              // index in parameters of value to write

    while (bytes > 0) {
        if (addr < REG_BAUD_RATE) {
            return ERR_INSTRUCTION;
        }
        else if (addr == REG_BAUD_RATE) {
                                            // Not on the CM9*
        }
        else if (addr < REG_RESCAN) {
            return ERR_INSTRUCTION;         // can't write digital inputs
        }
        else if (addr == REG_RESCAN) {
                                            // Not on the CM9*
        }
        else if (addr == REG_RETURN_LEVEL) {
            ret_level = params[k];
        }
        else if (addr == REG_ALARM_LED) {
                                            // TODO:
        }
        else if (addr < REG_SERVO_BASE) {
            return ERR_INSTRUCTION;         // error - analog are read only
        }
        else if (addr < REG_MOVING) {       // write servo
            int s = addr - REG_SERVO_BASE;
            if (s >= 20) {
                return ERR_INSTRUCTION;
            }
            else {
                if (s % 2 == 0) {           // low byte
                    s = s / 2;
                    servo_vals[s] = params[k];
                }
                else {                      // high byte
                    s = s / 2;
                    servo_vals[s] += (params[k] << 8);

                    if (servo_vals[s] > 500 && servo_vals[s] < 2500) {
                        if (!servos[s].attached()) {
                            servos[s].attach(SERVO_PIN_MAP[s]);
                        }
                        servos[s].writeMicroseconds(servo_vals[s]);
                    }
                    else if (servo_vals[s] == 0) {
                        servos[s].detach();
                    }
                }
            }
        }
        else if (addr == REG_MOVING) {
            return ERR_INSTRUCTION;
        }
        else if (addr < REG_RESERVED) {     // write digital pin
            int pin = addr - REG_DIGITAL_OUT0;
            // MW:XXX Map the pins!!

            if (params[k] & 0x02) {         // high
                digitalWrite(pin, HIGH);
            }
            else {
                digitalWrite(pin, LOW);
            }

            if (params[k] & 0x01) {         // output
                pinMode(pin, OUTPUT);
            }
            else {
                pinMode(pin, INPUT);
            }
        }
        else {
            int ret = userWrite(addr, params[k]);

            if (ret > ERR_NONE) {
                return ret;
            }
        }

        addr++;
        k++;
        bytes--;
    }

    return ERR_NONE;
}


/*
 * Handle a read from ArbotiX registers.
 */
int handleRead() {
    int checksum = 0;
    int addr = params[0];
    int bytes = params[1];
    unsigned char v;

    while (bytes > 0) {
        if (addr == REG_MODEL_NUMBER_L) {
            v = 132;
        }
        else if (addr == REG_MODEL_NUMBER_H) {
            v = 3;                          // 900
        }
        else if (addr == REG_VERSION) {
            v = 0;
        }
        else if (addr == REG_ID) {
            v = 253;
        }
        else if (addr == REG_BAUD_RATE) {
            v = 3;                          // 500000
        }
        else if (addr == REG_DIGITAL_IN0) {
                                            // digital 0-7
            pinMode(0, INPUT);
            pinMode(1, INPUT);
            pinMode(2, INPUT);
            pinMode(3, INPUT);
            pinMode(4, INPUT);
            pinMode(5, INPUT);
            pinMode(6, INPUT);
            pinMode(7, INPUT);
            v |= digitalRead(0) & 0x01;
            v |= (digitalRead(1) & 0x01) << 1;
            v |= (digitalRead(2) & 0x01) << 2;
            v |= (digitalRead(3) & 0x01) << 3;
            v |= (digitalRead(4) & 0x01) << 4;
            v |= (digitalRead(5) & 0x01) << 5;
            v |= (digitalRead(6) & 0x01) << 6;
            v |= (digitalRead(7) & 0x01) << 7;
        }
        else if (addr == REG_DIGITAL_IN1) {
                                            // digital 8-15
            pinMode(8, INPUT);
            pinMode(9, INPUT);
            pinMode(10, INPUT);
            pinMode(11, INPUT);
            pinMode(12, INPUT);
            pinMode(13, INPUT);
            pinMode(14, INPUT);
            pinMode(15, INPUT);
            v |= digitalRead(8) & 0x01;
            v |= (digitalRead(9) & 0x01) << 1;
            v |= (digitalRead(10) & 0x01) << 2;
            v |= (digitalRead(11) & 0x01) << 3;
            v |= (digitalRead(12) & 0x01) << 4;
            v |= (digitalRead(13) & 0x01) << 5;
            v |= (digitalRead(14) & 0x01) << 6;
            v |= (digitalRead(15) & 0x01) << 7;
        }
        else if (addr == REG_DIGITAL_IN2) {
                                            // digital 16-23
            pinMode(16, INPUT);
            pinMode(17, INPUT);
            pinMode(18, INPUT);
            pinMode(19, INPUT);
            pinMode(20, INPUT);
            pinMode(21, INPUT);
            pinMode(22, INPUT);
            pinMode(23, INPUT);
            v |= digitalRead(16) & 0x01;
            v |= (digitalRead(17) & 0x01) << 1;
            v |= (digitalRead(18) & 0x01) << 2;
            v |= (digitalRead(19) & 0x01) << 3;
            v |= (digitalRead(20) & 0x01) << 4;
            v |= (digitalRead(21) & 0x01) << 5;
            v |= (digitalRead(22) & 0x01) << 6;
            v |= (digitalRead(23) & 0x01) << 7;
        }
        else if (addr == REG_DIGITAL_IN3) {
                                            // digital 24-31
            pinMode(24, INPUT);
            pinMode(25, INPUT);
            pinMode(26, INPUT);
            pinMode(27, INPUT);
            pinMode(28, INPUT);
            pinMode(29, INPUT);
            pinMode(30, INPUT);
            pinMode(31, INPUT);
            v |= digitalRead(24) & 0x01;
            v |= (digitalRead(25) & 0x01) << 1;
            v |= (digitalRead(26) & 0x01) << 2;
            v |= (digitalRead(27) & 0x01) << 3;
            v |= (digitalRead(28) & 0x01) << 4;
            v |= (digitalRead(29) & 0x01) << 5;
            v |= (digitalRead(30) & 0x01) << 6;
            v |= (digitalRead(31) & 0x01) << 7;
        }
        else if (addr == REG_RETURN_LEVEL) {
            v = ret_level;
        }
        else if (addr == REG_ALARM_LED) {
                                            // TODO
        }
        else if (addr < REG_SERVO_BASE) {
                                            // send analog reading Pins 0-7
            pinMode(addr - REG_ANA_BASE, INPUT_ANALOG);
            int x = analogRead(addr - REG_ANA_BASE);
            x += analogRead(addr - REG_ANA_BASE);
            x += analogRead(addr - REG_ANA_BASE);
            x += analogRead(addr - REG_ANA_BASE);
            v = (x >> 4) / 4;
        }
        else if (addr < REG_MOVING) {
                                            // send servo position
            int s = addr - REG_SERVO_BASE;
            if (s >= 20) {
                return ERR_INSTRUCTION;
            }
            else if (!servos[s / 2].attached()) {
                v = 0;
            }
            else {
                if (s % 2 == 0) {           // low byte
                    s = s / 2;
                    v = servo_vals[s] & 0xff;
                }
                else {                      // high byte
                    s = s / 2;
                    v = (servo_vals[s] >> 8) & 0xff;

                }
            }
        }
        else {
            v = userRead(addr);
        }

        checksum += v;
        Serial2.write(v);
        addr++;
        bytes--;
    }

  return checksum;
}

int doPlaySeq() {
    seqPos = 0;

    while (sequence[seqPos].pose != 0xff) {
        int p = sequence[seqPos].pose;

        if (Serial2.read() == 'H') {      // Halt execution
            return 1;
        }

        // load pose
        for (int i = 0; i < controllers[0].poseSize; i++) {
            controllers[0].setNextPose(i + 1,poses[p][i]);
        }

        controllers[0].interpolateSetup(sequence[seqPos].time);

        while (controllers[0].interpolating) {
            controllers[0].interpolateStep();
        }

        // next transition
        seqPos++;
    }

    return 0;
}

/*
 * Send status packet
 */
void statusPacket(int id, int err) {
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(id);
    Serial2.write(2);
    Serial2.write(err);
    Serial2.write(255 - ((id + 2 + err) % 256));
}

/*
 * decode packets: ff ff id length ins params checksum
 *   same as ax-12 table, except, we define new instructions for Arbotix
 */
void loop() {
    int i;

    // process messages
    while (Serial2.available() > 0) {
        // We need to 0xFF at start of packet
        if (mode == 0) {                     // start of new packet
            if (Serial2.read() == 0xff) {
                mode = 2;
                digitalWrite(0,HIGH-digitalRead(0));
            }
        }
        else if (mode == 2) {               // next byte is index of servo
            id = Serial2.read();
            if (id != 0xff) {
                mode = 3;
            }
        }
        else if (mode == 3) {               // next byte is length
            length = Serial2.read();
            checksum = id + length;
            mode = 4;
        }
        else if (mode == 4) {               // next byte is instruction
            ins = Serial2.read();
            checksum += ins;
            index = 0;
            mode = 5;
        }
        else if (mode == 5) {               // read data in
            params[index] = Serial2.read();
            checksum += (int)params[index];
            index++;

            if (index + 1 == length) {      // we've read params & checksum
                mode = 0;
                if ((checksum % 256) != 255) {
                    // return an error packet: FF FF id Len Err=bad checksum, params=None check
                    statusPacket(id, ERR_CHECKSUM);
                }
                else if (id == 253) {       // ID = 253, ArbotiX instruction
                    switch (ins) {
                    case AX_WRITE_DATA:
                                            // send return packet
                        statusPacket(id, handleWrite());
                        break;
                    case AX_READ_DATA:
                        checksum = id + params[1] + 2;
                        Serial2.write(0xff);
                        Serial2.write(0xff);
                        Serial2.write(id);
                        Serial2.write((unsigned char)2 + params[1]);
                        Serial2.write((unsigned char)0);
                                            // send actual data
                        checksum += handleRead();
                        Serial2.write(255 - ((checksum) % 256));
                        break;
                    case ARB_SIZE_POSE:     // Pose Size = 7, followed by single param: size of pose
                        statusPacket(id, 0);
                        if (controllers[0].poseSize == 0) {
                            controllers[0].setup(18);
                        }
                        controllers[0].poseSize = params[0];
                        controllers[0].readPose();
                        break;
                    case ARB_LOAD_POSE:     // Load Pose = 8, followed by index, then pose positions (# of param = 2*pose_size)
                        statusPacket(id, 0);
                        for (i = 0; i < controllers[0].poseSize; i++) {
                            poses[params[0]][i] = params[(2 * i) + 1] + (params[(2 * i) + 2] << 8);
                        }
                        break;
                    case ARB_LOAD_SEQ:      // Load Seq = 9, followed by index/times (# of parameters = 3*seq_size)
                        statusPacket(id, 0);
                        for (i = 0; i < (length - 2) / 3; i++) {
                            sequence[i].pose = params[(i * 3)];
                            sequence[i].time = params[(i * 3) + 1] + (params[(i * 3) + 2] << 8);
                        }
                        break;
                    case ARB_PLAY_SEQ:      // Play Seq = A, no params
                        statusPacket(id, 0);
                        doPlaySeq();
                        break;
                    case ARB_LOOP_SEQ:      // Play Seq until we recieve a 'H'alt
                        statusPacket(id, 0);
                        while(doPlaySeq() > 0) {
                        }
                        break;
                    case ARB_CONTROL_SETUP: // Setup a controller
                        statusPacket(id, 0);
                        if (params[0] < CONTROLLER_COUNT) {
                            controllers[params[0]].setup(length - 3);
                            for (int i = 0; i < length - 3; i++) {
                                controllers[params[0]].setId(i, params[i+1]);
                            }
#ifdef USE_BASE
                        }
                        else if (params[0] == 10) {
                            Kp = params[1];
                            Kd = params[2];
                            Ki = params[3];
                            Ko = params[4];
#endif
                        }
                        break;
                    case ARB_CONTROL_WRITE: // Write values to a controller
                        statusPacket(id, 0);
                        if (params[0] < CONTROLLER_COUNT) {
                            for (int i = 0; i < length - 4; i += 2) {
                                controllers[params[0]].setNextPose(controllers[params[0]].getId(i / 2),
                                                                   params[i + 1] + (params[i + 2] << 8));
                            }
                            controllers[params[0]].readPose();
                            controllers[params[0]].interpolateSetup(params[length - 3] * 33);
#ifdef USE_BASE
                        }
                        else if (params[0] == 10) {
                            left_speed = params[1];
                            left_speed += (params[2] << 8);
                            right_speed = params[3];
                            right_speed += (params[4] < <8);

                            if ((left_speed == 0) && (right_speed == 0)) {
                                drive.set(0, 0);
                                ClearPID();
                            }
                            else {
                                if ((left.Velocity == 0) && (right.Velocity == 0)) {
                                    PIDmode = 1;
                                    moving = 1;
                                    left.PrevEnc = Encoders.left;
                                    right.PrevEnc = Encoders.right;
                                }
                            }
                            left.Velocity = left_speed;
                            right.Velocity = right_speed;
#endif
                        }
                        break;
                    case ARB_CONTROL_STAT:  // Read status of a controller
                        if (params[0] < CONTROLLER_COUNT) {
                            Serial2.write((unsigned char)0xff);
                            Serial2.write((unsigned char)0xff);
                            Serial2.write((unsigned char)id);
                            Serial2.write((unsigned char)3);
                            Serial2.write((unsigned char)0);
                            checksum = controllers[params[0]].interpolating;
                            Serial2.write((unsigned char)checksum);
                            checksum += id + 3;
                            Serial2.write((unsigned char)255 - ((checksum) % 256));
#ifdef USE_BASE
                        }
                        else if (params[0] == 10) {
                            checksum = id + 2 + 8;
                            Serial2.write((unsigned char)0xff);
                            Serial2.write((unsigned char)0xff);
                            Serial2.write((unsigned char)id);
                            Serial2.write((unsigned char)2+8);
                            Serial2.write((unsigned char)0);   // error level
                            int v = ((unsigned long)Encoders.left >> 0) % 256;
                            Serial2.write((unsigned char)v);
                            checksum += v;
                            v = ((unsigned long)Encoders.left >> 8) % 256;
                            Serial2.write((unsigned char)v);
                            checksum += v;
                            v = ((unsigned long)Encoders.left >> 16) % 256;
                            Serial2.write((unsigned char)v);
                            checksum += v;
                            v = ((unsigned long)Encoders.left >> 24) % 256;
                            Serial2.write((unsigned char)v);
                            checksum += v;
                            v = ((unsigned long)Encoders.right >> 0) % 256;
                            Serial2.write((unsigned char)v);
                            checksum += v;
                            v = ((unsigned long)Encoders.right >> 8) % 256;
                            Serial2.write((unsigned char)v);
                            checksum += v;
                            v = ((unsigned long)Encoders.right >> 16) % 256;
                            Serial2.write((unsigned char)v);
                            checksum += v;
                            v = ((unsigned long)Encoders.right >> 24) % 256;
                            Serial2.write((unsigned char)v);
                            checksum += v;
                            Serial2.write((unsigned char)255 - ((checksum) % 256));
#endif
                        }
                        break;
                    }
                }
                else if (id == 0xFE) {      // sync read or write
                    if (ins == ARB_SYNC_READ) {
                        int strt = params[0];    // address to read in control table
                        int bytes = params[1];    // # of bytes to read from each servo
                        checksum = id + (bytes * (length - 4)) + 2;
                        Serial2.write((unsigned char)0xff);
                        Serial2.write((unsigned char)0xff);
                        Serial2.write((unsigned char)id);
                        Serial2.write((unsigned char)2 + (bytes * (length - 4)));
                        Serial2.write((unsigned char)0);     // error code

                        for (int k = 2; k < length - 2; k++) { // send actual data
                            if (bytes == 1) {
                                unsigned char c = Dxl.readByte(params[k], strt);

                                checksum += c;
                                Serial2.write(c);
                            }
                            else if (bytes == 2) {
                                word w = Dxl.readWord(params[k], strt);
                                unsigned char b1, b2;
                                
                                b1 = w & 0xff;
                                b2 = (w >> 8) & 0xff;

                                checksum += b1;
                                Serial2.write(b1);
                                checksum += b2;
                                Serial2.write(b2);
                            }
                            else {
                                for (i = 0; i < bytes; i++) {
                                    checksum += 255;
                                    Serial2.write((unsigned char)255);
                                }
                            }
                        }

                        Serial2.write((unsigned char)255 - ((checksum) % 256));
                    }
                    else {
                        int k;

                        Dxl.setTxPacketId(id);
                        Dxl.setTxPacketLength(length);
                        Dxl.setTxPacketInstruction(ins);
                        for (k = 0; k < length - 2; k++) {
                            Dxl.setTxPacketParameter(k, (unsigned char)params[k]);
                        }
                        Dxl.txrxPacket();
                        // no return
                    }
                }
                else if (id == 2) {             // ID 2 - we handle two servos as one here, ID 2 and 10 (inverted)
                    unsigned char ret;

                    switch (ins) {              // Reads are only from the ID2
                    case AX_READ_DATA:
                        Dxl.setTxPacketId(id);
                        Dxl.setTxPacketLength(4);
                        Dxl.setTxPacketInstruction(ins);
                        Dxl.setTxPacketParameter(0, params[0]);
                        Dxl.setTxPacketParameter(1, params[1]);
                        Dxl.txrxPacket();

                        ret = Dxl.getResult();
                        length = 2;

                        if (ret == (1 << COMM_RXSUCCESS)) {
                            length = Dxl.getRxPacketLength();
                        }

                        // return a packet: FF FF id Len Err params check
                        Serial2.write(0xff);
                        Serial2.write(0xff);
                        Serial2.write(id);
                        Serial2.write((unsigned char)length);
                        Serial2.write(ret);
                        checksum = id + ret;
                        checksum += length;

                        for (i = 0; i < length - 2; i++) {
                            unsigned char b = Dxl.getRxPacketParameter(i);
                            checksum += b;
                            Serial2.write(b);
                        }

                        Serial2.write((unsigned char)255 - ((checksum) % 256));
                        break;
                    case AX_WRITE_DATA:         // Writes need to be done to both ID's with the goal reversed for the ID 10!
                        if (length == 4) {
                            ret = 0;

                            if (params[0] != AX_ID) {
                                Dxl.writeByte(id, params[0], params[1]);
                                ret = Dxl.getResult();
                                Dxl.writeByte(10, params[0], params[1]);
                                ret = Dxl.getResult();
                            }
                        }
                        else {
                            word x2 = params[1] + (params[2] << 8);
                            word x10 = x2;

                            if (params[0] == AX_GOAL_POSITION_L) {
                                if (x2 == 1023) {
                                    x10 = 0;
                                }
                                else if (x2 == 0) {
                                    x10 = 1023;
                                }
                                else {
                                    x10 = 512 - (x2 - 512);
                                }
                            }

                            Dxl.writeWord(id, params[0], x2);
                            ret = Dxl.getResult();
                            Dxl.writeWord(10, params[0], x10);
                            ret = Dxl.getResult();
                        }

                        statusPacket(id, ret);
                        break;
                    }
                }
                else {                          // ID != 253, pass thru
                    unsigned char ret;

                    switch (ins) {              // TODO: streamline this
                    case AX_READ_DATA:
                        Dxl.setTxPacketId(id);
                        Dxl.setTxPacketLength(4);
                        Dxl.setTxPacketInstruction(ins);
                        Dxl.setTxPacketParameter(0, params[0]);
                        Dxl.setTxPacketParameter(1, params[1]);
                        Dxl.txrxPacket();

                        ret = Dxl.getResult();
                        length = 2;

                        if (ret == (1 << COMM_RXSUCCESS)) {
                            length = Dxl.getRxPacketLength();
                        }

                        // return a packet: FF FF id Len Err params check
                        Serial2.write(0xff);
                        Serial2.write(0xff);
                        Serial2.write(id);
                        Serial2.write((unsigned char)length);
                        Serial2.write(ret);
                        checksum = id + ret;
                        checksum += length;

                        for (i = 0; i < length - 2; i++) {
                            unsigned char b = Dxl.getRxPacketParameter(i);
                            checksum += b;
                            Serial2.write(b);
                        }

                        Serial2.write((unsigned char)255 - ((checksum) % 256));
                        break;
                        ret = Dxl.getResult();
                        checksum = id + ret;
                        length = 0;

                        if (ret == (1 << COMM_RXSUCCESS)) {
                            length = Dxl.getRxPacketLength();
                            checksum += length;
                        }

                        Serial2.write((unsigned char)length);
                        Serial2.write(ret);

                        for (i = 0; i < length - 2; i++) {
                            unsigned char b = Dxl.getRxPacketParameter(i);
                            checksum += b;
                            Serial2.write(b);
                        }

                        Serial2.write((unsigned char)255 - ((checksum) % 256));
                        break;

                    case AX_WRITE_DATA:
                        if (length == 4) {
                            Dxl.writeByte(id, params[0], params[1]);
                        }
                        else {
                            word x = params[1] + (params[2] << 8);
                            Dxl.writeWord(id, params[0], x);
                        }

                        statusPacket(id, Dxl.getResult());
                        break;
                    }
                }
            }
        } // end mode == 5
    } // end while(available)

    // update joints
    for (int i = 0; i < CONTROLLER_COUNT; i++) {
        controllers[i].interpolateStep();
    }

#ifdef USE_BASE
    // update pid
    updatePID();
#endif
}
