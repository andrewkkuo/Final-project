#include "mbed.h"
#include "bbcar.h"
#include "erpc_simple_server.h"
#include "erpc_basic_codec.h"
#include "erpc_crc16.h"
#include "UARTTransport.h"
#include "DynamicMessageBufferFactory.h"
#include "bbcar_info_server.h"
#define CENTER_BASE 1500

Ticker timer;
int temp1 = 3;
int temp3 = 3;
int temp = 10;
Ticker encoder_ticker;
Ticker servo_ticker;
volatile int steps;
volatile int last;
int pattern = 0;
BufferedSerial pc(USBTX, USBRX);
DigitalIn encoder(D11);
PwmOut pin5(D5), pin6(D6);
BBCar car(pin5, pin6, servo_ticker);
DigitalInOut FR(D2), R(D3), L(D4), FL(D7);
DigitalInOut pin10(D10);
int first = 0;
Thread s(osPriorityBelowNormal);
int nextt = 0;
int nexttt = 0;
float llast = 0;
float distancee = 0;
float num  = 0;
Thread u;

ep::UARTTransport uart_transport(D1, D0, 9600);
ep::DynamicMessageBufferFactory dynamic_mbf;
erpc::BasicCodecFactory basic_cf;
erpc::Crc16 crc16;
erpc::SimpleServer rpc_server;
BBCarService_service bbcar_service;

void info() {
    while(1){
        printf("distance: %f\n", steps *6.5 * 3.14 / 32);
        printf("current speed: %f\n", num);
        ThisThread::sleep_for(1s);
    }
}

void calculate() {
    distancee = steps *6.5 * 3.14 / 32;
    num = distancee - llast;
    llast = distancee;
}

void callfunc() {
    timer.attach(&calculate, 1s);
}

void encoder_control() {
   int value = encoder;
   if (!last && value) steps++;
   last = value;
}

void rpc() {
    uart_transport.setCrc16(&crc16);
    rpc_server.setTransport(&uart_transport);
    rpc_server.setCodecFactory(&basic_cf);
    rpc_server.setMessageBufferFactory(&dynamic_mbf);
    rpc_server.addService(&bbcar_service);
    rpc_server.run();
}

int main()
{
    parallax_ping ping1(pin10);
    pc.set_baud(9600);
    encoder_ticker.attach(&encoder_control, 1ms);
    steps = 0;
    last = 0;
    u.start(callfunc);
    timer.attach(&calculate, 1s);
    s.start(rpc);
    while (1) {
        FR.output();
        FL.output();
        R.output();
        L.output();
        FR = 1;
        R = 1;
        L = 1;
        FL = 1;
        wait_us(230);
        FR.input();
        FL.input();
        R.input();
        L.input();
        wait_us(230);
        int pattern = FL * 1000 + L * 100 + R * 10 + FR;
        if (ping1 < 20) {
            ThisThread::sleep_for(200ms);
            car.turnaround();
            ThisThread::sleep_for(1200ms);
            nextt = 2;
        }
        else if (pattern == 111) {
            car.goStraight(70);
            temp--;
            if (temp <= 0) {
                nextt = 1;
            }
        }
        else if (pattern == 1111) {
            if (nextt == 1) {
                car.turn(200, -0.00000001);
                ThisThread::sleep_for(500ms);
            }
            else if (nextt == 2) {
                car.turn(200, 0.1);
                ThisThread::sleep_for(450ms);
            } else if (nextt == 0) car.goStraight(70);
        }
        else if (pattern == 1110) {
            if (first == 0 && temp3 <= 0) {
                car.turnaround();
                ThisThread::sleep_for(1200ms);
                nextt = 2;
                first++;
            }
            temp3--;
            temp1--;
            if (temp1 <= 0){
                nextt = 2;
            }
        }
        else if (pattern == 1000) {
            car.turn(200, 0.0000000001);
            pin5 = 0;
            temp = 3;
            temp1 = 3;
        }
        else if (pattern == 1100) {
            car.turn(100, 0.1);
            temp = 3;
            temp1 = 3;
        }
        else if (pattern == 100) {
            car.turn(160, 0.7);
            temp = 3;
            temp1 = 3;
        }
        else if (pattern == 110) {
            car.goStraight(70);
            temp = 3;
            temp1 = 3;
        }
        else if (pattern == 10) {
            car.turn(160, -0.5);
            temp = 3;
            temp1 = 3;
        }
        else if (pattern == 11) {
            car.turn(100, -0.1);
            temp = 3;
            temp1 = 3;
        }
        else if (pattern == 1) {
            car.turn(200, -0.00000001);
            pin6 = 0;
            temp = 3;
            temp1 = 3;
        } 
        else if (pattern == 0) {
            car.stop();
        }
        else car.goStraight(70);
        ThisThread::sleep_for(20ms);
    }
    return 0;
}
