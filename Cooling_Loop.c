#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

// PID declarations - tuned based on python simulation script
#define STEPS 100
#define DT 0.1
#define TAU 2.0
#define KP 1.2
#define KI 0.4
#define KD 0.05

// I've assumed CAN APIs are available, and that we do not need to go through bit-manipulation in translating CAN messages
extern int CAN_Send(uint32_t id, uint8_t* data, uint8_t dataLength);
extern int CAN_Receive(uint32_t* id, uint8_t* data, uint8_t* dataLength);

// send temperature data over CAN
void sendTemperature(float temperature) {
    uint8_t data[4];
    memcpy(data, &temperature, sizeof(float)); // store data into temperature address
    CAN_Send(0x010, data, 4);  // ID 0x010 = temperature sensor
}

// read temperature data over CAN
float readTemperature() {
    uint32_t id;
    uint8_t data[8];
    uint8_t dataLength;
    float temperature = 0.0; // emulated temperature

    if (CAN_Receive(&id, data, &dataLength) == 0 && id == 0x010 && dataLength == 4) {
        memcpy(&temperature, data, sizeof(float));
    }

    return temperature;
}

void main() {
    // PID temperatures
    float setpoint = 0.0; // desired temperature 
    float output = 0.0; // PID controller output
    float measured = 5.0; // default emulated temperature 

    // PID variables 
    float error = 0.0;
    float prevError = 0.0;
    float integral = 0.0;
    float derivative = 0.0;
    float control = 0.0;

    // print plant response 
    printf("Step\tMeasuredTemp\tControlOutput\n");

    for (int i = 0; i < STEPS; ++i) {
        // simulate sending temperature data over CAN (ie: to Power View Screen)  
        sendTemperature(output);

        // read temperature over CAN
        measured = readTemperature();

        // PID computation  
        // for the sake of simplicity, I've assumed we're directly controlling the temperature for this PID loop, whereas in reality, the output should go to the motors
        error = setpoint - measured; // proportional
        integral += error * DT;
        derivative = (error - prevError) / DT;
        control = KP * error + KI * integral + KD * derivative;
        prevError = error;

        // PID controller output
        output += (control - output) * DT / TAU;

        printf("%d\t%.2f\t\t%.2f\n", i, output, control);
    }

}
