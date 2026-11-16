/* Hardware diagnostic image. Hardware actuator enable is intentionally locked.
   This is a real Teensy target, not a claim of a commissioned robot application. */
#include <Arduino.h>
#include <SPI.h>
#include <FlexCAN_T4.h>
extern "C" {
#include "bipe.h"
#include "runtime.h"
#include "generated_config.h"
}

static_assert(BIPE_HARDWARE_QUALIFIED == 0,
              "Diagnostic image cannot enable motors: use a separately reviewed integration image");
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can;
static bipe_supervisor supervisor;
static bipe_inputs inputs={};
static uint32_t last_tick=0, last_report=0;
static bool previous_button=false;
static bipe_calibration calibration={};
static bipe_estimator estimator={};
static bipe_balance balance={};
static bipe_estimate estimate={};
static double gyro_sample[3]={},accel_sample[3]={},gyro_bias[3]={};
static uint32_t last_body=0;
static bool estimate_ready=false;
static bipe_runtime runtime={};
static unsigned raw_can_frames=0;
static constexpr int CS=10, SILENT=2, RUN_REQUEST=6, STOP_LOOP=5, ARM_BUTTON=4, PG=7;

/* The qualified driver adapter must be a separately reviewed implementation.
   This diagnostic target compiles the application but cannot send any frame. */
static bool inhibited_send(void *,const bipe_frame *) { return false; }
static bool inhibited_disable(void *) { return false; }
static void inhibited_power(void *,bool,bool) {
    digitalWrite(RUN_REQUEST,LOW); digitalWrite(SILENT,HIGH);
}

static uint8_t read_register(uint8_t address) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS,LOW); SPI.transfer(address|0x80); uint8_t value=SPI.transfer(0);
    digitalWrite(CS,HIGH); SPI.endTransaction(); return value;
}
static void write_register(uint8_t address,uint8_t value) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS,LOW); SPI.transfer(address&0x7f); SPI.transfer(value);
    digitalWrite(CS,HIGH); SPI.endTransaction();
}
void setup() {
    pinMode(RUN_REQUEST,OUTPUT); digitalWrite(RUN_REQUEST,LOW);
    pinMode(SILENT,OUTPUT); digitalWrite(SILENT,HIGH);
    pinMode(CS,OUTPUT); digitalWrite(CS,HIGH);
    pinMode(ARM_BUTTON,INPUT_PULLUP); pinMode(STOP_LOOP,INPUT_PULLUP); pinMode(PG,INPUT);
    Serial.begin(115200); analogReadResolution(12); bipe_init(&supervisor);
    bipe_app_profile unavailable={};
    bipe_runtime_init(&runtime,&unavailable,{inhibited_send,inhibited_power,nullptr,inhibited_disable},0,millis());
    SPI.begin();
    /* ST DS12140: WHO_AM_I=0x6c; BDU+IF_INC, 833 Hz, ±4g / ±500dps. */
    if(read_register(0x0f)==0x6c) {
        write_register(0x12,0x44); write_register(0x10,0x78); write_register(0x11,0x74);
    }
    can.begin(); can.setBaudRate(1000000,LISTEN_ONLY);
    last_tick=millis();
}
void loop() {
    uint32_t now=millis(); if(now-last_tick<1) return; last_tick=now;
    inputs.estop_closed=digitalRead(STOP_LOOP)==LOW;
    inputs.power_good=digitalRead(PG)==HIGH;
    inputs.bus_V=analogRead(14)*(BIPE_ADC_REFERENCE_V/BIPE_ADC_MAX_COUNTS)*BIPE_BUS_ADC_SCALE;
    uint8_t status=read_register(0x1e);
    if((status&3)==3) {
        uint8_t data[12];
        SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE3));
        digitalWrite(CS,LOW); SPI.transfer(0x22|0x80);
        for(auto &value:data) value=SPI.transfer(0);
        digitalWrite(CS,HIGH); SPI.endTransaction();
        double g2=0,a2=0;
        for(unsigned axis=0;axis<3;axis++) {
            int16_t g=(int16_t)((uint16_t)data[2*axis]|((uint16_t)data[2*axis+1]<<8));
            int16_t a=(int16_t)((uint16_t)data[6+2*axis]|((uint16_t)data[7+2*axis]<<8));
            double gyro=g*0.0175*0.017453292519943295; gyro_sample[axis]=gyro;
            double accel=a*0.000122*9.81; accel_sample[axis]=accel; g2+=gyro*gyro; a2+=accel*accel;
        }
        inputs.gyro_norm=sqrt(g2); inputs.acceleration_norm=sqrt(a2);
        inputs.imu_valid=read_register(0x0f)==0x6c;
        inputs.imu_present=true; inputs.imu_time_ms=now; inputs.imu_sequence++;
    }
    CAN_message_t received;
    while(can.read(received)) {
        /* Exact AK48 codec remains undocumented; retain a raw receive count
           rather than assigning provisional ranges to real telemetry. */
        raw_can_frames++;

    }
    /* Diagnostic board-frame estimator: identity axes are provisional until the
       assembled IMU-to-chassis transform and actuator zero/sign mapping are qualified.
       Compute at 200 Hz; no control result can reach hardware in this image. */
    if(now-last_body>=5 && inputs.imu_present && inputs.imu_valid) {
        last_body=now;
        double encoder[6]; bool stationary=true;
        for(unsigned j=0;j<6;j++) {
            encoder[j]=inputs.motor[j].position;
            stationary=stationary && inputs.motor[j].present && now-inputs.motor[j].time_ms<=20 &&
                fabs(inputs.motor[j].speed)<0.05;
        }
        if(!estimate_ready) {
            double gravity_sample[3];
            if(bipe_calibrate(&calibration,gyro_sample,accel_sample,stationary,now,
                               inputs.imu_sequence,gyro_bias,gravity_sample))
                estimate_ready=bipe_estimator_init(&estimator,gravity_sample,encoder);
        } else {
            double corrected[3]; for(unsigned j=0;j<3;j++) corrected[j]=gyro_sample[j]-gyro_bias[j];
            bipe_sensor_config cfg=bipe_default_sensor_config();
            if(!bipe_estimator_step(&estimator,&cfg,corrected,accel_sample,encoder,&estimate))
                estimate_ready=false;
            /* Profile false also covers unqualified encoder/axis mapping. */
            if(BIPE_HARDWARE_QUALIFIED && estimate_ready) {
                const double angle=0.6981317007977318;
                double qr[4]={angle,-2*angle,angle,-2*angle},qd[4]={},torque[6],ff[4],reference;
                bool valid=bipe_control_step(&balance,&estimate,0,0,qr,qd,0.005,torque,ff,&reference);
                inputs.attitude_error=valid?estimate.pitch:1.0;
                /* Motor command transport remains absent until exact profile qualification. */
            }
        }
    }
    bool button=digitalRead(ARM_BUTTON)==LOW;
    if(button && !previous_button) bipe_start(&supervisor);
    if(button && !previous_button) bipe_start(&runtime.application.supervisor);
    previous_button=button;
    /* Profile gate is compile-time false. CAN stays listen-only and run request low. */
    bipe_settings settings=bipe_default_settings();
    bipe_tick(&supervisor,&settings,&inputs,now,BIPE_HARDWARE_QUALIFIED);
    bipe_runtime_tick(&runtime,nullptr,now);
    digitalWrite(RUN_REQUEST,LOW); digitalWrite(SILENT,HIGH);
    if(now-last_report>=1000 && Serial.availableForWrite()>64) {
        last_report=now;
        Serial.printf("bipe diagnostics: state=%u fault=%u bus=%.2f IMU=%u hardware_enable=0\n",
                      (unsigned)supervisor.state,(unsigned)supervisor.fault,
                      inputs.bus_V,(unsigned)inputs.imu_valid);
    }
}
