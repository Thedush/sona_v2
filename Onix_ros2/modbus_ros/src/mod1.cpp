#include <iostream>
#include <stdio.h>
#include "modbus_ros/modbus.h"

#include "modbus_ros/modbus-rtu.h"
#include <chrono>
#include <thread>


using namespace std;

void delay(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

int main() 
{
    modbus_t *ctx = NULL;
    modbus_t *ctx_plc = NULL;
    uint16_t tab_reg[64];
    int rc, rc1;
    int i;
    // ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);

    // if (ctx == NULL) 
    // {
    // fprintf(stderr, "Unable to create the libmodbus context\n");
    // return -1;
    // }
    // int status = modbus_connect(ctx);
    // cout << status <<endl;
    // if (status == -1) {
    // // fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
    // modbus_free(ctx);
    // return -1;
    // }

      ctx_plc = modbus_new_rtu("/dev/ttyplc", 115200, 'N', 8, 1);

     if (ctx_plc == NULL) 
     {
     fprintf(stderr, "Unable to create the libmodbus context\n");
     return -1;
     }
     int status = modbus_connect(ctx_plc);
     cout << status <<endl;
     if (status == -1) {
     fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
     modbus_free(ctx_plc);
     return -1;
    }
    /*
    rc1 = modbus_set_slave(ctx, 2);
    cout << "Battery connected"<<status;
    rc = modbus_read_registers(ctx, 40, 1, tab_reg);
    cout <<"\n "<< "Reg Count " <<rc << std::endl;
    if (rc == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }
    for (i=0; i < rc; i++) {
        printf("reg[%d]=%d (0x%X)\n", 20+i, tab_reg[i], tab_reg[i]);
    }
    delay(10);
*/
    // rc1 = modbus_set_slave(ctx, 2);
    // cout << "Sonar2 connected"<<status;
    // rc = modbus_read_registers(ctx, 262, 4, tab_reg);
    // cout <<"\n "<< "Reg Count " <<rc << std::endl;
    // if (rc == -1) {
    //     fprintf(stderr, "%s\n", modbus_strerror(errno));
    //     return -1;
    // }
    // for (i=0; i < rc; i++) {
    //     printf("reg[%d]=%d (0x%X)\n", 262+i, tab_reg[i], tab_reg[i]);
    // }
    // delay(10);

    // rc1 = modbus_set_slave(ctx, 3);
    // cout << "Sonar1 connected"<<status;
    // rc = modbus_read_registers(ctx, 262, 4, tab_reg);
    // cout <<"\n "<< "Reg Count " <<rc << std::endl;
    // if (rc == -1) {
    //     fprintf(stderr, "%s\n", modbus_strerror(errno));
    //     return -1;
    // }
    // for (i=0; i < rc; i++) {
    //     printf("reg[%d]=%d (0x%X)\n", 262+i, tab_reg[i], tab_reg[i]);
    // }
    // delay(10);
/*
     // line array
    rc1 = modbus_set_slave(ctx, 3);
    cout << "Magnetic tape connected"<<status;
    rc = modbus_read_input_registers(ctx, 1000, 10, tab_reg);
    cout <<"\n "<< "Reg Count " <<rc << std::endl;
    if (rc == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }
    for (i=0; i < rc; i++) {
        printf("reg[%d]=%d (0x%X)\n", 1000+i, tab_reg[i], tab_reg[i]);
    }
    delay(10);

    rc1 = modbus_set_slave(ctx, 5);
    cout << "RFID tape connected"<<status;
    rc = modbus_read_input_registers(ctx, 1000, 10, tab_reg);
    cout <<"\n "<< "Reg Count " <<rc << std::endl;
    if (rc == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }
    for (i=0; i < rc; i++) {
        printf("reg[%d]=%d (0x%X)\n", 1000+i, tab_reg[i], tab_reg[i]);
    }
    delay(10);
   */    
    rc1 = modbus_set_slave(ctx_plc, 1);
    cout << "PLC connected"<<status;
    rc = modbus_read_registers(ctx_plc, 4246, 10, tab_reg);
    cout <<"\n "<< "Reg Count " <<rc << std::endl;
    if (rc == -1) {
        fprintf(stderr, "%s\n", modbus_strerror(errno));
        return -1;
    }

    for (i=0; i < rc; i++) {
        printf("reg[%d]=%d (0x%X)\n", 4246+i, tab_reg[i], tab_reg[i]);
    }

    // rc = modbus_write_bit(ctx, 2050, 1);
    // cout << "memory on "<<rc << endl;
    // rc = modbus_write_bit(ctx, 2050, 0);
    // cout << "memory on "<<rc << endl;
    // rc = modbus_write_register(ctx, 4196, 0);
    // cout << "memory on "<<rc << endl;
    // rc = modbus_write_register(ctx, 4198, 0);
    // cout << "memory on "<<rc << endl;
    // uint16_t write_reg[4] = {0};
    // write_reg[0]= 2000;
    // // write_reg[1]=
    // write_reg[2]= 2000;
    // rc = modbus_write_registers(ctx,4196,4,write_reg);
    // cout << "memory on "<<rc << endl;
    


modbus_close(ctx);
modbus_free(ctx);
}