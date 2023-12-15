#include "modbus_ros/modbus_ros.h"
#include <chrono>
#include <thread>

Modbus::Modbus() : Node("modbus")
{
    // initialization
    service_ = this->create_service<agv_srv::srv::ModbusNormal>("modbus",
                                                                   std::bind(&Modbus::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Modbus Server stated");
}



bool Modbus::handle_service(const std::shared_ptr<agv_srv::srv::ModbusNormal::Request> request,
                            std::shared_ptr<agv_srv::srv::ModbusNormal::Response> response)
{
    response->success = true;
    // response->message = "Sucess";
    // for (auto val : request->write_data)
    // {
    //     //    std::cout << " " << val;
    //     RCLCPP_INFO(this->get_logger(), "Received Data: %ld,", val);
    // }
    RCLCPP_INFO(this->get_logger(), "Received request: slave_id=%ld, slave address=%ld, function=%ld", request->slave_id, request->slave_address, request->function);
    // RCLCPP_INFO(this->get_logger(), "Sending response: [%ld]", response->message);

    ctx = modbus_new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1);
    if (ctx == NULL)
    {
        // fprintf(stderr, "Unable to create the libmodbus context\n");
        response->message = "Unable to create the modbus port";
        response->success = false;
        modbus_free(ctx);
        return false;
    }

    int status = modbus_connect(ctx);
    // cout << status <<endl;
    if (status == -1)
    {
        // fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        response->message = "Connection failed";
        response->success = false;
        modbus_free(ctx);
        return false;
    }

    int remote_connection, remote_connection1;
    int i;
    remote_connection1 = modbus_set_slave(ctx, request->slave_id);
    if (remote_connection1 == -1)
    {
        // fprintf(stderr, "%s\n", modbus_strerror(errno));
        response->message = "Unable to set slave";
        response->success = false;
        modbus_free(ctx);
        return false;
    }
    // cout << "Sonar2 connected"<<status;

    switch (request->function)
    {
    // Modbus Read Bits
    // checked
    case 0:
        remote_connection = modbus_read_bits(ctx, request->slave_address, request->quantity, read_coil);
        // cout <<"\n "<< "Reg Count " <<remote_connection << std::endl;
        if (remote_connection == -1)
        {
            // fprintf(stderr, "%s\n", modbus_strerror(errno));
            response->message = "Unable to do modbus_read_bits, check slave_address and  quantity is correct";
            response->success = false;
            modbus_free(ctx);
            return false;
        }
        for (i = 0; i < remote_connection; i++)
        {   response->read_coil.resize(remote_connection);
            response->read_coil[i] = read_coil[i];
            printf("reg[%d]=%d (0x%X)\n", request->slave_address + i, read_coil[i], read_coil[i]);
        }
        break;
    // Modbus Input Read Bits
    // checked
    case 1:
        remote_connection = modbus_read_input_bits(ctx, request->slave_address, request->quantity, read_coil);
        // cout <<"\n "<< "Reg Count " <<remote_connection << std::endl;
        if (remote_connection == -1)
        {
            // fprintf(stderr, "%s\n", modbus_strerror(errno));
            response->message = "Unable to do modbus_read_input_bits, check slave_address and  quantity is correct";
            response->success = false;
            modbus_free(ctx);
            return false;
        }
        for (i = 0; i < remote_connection; i++)
        {   response->read_coil.resize(remote_connection);
            response->read_coil[i] = read_coil[i];
            printf("reg[%d]=%d (0x%X)\n", request->slave_address + i, read_coil[i], read_coil[i]);
        }
        break;
    // Modbus Read Registers
    // checked
    case 2:
        remote_connection = modbus_read_registers(ctx, request->slave_address, request->quantity, read_register);
        // cout <<"\n "<< "Reg Count " <<remote_connection << std::endl;
        if (remote_connection == -1)
        {
            // fprintf(stderr, "%s\n", modbus_strerror(errno));
            response->message = "Unable to read the modbus_read_registers check slave_address and  quantity is correct";
            response->success = false;
            modbus_free(ctx);
            return false;
        }

        for (i = 0; i < remote_connection; i++)
        {   response->read_data.resize(remote_connection);
            response->read_data[i] = read_register[i] ;
            printf("reg[%d]=%d (0x%X)\n", request->slave_address + i, read_register[i], read_register[i]);
        }
        break;
    // Modbus Read Input registers
    // checked
    case 3:
        remote_connection = modbus_read_input_registers(ctx, request->slave_address, request->quantity, read_register);
        // cout <<"\n "<< "Reg Count " <<remote_connection << std::endl;
        if (remote_connection == -1)
        {
            // fprintf(stderr, "%s\n", modbus_strerror(errno));
            response->message = "Unable to do modbus_read_input_registers, check slave_address and  quantity is correct";
            response->success = false;
            modbus_free(ctx);
            return false;
        }
        for (i = 0; i < remote_connection; i++)
        {   response->read_data.resize(remote_connection);
            response->read_data[i] = read_register[i] ;
            printf("reg[%d]=%d (0x%X)\n", request->slave_address + i, read_register[i], read_register[i]);
        }
        break;
    // Modbus Write bit
    // checked
    case 4:
        remote_connection = modbus_write_bit(ctx, request->slave_address, request->write_data[0]);
        // cout <<"\n "<< "Reg Count " <<remote_connection << std::endl;
        if (remote_connection == -1)
        {
            // fprintf(stderr, "%s\n", modbus_strerror(errno));
            response->message = "Unable to do modbus_write_bit, check slave_address and  quantity is correct";
            response->success = false;
            modbus_free(ctx);
            return false;
        }

        break;
    // Modbus Write Register
    // checked
    case 5:

        remote_connection = modbus_write_register(ctx, request->slave_address, request->write_data[0]);
        // cout <<"\n "<< "Reg Count " <<remote_connection << std::endl;
        if (remote_connection == -1)
        {
            // fprintf(stderr, "%s\n", modbus_strerror(errno));
            response->message = "Unable to do modbus_write_register, check slave_address and  quantity is correct";
            response->success = false;
            modbus_free(ctx);
            return false;
        }

        break;
    // Modbus Write Bits
    // checked
    case 6:
        {
            int quantity = request->quantity;
            int received_input = request->write_data.size() ;
            if (quantity == received_input)
            {
                
                uint8_t *write_coil = (uint8_t *)malloc(request->quantity * sizeof(uint8_t));

                
                for (int i = 0; i < request->quantity; i++)
                {
                    write_coil[i] = request->write_data[i];
                }

                remote_connection = modbus_write_bits(ctx, request->slave_address, request->quantity, write_coil);
                // cout <<"\n "<< "Reg Count " <<remote_connection << std::endl;
                if (remote_connection == -1)
                {
                    // fprintf(stderr, "%s\n", modbus_strerror(errno));
                    response->message = "Unable to read the modbus_write_bits ,check slave_address and  quantity is correct";
                    response->success = false;
                    modbus_free(ctx);
                    free(write_coil);
                    return false;
                }

                // Free the dynamically allocated memory
                free(write_coil);
            
            }
            else
            {
                response->message = "quantity and write data length is not same";
                response->success = false;
                modbus_free(ctx);
                return false;
            }
            
            break;
        }
        break;

    // Modbus write Registers
    // checked
    case 7:
        {
           
            int quantity = request->quantity;
            int received_input = request->write_data.size() ;
            if (quantity == received_input)
            {
                
                uint16_t *write_reg = (uint16_t *)malloc(request->quantity * sizeof(uint16_t));

                
                for (int i = 0; i < request->quantity; i++)
                {
                    write_reg[i] = request->write_data[i];
                }

                remote_connection = modbus_write_registers(ctx, request->slave_address, request->quantity, write_reg);
                // cout <<"\n "<< "Reg Count " <<remote_connection << std::endl;
                if (remote_connection == -1)
                {
                    // fprintf(stderr, "%s\n", modbus_strerror(errno));
                    response->message = "Unable to read the modbus_write_registers ,check slave_address and  quantity is correct";
                    response->success = false;
                    modbus_free(ctx);
                    free(write_reg);
                    return false;
                }

                // Free the dynamically allocated memory
                free(write_reg);
            
            }
            else
            {
                response->message = "quantity and write data length is not same";
                response->success = false;
                modbus_free(ctx);
                return false;
            }
            
            break;
        }
    // TODO modbus_write_and_read_registers 
    case 8:
        // TODO
        break;
    default:
        response->message = "Unknown Function code";
        response->success = false;
        modbus_free(ctx);
        return false;
        break;
    }

    response->success = true;
    response->message = "Successfully processed";

    modbus_close(ctx);
    modbus_free(ctx);
    return true;
}
void Modbus::delay(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}