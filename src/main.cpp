#include "drivingAgent.hpp"
#include "interfaces_data_structs.h"
#include "additional_logging.h"
#include <math.h>
#include <csignal>
#include <iostream>
#include <fstream>
#include <string>

extern "C" {
#include "screen_print_c.h"
}
#include "server_lib.h"

#define DEFAULT_SERVER_IP    "127.0.0.1"
#define SERVER_PORT               30000  // Server port

static uint32_t server_run = 1;
void handle_sigint(int) {
    server_run = 0;
}

int main(int argc, const char * argv[]) {
    std::signal(SIGINT, handle_sigint);

    std::ofstream statsFile("stats.csv");
    statsFile << "time,velocity,acceleration" << std::endl;

    server_agent_init(DEFAULT_SERVER_IP, SERVER_PORT);

    // Start server of the Agent
    std::cout << "Waiting for scenario message..." << std::endl;

    DrivingAgent drivingAgent;

    while (server_run == 1) {

        scenario_msg_t scenario_msg;
        uint32_t message_id = 0;

        // Receive scenario message from the environment
        if (server_receive_from_client(&server_run, &message_id, &scenario_msg.data_struct) == 0) {
            // Init time
            static auto start = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::now()-start;
            double num_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(time).count()/1000.0;
            // std::cout << "received message at " << num_seconds << " seconds." << std::endl;

            // Data struct
            input_data_str in = scenario_msg.data_struct;
            output_data_str manoeuvre_msg = drivingAgent.computeManoeuvre(num_seconds, in);

            statsFile << num_seconds << "," << in.VLgtFild << "," << in.ALgtFild << std::endl;

            // Example of using log
            // std::cout << in << std::endl;
            // statsFile << in.CycleNumber << "," << num_seconds << "," << in.VLgtFild << std::endl;
            // std::cout << manoeuvre_msg << std::endl;

            // ADD AGENT CODE HERE

            // ADD LOW LEVEL CONTROL CODE HERE
            // Write log

            // Screen print

            // Send manoeuvre message to the environment
            if (server_send_to_client(server_run, message_id, &manoeuvre_msg) == -1) {
                perror("error send_message()");
                exit(EXIT_FAILURE);
            }
        }

    }

    // Close the server of the agent
    server_agent_close();
    return 0;
}
