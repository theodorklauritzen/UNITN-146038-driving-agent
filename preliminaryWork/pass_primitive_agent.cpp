#include <iostream>
#include <string.h>
#include <math.h>
#include <csignal>

#include "server_lib.h"
#include "additional_logging.h"
#include "scenario_constants.h"

// --- MATLAB PRIMITIVES INCLUDE ---
// #include "primitives.h"
// --- MATLAB PRIMITIVES INCLUDE ---

#define DEFAULT_SERVER_IP    "127.0.0.1"
#define SERVER_PORT               30000  // Server port

static uint32_t server_run = 1;
void handle_sigint(int) {
    server_run = 0;
}

int main(int argc, const char * argv[]) {
    std::signal(SIGINT, handle_sigint);

    server_agent_init(DEFAULT_SERVER_IP, SERVER_PORT);

    // Start server of the Agent
    std::cout << "Waiting for scenario message..." << std::endl;

    double targetVelocity = 10;
    double integratedError = 0;

    constexpr double regP = 1;
    constexpr double regI = 0.1;

    double lastMessageTime = 0;

    while (server_run == 1) {

        scenario_msg_t scenario_msg;
        output_data_str manoeuvre_msg;
        uint32_t message_id = 0;

        // Receive scenario message from the environment
        if (server_receive_from_client(&server_run, &message_id, &scenario_msg.data_struct) == 0) {
            // Init time
            static auto start = std::chrono::system_clock::now();
            auto time = std::chrono::system_clock::now()-start;
            double num_seconds = std::chrono::duration_cast<std::chrono::milliseconds>(time).count()/1000.0;
            std::cout << "received message at " << num_seconds << " seconds." << std::endl;

            // Data struct
            input_data_str *in = &scenario_msg.data_struct;
            manoeuvre_msg.CycleNumber = in->CycleNumber;
            manoeuvre_msg.Status = in->Status;

            // Example of using log
            std::cout << scenario_msg.data_struct << std::endl;

            // ADD AGENT CODE HERE

            // ADD LOW LEVEL CONTROL CODE HERE
            double regError = targetVelocity - in->VLgtFild;
            integratedError += regError * (num_seconds - lastMessageTime);
            manoeuvre_msg.RequestedAcc = regError * regP + integratedError * regI;

            manoeuvre_msg.TargetSpeed = targetVelocity;
            manoeuvre_msg.RequestedSteerWhlAg = 0.0;

            // Write log

            // Screen print

            // Send manoeuvre message to the environment
            if (server_send_to_client(server_run, message_id, &manoeuvre_msg) == -1) {
                perror("error send_message()");
                exit(EXIT_FAILURE);
            } else {
                std::cout << "Message sent" << std::endl;
            }

            lastMessageTime = num_seconds;
        }

    }

    // Close the server of the agent
    server_agent_close();
    return 0;
}
