// Copyright (c) 2014, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <iostream>
#include <signal.h>
#include <vsm/vsm.h>
#include <vsm/callback.h>
#include <vsm/run_as_service.h>
#include <ardupilot_vehicle_manager.h>
#include <adsb_manager.h>

#define ADSB_DISABLED

#ifdef __unix__
#include <signal.h>
#endif /* __unix__ */

bool terminate;

#ifdef __unix__
void Sigint_handler(int signum __UNUSED)
{
    LOG_INFO("Signal caught, exiting...");
    terminate = true;
}
#endif /* __unix__ */


Ardupilot_vehicle_manager::Ptr manager;
Adsb_manager::Ptr adsb_manager;

int
start_main(int argc, char *argv[])
{
    vsm::Initialize(argc, argv);
    manager = Ardupilot_vehicle_manager::Create();
    manager->Enable();

#ifndef ADSB_DISABLED
    adsb_manager = Adsb_manager::Create("vehicle.apm.serial_port");
    adsb_manager->Enable();
#endif
    return 0;
}

void
stop_main()
{
    manager->Disable();
    manager = nullptr;

#ifndef ADSB_DISABLED
    adsb_manager->Disable();
    adsb_manager = nullptr;
#endif
    vsm::Terminate();
}

void
wait_for_termination()
{
    while(!terminate) {
        /* Think about better way. */
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int
main (int argc, char *argv[])
{
    if (vsm::Run_as_service(
            "ugcs-vsm-ardupilot",
            argc,
            argv,
            vsm::Make_program_init_handler(start_main),
            vsm::Make_callback(stop_main)))
        return 0;

#ifdef __unix__
    struct sigaction action;
    memset(&action, 0, sizeof(action));
    action.sa_handler = Sigint_handler;
    sigaction(SIGINT, &action, NULL);
#endif /* __unix__ */

    start_main(argc, argv);
    wait_for_termination();
    stop_main();
    return 0;
}
