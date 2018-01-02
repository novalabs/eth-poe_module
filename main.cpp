/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */
 
#include <ModuleConfiguration.hpp>
#include <Module.hpp>

#include <lwip_bindings/lwipthread.h>
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "chprintf.h"
#include <cstdio>


// MESSAGES
#include <core/common_msgs/Led.hpp>
#include <core/common_msgs/Float32.hpp>

// NODES
#include <core/led/Subscriber.hpp>

// BOARD IMPL

// *** DO NOT MOVE ***
Module module;

// TYPES

// NODES
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);

/*
 * Subscriber node
 */
float data = 0.0f;
core::os::Time last_data;

bool
adc_cb(
    const core::common_msgs::Float32& msg,
    core::mw::Node*                   node
)
{
    data      = msg.value;
    last_data = core::os::Time::now();

    return true;
}

void
sub_node(
    void* arg
)
{
    core::mw::Node node("sub_node");
    core::mw::Subscriber<core::common_msgs::Float32, 5> sub;

    sub.set_callback(adc_cb);

    (void)arg;
    chRegSetThreadName("sub_node");

    node.subscribe(sub, "adc");

    while (TRUE) {
        node.spin(500);
    }
}

/*
 * Server thread
 */
void
server_thread(
    void* arg
)
{
    uint16_t        port = 12345;
    struct netconn* conn, * newconn;
    err_t           err;

    (void)arg;
    chRegSetThreadName("server_thread");

    /* Make the PHY wake up.*/
    module.enablePHY();

    lwipInit(nullptr);

    chThdSleepMilliseconds(500);

    /* Create a new TCP connection handle */
    conn = netconn_new(NETCONN_TCP);

    if (conn == nullptr) {
        core::os::Thread::exit(55);
    }

    /* Bind to a port. */
    netconn_bind(conn, NULL, port);

    /* Listen for connections. */ netconn_listen(conn);

    while (TRUE) {
        err = netconn_accept(conn, &newconn);

        if (err != ERR_OK) {
            continue;
        }

        if (!newconn) {
            continue;
        }

        while (TRUE) {
            char buf[32];
            struct netbuf* nbuf;

            sprintf(buf, "%f;%f\n", core::os::Time::now().raw / 1000.0, data);
            err = netconn_write_partly(newconn, buf, strlen(buf), NETCONN_COPY, NULL);

            if (err != ERR_OK) {
                netconn_delete(newconn);
                break;
            }

            core::os::Thread::sleep(core::os::Time::ms(500));
        }
    }
} // server_thread

// MAIN
extern "C" {
    int
    main()
    {
        module.initialize();

        // Led subscriber node
        core::led::SubscriberConfiguration led_subscriber_configuration;
        led_subscriber_configuration.topic = "led";
        led_subscriber.setConfiguration(led_subscriber_configuration);

        module.add(led_subscriber);

        core::os::Thread::create_heap(NULL, THD_WORKING_AREA_SIZE(2048), core::os::Thread::PriorityEnum::NORMAL + 1, sub_node, NULL);
        core::os::Thread::create_heap(NULL, THD_WORKING_AREA_SIZE(2048), core::os::Thread::PriorityEnum::NORMAL + 1, server_thread, NULL);

        // Setup and run
        module.setup();
        module.run();

        // Is everything going well?
        for (;;) {
            if (!module.isOk()) {
                module.halt("This must not happen!");
            }

            core::os::Thread::sleep(core::os::Time::ms(500));
        }

        return core::os::Thread::OK;
    } // main
}
