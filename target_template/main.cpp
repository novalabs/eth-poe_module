/* COPYRIGHT (c) 2016-2018 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */
 
#include <ModuleConfiguration.hpp>
#include <Module.hpp>

// MESSAGES
#include <core/common_msgs/Led.hpp>

// NODES
#include <core/led/Subscriber.hpp>

// BOARD IMPL

// *** DO NOT MOVE ***
Module module;

// TYPES

// NODES
core::led::Subscriber led_subscriber("led_subscriber", core::os::Thread::PriorityEnum::LOWEST);

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

        module.nodes().add(led_subscriber);

        // Setup and run
        module.nodes().setup();
        module.nodes().run();

        // Is everything going well?
        for (;;) {
            if (!module.nodes().areOk()) {
                module.halt("This must not happen!");
            }

            core::os::Thread::sleep(core::os::Time::ms(500));
        }

        return core::os::Thread::OK;
    } // main
}
