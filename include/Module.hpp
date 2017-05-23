/* COPYRIGHT (c) 2016-2017 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/CoreModule.hpp>
#include <core/os/IOChannel.hpp>
#include <ModuleConfiguration.hpp>

namespace core {
namespace hw {
class Pad;
}
}

class Module:
    public core::mw::CoreModule
{
public:
// --- DEVICES ----------------------------------------------------------------
    static core::hw::Pad& sd_led;

    static core::os::IOChannel& stream;
    static core::os::IOChannel& serial;
// ----------------------------------------------------------------------------

// --- Ethernet PHY control ---------------------------------------------------
    static void
    enablePHY();

    static void
    disablePHY();
// ----------------------------------------------------------------------------

    static bool
    initialize();


    Module();
    virtual ~Module() {}
};
