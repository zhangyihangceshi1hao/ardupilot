#include <AP_Math/AP_Math.h>
#include "FD_DATA.h"

extern const AP_HAL::HAL& hal;

// storage object
StorageAccess FD_DATA::_storage(StorageManager::StorageFDData);

assert_storage_size<Serial_number, 4> _assert_storage_size_Serial_number;
/*
 * init - perform required initialisation
 */

FD_DATA *FD_DATA::_singleton;

// constructor
FD_DATA::FD_DATA()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("FD_DATA must be singleton");
    }
    _singleton = this;
}

void FD_DATA::update()
{
    ;
}

void FD_DATA::handle_message(const mavlink_message_t &msg)
{
    handle_message_txhy_sn(msg);
    handle_message_command_long_txhy_sn(msg);
    handle_message_power_control(msg);
}

namespace AP {

FD_DATA &fd_data()
{
    return *FD_DATA::get_singleton();
}

};
