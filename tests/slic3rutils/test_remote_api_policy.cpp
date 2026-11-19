#include <catch2/catch_test_macros.hpp>

#include <string>

#include "slic3r/GUI/RemoteAPI/RemoteAPIController.hpp"

using Slic3r::GUI::RemoteAPI::Controller;

TEST_CASE("Custom G-code parameters are not behind the script and G-code policy", "[RemoteAPI]")
{
    for (const std::string key : { "custom_parameters_print", "custom_parameters_printer", "custom_parameters_filament" }) {
        CAPTURE(key);
        CHECK_FALSE(Controller::sensitive_config_key(key));
    }
    CHECK(Controller::sensitive_config_key("machine_start_gcode"));
    CHECK(Controller::sensitive_config_key("change_filament_gcode"));
    CHECK(Controller::sensitive_config_key("post_process"));
}
