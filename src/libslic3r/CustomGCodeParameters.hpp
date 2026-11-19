#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace Slic3r {

class DynamicConfig;

// User defined values of the process, printer and filament profiles (custom_parameters_print,
// custom_parameters_printer, custom_parameters_filament), each a flat JSON object of numbers and
// booleans such as {"chamber": 45, "soak": true}. Custom G-code reads them as
// custom_parameter_print_<name>, custom_parameter_printer_<name> and custom_parameter_filament_<name>.
// Text values are refused on purpose: a number or a boolean cannot carry G-code commands, so these
// settings stay plain data and are not guarded as G-code templates by the Remote API.
namespace CustomGCodeParameters {

inline constexpr std::string_view variable_prefix          = "custom_parameter_";
inline constexpr std::string_view filament_variable_prefix = "custom_parameter_filament_";

inline bool is_variable(std::string_view name) { return name.substr(0, variable_prefix.size()) == variable_prefix; }
inline bool is_filament_variable(std::string_view name)
{
    return name.substr(0, filament_variable_prefix.size()) == filament_variable_prefix;
}

// Returns a translated description of the first problem found, or an empty string.
std::string validate(const std::string &print, const std::string &printer, const std::vector<std::string> &filaments);

// Adds the placeholder variables to the given dictionary. The filament ones are vectors with one
// element per filament, nil where a filament does not define the parameter; filament_count sizes them
// when the setting holds fewer entries than there are filaments.
// Throws Slic3r::PlaceholderParserError with the message of validate() when the settings are not valid.
void apply(DynamicConfig &variables, const std::string &print, const std::string &printer, const std::vector<std::string> &filaments,
           size_t filament_count);

} // namespace CustomGCodeParameters

} // namespace Slic3r
