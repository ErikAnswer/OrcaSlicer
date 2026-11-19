#include "CustomGCodeParameters.hpp"

#include "Config.hpp"
#include "Exception.hpp"
#include "I18N.hpp"
#include "format.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>

#include "nlohmann/json.hpp"

namespace Slic3r {
namespace CustomGCodeParameters {

namespace {

enum class ValueType { Boolean, Integer, Real };

struct Value
{
    ValueType type;
    double    number;
};

using Parameters = std::map<std::string, Value>;

class InvalidParameters : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

bool is_valid_name(const std::string &name)
{
    const auto letter = [](char c) { return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_'; };
    const auto digit  = [](char c) { return c >= '0' && c <= '9'; };
    return !name.empty() && letter(name.front()) &&
           std::all_of(name.begin(), name.end(), [&](char c) { return letter(c) || digit(c); });
}

std::optional<Value> to_value(const nlohmann::json &json)
{
    if (json.is_boolean())
        return Value{ValueType::Boolean, json.get<bool>() ? 1. : 0.};
    if (!json.is_number())
        return std::nullopt;
    const double number = json.get<double>();
    if (!std::isfinite(number))
        return std::nullopt;
    // The integer nil marker (the largest int) is left to the reals, so a defined value never reads as undefined.
    const bool fits_int = number > double(std::numeric_limits<int>::min()) && number < double(std::numeric_limits<int>::max());
    return Value{json.is_number_integer() && fits_int ? ValueType::Integer : ValueType::Real, number};
}

// source names the profile in the error messages, already translated.
Parameters parse(const std::string &text, const std::string &source)
{
    Parameters parameters;
    if (text.find_first_not_of(" \t\r\n") == std::string::npos)
        return parameters;
    const nlohmann::json json = nlohmann::json::parse(text, nullptr, false);
    if (!json.is_object())
        throw InvalidParameters(Slic3r::format(
            _u8L("The custom parameters of %1% must be a JSON object of numbers and true/false values, for example {\"chamber\": 45, \"soak\": true}."),
            source));
    for (const auto &[name, json_value] : json.items()) {
        if (!is_valid_name(name))
            throw InvalidParameters(Slic3r::format(
                _u8L("Custom parameters of %1%: the name \"%2%\" may only contain letters, digits and underscores, and must not start with a digit."),
                source, name));
        const std::optional<Value> value = to_value(json_value);
        if (!value)
            throw InvalidParameters(Slic3r::format(
                _u8L("Custom parameters of %1%: the value of \"%2%\" must be a number or true/false. Text values are not allowed."), source, name));
        parameters.emplace(name, *value);
    }
    return parameters;
}

ConfigOption *scalar_option(const Value &value)
{
    switch (value.type) {
    case ValueType::Boolean: return new ConfigOptionBool(value.number != 0.);
    case ValueType::Integer: return new ConfigOptionInt(int(value.number));
    default: return new ConfigOptionFloat(value.number);
    }
}

std::string filament_source(size_t filament_idx)
{
    // TRN %1% is the filament number as shown in the filament list, starting from 1.
    return Slic3r::format(_u8L("filament %1%"), filament_idx + 1);
}

// The common type of a parameter over the filaments that define it. Integers widen to reals,
// while booleans do not mix with numbers.
ValueType filament_value_type(const std::string &name, const std::vector<Parameters> &filaments)
{
    std::optional<ValueType> common;
    for (const Parameters &parameters : filaments) {
        const auto it = parameters.find(name);
        if (it == parameters.end())
            continue;
        const ValueType type = it->second.type;
        if (!common)
            common = type;
        else if ((*common == ValueType::Boolean) != (type == ValueType::Boolean))
            throw InvalidParameters(Slic3r::format(
                _u8L("The custom parameter \"%1%\" is a number in some filaments and true/false in others. It must be of the same kind in every filament."),
                name));
        else if (type == ValueType::Real)
            common = ValueType::Real;
    }
    return *common;
}

template<class Option, class Convert>
ConfigOption *filament_option(const std::string &name, const std::vector<Parameters> &filaments, Convert convert)
{
    auto option = new Option();
    option->values.reserve(filaments.size());
    for (const Parameters &parameters : filaments) {
        const auto it = parameters.find(name);
        option->values.emplace_back(it == parameters.end() ? Option::nil_value() : convert(it->second.number));
    }
    return option;
}

struct Parsed
{
    Parameters              print;
    Parameters              printer;
    std::vector<Parameters> filaments;
    std::map<std::string, ValueType> filament_types;
};

Parsed parse_all(const std::string &print, const std::string &printer, const std::vector<std::string> &filaments)
{
    Parsed parsed;
    parsed.print   = parse(print, _u8L("the process"));
    parsed.printer = parse(printer, _u8L("the printer"));
    parsed.filaments.reserve(filaments.size());
    for (size_t idx = 0; idx < filaments.size(); ++idx)
        parsed.filaments.emplace_back(parse(filaments[idx], filament_source(idx)));
    std::set<std::string> names;
    for (const Parameters &parameters : parsed.filaments)
        for (const auto &[name, value] : parameters)
            names.insert(name);
    for (const std::string &name : names)
        parsed.filament_types.emplace(name, filament_value_type(name, parsed.filaments));
    return parsed;
}

} // namespace

std::string validate(const std::string &print, const std::string &printer, const std::vector<std::string> &filaments)
{
    try {
        parse_all(print, printer, filaments);
    } catch (const InvalidParameters &err) {
        return err.what();
    }
    return {};
}

void apply(DynamicConfig &variables, const std::string &print, const std::string &printer, const std::vector<std::string> &filaments,
           size_t filament_count)
{
    Parsed parsed;
    try {
        parsed = parse_all(print, printer, filaments);
    } catch (const InvalidParameters &err) {
        throw Slic3r::PlaceholderParserError(err.what());
    }
    if (parsed.filaments.size() < filament_count)
        parsed.filaments.resize(filament_count);
    const std::string prefix(variable_prefix);
    for (const auto &[name, value] : parsed.print)
        variables.set_key_value(prefix + "print_" + name, scalar_option(value));
    for (const auto &[name, value] : parsed.printer)
        variables.set_key_value(prefix + "printer_" + name, scalar_option(value));
    for (const auto &[name, type] : parsed.filament_types) {
        ConfigOption *option = nullptr;
        switch (type) {
        case ValueType::Boolean:
            option = filament_option<ConfigOptionBoolsNullable>(name, parsed.filaments, [](double v) { return (unsigned char) (v != 0.); });
            break;
        case ValueType::Integer:
            option = filament_option<ConfigOptionIntsNullable>(name, parsed.filaments, [](double v) { return int(v); });
            break;
        default:
            option = filament_option<ConfigOptionFloatsNullable>(name, parsed.filaments, [](double v) { return v; });
        }
        variables.set_key_value(std::string(filament_variable_prefix) + name, option);
    }
}

} // namespace CustomGCodeParameters
} // namespace Slic3r
