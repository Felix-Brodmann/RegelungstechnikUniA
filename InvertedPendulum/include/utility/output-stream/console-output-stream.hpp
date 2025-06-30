#ifndef UTILITY_OUTPUT_STREAM_CONSOLEOUTPUTSTREAM_HPP
#define UTILITY_OUTPUT_STREAM_CONSOLEOUTPUTSTREAM_HPP

#include <iostream>
#include <map>
#include <string>

#include "utility/output-stream.hpp"

namespace control_engineering_uni_a
{
/**
 * @class ConsoleOutputStream
 * @brief An output stream that writes to the console.
 * @details This class implements the OutputStream interface and writes messages to the standard output stream (console).
 */
class ConsoleOutputStream : public OutputStream
{
public:
    /**
     * @brief Default constructor for the ConsoleOutputStream class.
     */
    ConsoleOutputStream() = default;
    
    /**
     * @brief Destructor for the ConsoleOutputStream class.
     */
    ~ConsoleOutputStream() override = default;

    /**
     * @brief Writes a string to the console.
     * @param t_component The component name or identifier that is writing to the console.
     * @param t_string The string to write to the console.
     * @param metadata THIS IS NOT USED IN THIS IMPLEMENTATION.
     */
    void write(const std::string &t_component, const std::string &t_string, const std::map<std::string, std::string>& metadata = {}) override
    {
        std::cout << "[" << t_component << "] ";
        std::cout << t_string;
        std::cout << std::endl;
    }
};

} // namespace control_engineering_uni_a

#endif // UTILITY_OUTPUT_STREAM_CONSOLEOUTPUTSTREAM_HPP