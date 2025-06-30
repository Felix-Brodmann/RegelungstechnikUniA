#ifndef UTILITY_OUTPUT_STREAM_VOIDOUTPUTSTREAM_HPP
#define UTILITY_OUTPUT_STREAM_VOIDOUTPUTSTREAM_HPP

#include <string>
#include <map>

#include "utility/output-stream.hpp"

namespace control_engineering_uni_a
{
/**
 * @class VoidOutputStream
 * @brief An output stream that does nothing.
 * @details This class implements the OutputStream interface but does not perform any output operations.
 */
class VoidOutputStream : public OutputStream
{
public:
    /**
     * @brief Default constructor for the VoidOutputStream class.
     */
    VoidOutputStream() = default;

    /**
     * @brief Destructor for the VoidOutputStream class.
     */
    ~VoidOutputStream() override = default;

    /**
     * @brief Writes a string to the void output stream.
     * @param t_component The component name or identifier that is writing to the output stream.
     * @param t_string The string to write to the output stream.
     * @param metadata THIS IS NOT USED IN THIS IMPLEMENTATION.
     */
    void write(const std::string &t_component, const std::string &t_string, const std::map<std::string, std::string>& metadata = {}) override
    {
        // This method intentionally does nothing
        (void)t_component; // Suppress unused parameter warning
        (void)t_string;    // Suppress unused parameter warning
        (void)metadata;    // Suppress unused parameter warning
    }
};

} // namespace control_engineering_uni_a

#endif // UTILITY_OUTPUT_STREAM_VOIDOUTPUTSTREAM_HPP