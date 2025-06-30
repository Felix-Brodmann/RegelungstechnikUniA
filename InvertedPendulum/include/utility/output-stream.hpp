#ifndef UTILITY_OUTPUT_STREAM_HPP
#define UTILITY_OUTPUT_STREAM_HPP

#include <string>
#include <map>

namespace control_engineering_uni_a
{
class OutputStream
{
public:

    /**
     * @brief Default constructor for the OutputStream class.
     */
    OutputStream() = default;

    /**
     * @brief Destructor for the OutputStream class.
     */
    virtual ~OutputStream() = default;

    /**
     * @brief Writes to the output stream.
     * @param t_component The component name or identifier that is writing to the output stream.
     * @param t_string The string to write to the output stream.
     * @param metadata Optional metadata that can be used for additional information.
     * @details This method is pure virtual, meaning that derived classes must implement it.
     */
    virtual void write(const std::string &t_component, const std::string &t_string, const std::map<std::string, std::string>& metadata = {}) = 0;
};

} // namespace control_engineering_uni_a

#endif // UTILITY_OUTPUT_STREAM_HPP