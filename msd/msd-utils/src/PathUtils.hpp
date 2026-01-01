#ifndef MSD_UTILS_PATH_UTILS_HPP
#define MSD_UTILS_PATH_UTILS_HPP

#include <filesystem>
#include <string>

namespace msd_utils
{

/**
 * Convert a string path to an absolute filesystem path relative to the
 * directory containing the current executable.
 *
 * @param relativePath The path string relative to the executable directory
 * @return An absolute filesystem path
 *
 * Example:
 *   If executable is at: /home/user/app/bin/myapp
 *   And relativePath is: "../data/config.json"
 *   Returns: /home/user/app/data/config.json
 */
std::filesystem::path absolutePath(const std::string& relativePath);

}  // namespace msd_utils

#endif  // MSD_UTILS_PATH_UTILS_HPP
