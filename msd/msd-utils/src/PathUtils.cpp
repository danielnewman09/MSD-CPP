#include "msd-utils/src/PathUtils.hpp"

#ifdef __EMSCRIPTEN__
// Emscripten uses a virtual filesystem - no platform-specific includes needed
#elif defined(_WIN32)
#include <windows.h>
#elif defined(__APPLE__)
#include <limits.h>
#include <mach-o/dyld.h>
#elif defined(__linux__)
#include <limits.h>
#include <unistd.h>
#endif

#include <stdexcept>

namespace msd_utils
{

std::filesystem::path absolutePath(const std::string& relativePath)
{
#ifdef __EMSCRIPTEN__
  // Emscripten uses a virtual filesystem with assets mounted at root
  // Paths like "example_assets.db" or "/assets/example_assets.db" work directly
  if (relativePath.empty())
  {
    return std::filesystem::path("/");
  }
  if (relativePath[0] == '/')
  {
    // Already an absolute path in the virtual filesystem
    return std::filesystem::path(relativePath);
  }
  // Treat relative paths as relative to root in the virtual filesystem
  return std::filesystem::path("/" + relativePath);

#else
  // Native platform implementations
  std::filesystem::path executablePath;

#ifdef _WIN32
  // Windows implementation
  char buffer[MAX_PATH];
  DWORD length = GetModuleFileNameA(nullptr, buffer, MAX_PATH);
  if (length == 0 || length == MAX_PATH)
  {
    throw std::runtime_error("Failed to get executable path on Windows");
  }
  executablePath = std::filesystem::path(buffer);

#elif defined(__APPLE__)
  // macOS implementation
  char buffer[PATH_MAX];
  uint32_t size = sizeof(buffer);
  if (_NSGetExecutablePath(buffer, &size) != 0)
  {
    throw std::runtime_error("Failed to get executable path on macOS");
  }
  // Resolve symlinks and relative paths
  char realBuffer[PATH_MAX];
  if (realpath(buffer, realBuffer) == nullptr)
  {
    throw std::runtime_error("Failed to resolve executable path on macOS");
  }
  executablePath = std::filesystem::path(realBuffer);

#elif defined(__linux__)
  // Linux implementation
  char buffer[PATH_MAX];
  ssize_t length = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (length == -1)
  {
    throw std::runtime_error("Failed to get executable path on Linux");
  }
  buffer[length] = '\0';
  executablePath = std::filesystem::path(buffer);

#else
#error "Unsupported platform for absolutePath"
#endif

  // Get the directory containing the executable
  std::filesystem::path executableDir = executablePath.parent_path();

  // Construct the full path and normalize it
  std::filesystem::path fullPath = executableDir / relativePath;

  // Return the canonical (absolute, normalized) path
  // Note: canonical() requires the path to exist; use absolute() if you want
  // to allow non-existent paths
  return std::filesystem::absolute(fullPath).lexically_normal();

#endif  // __EMSCRIPTEN__
}

}  // namespace msd_utils
