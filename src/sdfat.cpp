#include "sdfat.h"
#include "misc.h"

#include <cassert>
#include <cstddef>
#include <memory>
#include <unordered_map>

// TODO: Check that use of path class is correct

size_t FsFile::println(const String &str) {
  if (file_s == nullptr) {
    return 0;
  }

  // https://github.com/greiman/SdFat/blob/cda057318bec196183d4cc92b01bc1dd64bbfb02/extras/cppcheck/api/Print.cpp#L150-L153
  // This appears to be the ending used by the SdFat library printing
  file_s->content << str << "\r\n";

  return str.length() + 2;
}

bool FsFile::close() {
  file_s = nullptr;
  return true;
}

bool SdFs::begin(SdSpiConfig spiConfig) {
  if (began) { throw std::invalid_argument("Already began"); }
  if (spiConfig.csPin != 17) { throw std::invalid_argument("Unsupport csPin"); }
  if (spiConfig.options != DEDICATED_SPI) { throw std::invalid_argument("Unsupported options"); }
  if (spiConfig.maxSck != SD_SCK_MHZ(50)) { throw std::invalid_argument("Unsupported maxSck"); }

  began = true;
  return true;
}

bool SdFs::mkdir(const String &path, bool pFlag) {
  if (!began) { throw std::invalid_argument("Already began"); }
  if (path.length() != 0 && (path[path.length() - 1] == '/' || path[0] == '/')) { throw std::invalid_argument("Can't handle ending or leading /"); }

  if (exists(path)) {
    return false;
  }

  int index = path.find_last_of('/');
  if (index != -1) {
    String parent = path.substr(0, index);
    if (!dirs.contains(parent)) {
      if (!pFlag || !mkdir(parent, true)) {
        return false;
      }
    }
  }

  dirs.insert(path);

  return true;
}

bool SdFs::exists(const String &path) const {
  if (!began) { throw std::invalid_argument("Already began"); }

  return dirs.contains(path) || files_s.contains(path);
}

FsFile SdFs::open(const String &path, oflag_t oflag) {
  if (!began) { throw std::invalid_argument("Already began"); }

  // Currently files are only writable
  if (!(oflag & O_WRONLY)) {
    return FsFile();
  }

  if (dirs.contains(path)) {
    return FsFile();
  }

  auto index = files_s.find(path);
  if (index == files_s.end()) {
    if (oflag & O_CREAT) {
      std::shared_ptr<File_s> file = std::make_shared<File_s>();
      files_s.insert({path, file});
      return file;
    } else {
      return FsFile();
    }
  }

  return index->second;
}
