// Copyright 2026 OmniLink
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "OmAssimpIoSystem.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <sys/stat.h>
#endif

namespace {
#ifdef _WIN32
  // UTF-8 path -> absolute wide path carrying the \\?\ long-path prefix, which
  // lets the CRT open it past MAX_PATH whatever the executable's manifest says.
  std::wstring longWidePath(const char *utf8) {
    const int n = MultiByteToWideChar(CP_UTF8, 0, utf8, -1, nullptr, 0);
    if (n <= 0)
      return std::wstring();
    std::wstring wide(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8, -1, &wide[0], n);
    wide.resize(static_cast<size_t>(n - 1));
    if (wide.rfind(L"\\\\?\\", 0) == 0)
      return wide;
    // GetFullPathNameW also turns '/' into '\', which the prefixed form requires.
    const DWORD full = GetFullPathNameW(wide.c_str(), 0, nullptr, nullptr);
    if (full == 0)
      return wide;
    std::wstring absolute(full, L'\0');
    const DWORD written = GetFullPathNameW(wide.c_str(), full, &absolute[0], nullptr);
    if (written == 0 || written >= full)
      return wide;
    absolute.resize(written);
    if (absolute.rfind(L"\\\\", 0) == 0)  // UNC share: \\server\share -> \\?\UNC\server\share
      return L"\\\\?\\UNC\\" + absolute.substr(2);
    return L"\\\\?\\" + absolute;
  }
#endif

  FILE *openFile(const char *path, const char *mode) {
#ifdef _WIN32
    const std::wstring wide = longWidePath(path);
    std::wstring wideMode;
    for (const char *c = mode; *c != '\0'; ++c)
      wideMode.push_back(static_cast<wchar_t>(*c));
    return wide.empty() ? nullptr : _wfopen(wide.c_str(), wideMode.c_str());
#else
    return std::fopen(path, mode);
#endif
  }

  class OmFileIoStream : public Assimp::IOStream {
  public:
    explicit OmFileIoStream(FILE *file) : mFile(file) {}
    ~OmFileIoStream() override { std::fclose(mFile); }

    size_t Read(void *buffer, size_t size, size_t count) override {
      return (size == 0 || count == 0) ? 0 : std::fread(buffer, size, count, mFile);
    }
    size_t Write(const void *buffer, size_t size, size_t count) override {
      return (size == 0 || count == 0) ? 0 : std::fwrite(buffer, size, count, mFile);
    }
    aiReturn Seek(size_t offset, aiOrigin origin) override {
      // Assimp passes a negative offset (from the end) as a wrapped size_t, as it
      // does to fseek in its own DefaultIOStream; read it back as signed.
      const int64_t delta = static_cast<int64_t>(offset);
      const int whence = origin == aiOrigin_SET ? SEEK_SET : origin == aiOrigin_CUR ? SEEK_CUR : SEEK_END;
#ifdef _WIN32
      return _fseeki64(mFile, delta, whence) == 0 ? aiReturn_SUCCESS : aiReturn_FAILURE;
#else
      return fseeko(mFile, static_cast<off_t>(delta), whence) == 0 ? aiReturn_SUCCESS : aiReturn_FAILURE;
#endif
    }
    size_t Tell() const override {
#ifdef _WIN32
      return static_cast<size_t>(_ftelli64(mFile));
#else
      return static_cast<size_t>(ftello(mFile));
#endif
    }
    size_t FileSize() const override {
      const size_t here = Tell();
#ifdef _WIN32
      _fseeki64(mFile, 0, SEEK_END);
      const size_t size = static_cast<size_t>(_ftelli64(mFile));
      _fseeki64(mFile, static_cast<int64_t>(here), SEEK_SET);
#else
      fseeko(mFile, 0, SEEK_END);
      const size_t size = static_cast<size_t>(ftello(mFile));
      fseeko(mFile, static_cast<off_t>(here), SEEK_SET);
#endif
      return size;
    }
    void Flush() override { std::fflush(mFile); }

  private:
    FILE *mFile;
  };
}  // namespace

bool OmAssimpIoSystem::Exists(const char *file) const {
  if (file == nullptr)
    return false;
#ifdef _WIN32
  const std::wstring wide = longWidePath(file);
  if (wide.empty())
    return false;
  const DWORD attributes = GetFileAttributesW(wide.c_str());
  return attributes != INVALID_FILE_ATTRIBUTES && (attributes & FILE_ATTRIBUTE_DIRECTORY) == 0;
#else
  FILE *f = std::fopen(file, "rb");
  if (f == nullptr)
    return false;
  std::fclose(f);
  return true;
#endif
}

Assimp::IOStream *OmAssimpIoSystem::Open(const char *file, const char *mode) {
  if (file == nullptr)
    return nullptr;
  FILE *f = openFile(file, mode != nullptr ? mode : "rb");
  return f == nullptr ? nullptr : new OmFileIoStream(f);
}

void OmAssimpIoSystem::Close(Assimp::IOStream *stream) {
  delete stream;
}
