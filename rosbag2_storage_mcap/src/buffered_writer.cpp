
// Copyright 2022, Foxglove Technologies. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "rosbag2_storage_mcap/buffered_writer.hpp"
#ifdef _WIN32
#include <fileapi.h>
#else
#include <unistd.h>
#include <stdio.h>
#endif

namespace rosbag2_storage_mcap {

BufferedWriter::~BufferedWriter() {
  end();
}

bool BufferedWriter::open(std::string_view filename, std::optional<size_t> bufferCapacity) {
  end();
  file_ = std::fopen(filename.data(), "wb");
  if (!file_) {
    // add ros log
    return false;
  }
  bufferCapacity_ = bufferCapacity;
  if (bufferCapacity) {
    buffer_.reserve(*bufferCapacity);
  }
  return true;
}

void BufferedWriter::handleWrite(const std::byte* data, uint64_t size) {
  assert(file_);

  if (bufferCapacity_ != std::nullopt) {
    // If this will overflow the buffer, flush it
    if (buffer_.size() > 0 && buffer_.size() + size > *bufferCapacity_) {
      flush();
    }
    // Append to the buffer if it will fit, otherwise directly write
    if (buffer_.size() + size <= bufferCapacity_) {
      buffer_.insert(buffer_.end(), data, data + size);
    } else {
      const size_t written = std::fwrite(data, 1, size, file_);
      (void)written;
      assert(written == size);
    }
  } else {
    buffer_.insert(buffer_.end(), data, data + size);
  }

  size_ += size;
}

void BufferedWriter::flush() {
  const size_t written = std::fwrite(buffer_.data(), 1, buffer_.size(), file_);
  (void)written;
  assert(written == buffer_.size());
  buffer_.clear();
}

bool BufferedWriter::syncToDisk() {
#ifdef _WIN32
  return FlushFileBuffers((HANDLE) _get_osfhandle(_fileno(file_))) != 0;
#else
  return fsync(fileno(file_)) == 0;
#endif
}

void BufferedWriter::end() {
  if (file_) {
    if (buffer_.size() > 0) {
      std::fwrite(buffer_.data(), 1, buffer_.size(), file_);
    }

    std::fclose(file_);
    file_ = nullptr;
  }
  buffer_.clear();
  size_ = 0;
}

uint64_t BufferedWriter::size() const {
  return size_;
}

}  // namespace rosbag2_storage_mcap
