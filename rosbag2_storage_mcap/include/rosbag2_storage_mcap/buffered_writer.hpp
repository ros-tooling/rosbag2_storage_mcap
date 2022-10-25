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
#ifndef ROSBAG2_STORAGE_MCAP__BUFFERED_WRITER_HPP_
#define ROSBAG2_STORAGE_MCAP__BUFFERED_WRITER_HPP_

#include <cstddef>
#include <optional>
#include <vector>
#include <mcap/mcap.hpp>

namespace rosbag2_storage_mcap {

class BufferedWriter final : public mcap::IWritable {
public:
  ~BufferedWriter() override;

  /**
   * @brief opens a file for writing.
   * 
   * @param filename The path to the file to write.
   * @param bufferSize if std::nullopt, the buffer will grow until flush() is called.
   *   otherwise, the buffer will be written to the file if it exceeds this size.
   * @return true if the file opened successfully.
   * @return false if the file could not be opened for writing.
   */
  bool open(std::string_view filename, std::optional<size_t> bufferSize);
  /**
   * @brief write any buffered bytes to disk and empty the buffer. Does not imply fsync().
   */
  void flush();
  /**
   * @brief asks the OS to block until all bytes are actually written to the underlying storage
   * medium. Equivalent to `fsync()` on Unix-like systems, or `FlushFileBuffers()` on Windows.
   * 
   */
  bool syncToDisk();

  void handleWrite(const std::byte* data, uint64_t size) override;
  void end() override;
  uint64_t size() const override;

private:
  std::vector<std::byte> buffer_;
  std::optional<size_t> bufferCapacity_;
  std::FILE* file_ = nullptr;
  uint64_t size_ = 0;
};

}  // namespace rosbag2_storage_mcap

#endif  // ROSBAG2_STORAGE_MCAP__BUFFERED_WRITER_HPP_
