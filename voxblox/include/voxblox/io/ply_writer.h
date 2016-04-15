#ifndef VOXBLOX_CORE_IO_PLY_WRITER_H
#define VOXBLOX_CORE_IO_PLY_WRITER_H

#include "voxblox/core/common.h"

#include <fstream>
#include <glog/logging.h>

namespace voxblox {

namespace io {
// For reference on the format, see:
//  http://paulbourke.net/dataformats/ply/
class PlyWriter {
 public:
  PlyWriter(const std::string& filename)
      : header_written_(false),
        parameters_set_(false),
        vertices_total_(0),
        vertices_written_(0),
        has_color_(false),
        file_(filename) {}
  virtual ~PlyWriter() {}

  void addVerticesWithProperties(size_t num_vertices, bool has_color) {
    vertices_total_ = num_vertices;
    has_color_ = has_color;
    parameters_set_ = true;
  }

  bool writeHeader() {
    if (!file_) {
      // Output a warning -- couldn't open file?
      LOG(WARNING) << "Could not open file for PLY output.";
      return false;
    }
    if (!parameters_set_) {
      LOG(WARNING) << "Could not write header out -- parameters not set.";
      return false;
    }
    if (header_written_) {
      LOG(WARNING) << "Header already written.";
      return false;
    }

    file_ << "ply" << std::endl;
    file_ << "format ascii 1.0" << std::endl;
    file_ << "element vertex " << vertices_total_ << std::endl;
    file_ << "property float x" << std::endl;
    file_ << "property float y" << std::endl;
    file_ << "property float z" << std::endl;

    if (has_color_) {
      file_ << "property uchar red" << std::endl;
      file_ << "property uchar green" << std::endl;
      file_ << "property uchar blue" << std::endl;
    }

    file_ << "end_header" << std::endl;

    header_written_ = true;
    return true;
  }

  bool writeVertex(const Coordinates& coord) {
    if (!header_written_) {
      if (!writeHeader()) {
        return false;
      }
    }
    if (vertices_written_ >= vertices_total_ || has_color_) {
      return false;
    }
    file_ << coord.x() << " " << coord.y() << " " << coord.z() << std::endl;
    return true;
  }

  bool writeVertex(const Coordinates& coord, const uint8_t rgb[3]) {
    if (!header_written_) {
      if (!writeHeader()) {
        return false;
      }
    }
    if (vertices_written_ >= vertices_total_ || !has_color_) {
      return false;
    }
    file_ << coord.x() << " " << coord.y() << " " << coord.z() << " ";
    file_ << static_cast<int>(rgb[0]) << " " << static_cast<int>(rgb[1]) << " "
          << static_cast<int>(rgb[2]) << std::endl;
    return true;
  }

  void closeFile() { file_.close(); }

 private:
  bool header_written_;
  bool parameters_set_;

  size_t vertices_total_;
  size_t vertices_written_;
  bool has_color_;

  std::ofstream file_;
};

}  // namespace io

}  // namespace voxblox

#endif  // VOXBLOX_CORE_IO_PLY_WRITER_H
