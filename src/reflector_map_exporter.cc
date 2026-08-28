#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "cartographer/io/proto_stream_deserializer.h"

namespace {

std::string EscapeYamlString(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      escaped.push_back('\\');
    }
    escaped.push_back(character);
  }
  return escaped;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3 || argc > 5) {
    std::cerr << "Usage: " << argv[0]
              << " INPUT.pbstream OUTPUT.yaml [MAP_FRAME] [ID_PREFIX]\n";
    return EXIT_FAILURE;
  }

  const std::string input_filename = argv[1];
  const std::string output_filename = argv[2];
  const std::string map_frame = argc >= 4 ? argv[3] : "map";
  const std::string id_prefix = argc == 5 ? argv[4] : "reflector_";
  const auto pose_graph =
      cartographer::io::DeserializePoseGraphFromFile(input_filename);

  std::ofstream output(output_filename);
  if (!output.is_open()) {
    std::cerr << "Cannot open '" << output_filename << "' for writing.\n";
    return EXIT_FAILURE;
  }

  output << std::setprecision(17);
  output << "map_frame: \"" << EscapeYamlString(map_frame) << "\"\n";
  output << "reflectors:\n";
  int exported_count = 0;
  for (const auto& landmark : pose_graph.landmark_poses()) {
    if (!id_prefix.empty() &&
        landmark.landmark_id().compare(0, id_prefix.size(), id_prefix) != 0) {
      continue;
    }
    const auto& translation = landmark.global_pose().translation();
    output << "  - id: \"" << EscapeYamlString(landmark.landmark_id())
           << "\"\n"
           << "    x: " << translation.x() << "\n"
           << "    y: " << translation.y() << "\n"
           << "    z: " << translation.z() << "\n"
           << "    observations: 0\n";
    ++exported_count;
  }
  output.close();
  if (!output) {
    std::cerr << "Failed while writing '" << output_filename << "'.\n";
    return EXIT_FAILURE;
  }

  std::cout << "Exported " << exported_count
            << " reflectors to '" << output_filename << "'.\n";
  return EXIT_SUCCESS;
}
