/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef HUMANOID_OPERATOR_CLIENT_UTILS_HPP
#define HUMANOID_OPERATOR_CLIENT_UTILS_HPP

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "operator_client.h"
#include "robot_base.h"

namespace humanoid_operator
{

inline std::string ResolvePath(const std::string & path)
{
  if (path.empty() || path.front() == '/') {
    return path;
  }
  const char * sdk_root = std::getenv("SDK_ROOT");
  if (sdk_root && sdk_root[0] != '\0') {
    return std::string(sdk_root) + "/" + path;
  }
  return std::filesystem::absolute(path).string();
}

inline std::string DefaultConnectionFile(const std::string & robot)
{
  const char * state_home = std::getenv("XDG_STATE_HOME");
  if (state_home && state_home[0] != '\0') {
    return std::string(state_home) + "/humanoid-operator/" + robot +
           "/connection.json";
  }
  const char * user_home = std::getenv("HOME");
  if (!user_home || user_home[0] == '\0') {
    throw std::runtime_error(
            "operator connection requires HOME or XDG_STATE_HOME");
  }
  return std::string(user_home) + "/.local/state/humanoid-operator/" +
         robot + "/connection.json";
}

inline std::string ConnectionFile(
  const std::string & robot_config_path,
  const std::string & override_path)
{
  if (!override_path.empty()) {
    return ResolvePath(override_path);
  }
  const auto yaml = robot_base::YamlFile::Load(robot_config_path);
  const auto configured =
    yaml.Read<std::string>("operator_service.connection_file");
  if (configured && !configured->empty()) {
    return *configured;
  }
  return DefaultConnectionFile(
    yaml.Read<std::string>("robot_base.name").value_or("robot"));
}

inline void Connect(
  operator_service::Client * client,
  const std::string & connection_file, const std::string & name,
  int timeout_ms)
{
  auto connection =
    operator_service::Client::ReadConnectionFile(connection_file);
  connection.name = name;
  connection.timeout_ms = timeout_ms;
  std::string error;
  if (!client->Connect(connection, &error)) {
    throw std::runtime_error("operator service connection failed: " + error);
  }
}

inline bool RequestPending(const operator_service::Status & status)
{
  return status.request.phase == "accepted";
}

}  // namespace humanoid_operator

#endif  // HUMANOID_OPERATOR_CLIENT_UTILS_HPP
