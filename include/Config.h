#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

class Config {
 private:
  static std::shared_ptr<Config> config_;
  YAML::Node node_;
  std::string filepath_;
  Config() {}

 public:
  ~Config() {
  }

  static void setParameterFile(const std::string &filename) {
    if (config_ == nullptr)
      config_ = std::shared_ptr<Config>(new Config);

    config_->node_ = YAML::LoadFile(filename);
    config_->filepath_ = filename;
    if (config_->node_.IsNull()) {
      std::cerr << "parameter file " << filename << " does not exist." << std::endl;
      return;
    }
  }

  template<typename T>
  static T get(const std::string &key) {
    auto result = config_->node_[key];
    if (result.IsDefined()) {
        auto value = result.as<T>();
        std::cout << key << " is set to: " << value << std::endl;
        return value;
    } else {
      return T();
    }
  }

  template<typename T>
  static T get(const std::string &key, const T defaultVal) {
    auto result = config_->node_[key];
    if (result.IsDefined()) {
        auto value = result.as<T>();
        std::cout << key << " is " << value << std::endl;
        return value;
    } else {
      return defaultVal;
    }
  }

  static std::string getConfigFilePath() {
    return Config::config_->filepath_;
  }

};

