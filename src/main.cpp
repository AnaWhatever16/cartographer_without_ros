#include <cartographer/mapping/map_builder.h>
#include <cartographer/mapping/map_builder_interface.h>
#include <cartographer/mapping/proto/map_builder_options.pb.h>
#include <cartographer/common/configuration_file_resolver.h>

#if defined(WIN32)
    #include <filesystem>
    namespace fs = std::filesystem;
#else
    #include <experimental/filesystem>  // Not implemented until g++8
    namespace fs = std::experimental::filesystem;
#endif

using namespace cartographer;

mapping::proto::MapBuilderOptions loadOptions(
    const std::string& configuration_directory,
    const std::string& configuration_basename) {

    auto file_resolver = absl::make_unique<common::ConfigurationFileResolver>(
        std::vector<std::string>{configuration_directory});

    const std::string code =
        file_resolver->GetFileContentOrDie(configuration_basename);

    common::LuaParameterDictionary lua_parameter_dictionary(
        code, std::move(file_resolver));

    return mapping::CreateMapBuilderOptions(&lua_parameter_dictionary);
}

// As input, give path to config file for map_builder

int main(int _argc, char** _argv){
    if(_argc != 2){
        std::cout << "Give the path where the map_builder config can be found, please :')" << std::endl;
        return 0;
    }

    fs::path p = _argv[1];
    std::string configDir = p.parent_path(); 
    std::string configBaseName = p.filename(); 

    auto mapBuilderOptions = loadOptions(configDir, configBaseName);
    auto mapBuilder = mapping::CreateMapBuilder(mapBuilderOptions);

    // mp.addTrajectoryBuilder(tb);

    // tb.addOdometry....

    return 1;
}