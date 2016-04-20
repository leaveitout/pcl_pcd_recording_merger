#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/iterator_range.hpp>
#include <map>
#include <iomanip>


namespace fs = boost::filesystem;

constexpr int MAX_VALID_ARGS = 3;



auto printHelp (int argc, char **argv) -> void {
  using pcl::console::print_error;
  using pcl::console::print_info;

  print_error ("Syntax is: %s [-m] (<path-to-pcd-recordings>) | -h | --help)]\n", argv [0]);
  print_info ("%s -h | --help : shows this help\n", argv [0]);
  print_info ("%s -m : use moving least squares smoothing\n", argv [0]);
  print_info ("Path should contain individual folders for each camera and yml named xy.yml");
  print_info ("where x, y are the camera numbers (e.g. 12.yml).\n");
}

auto getTimedPcdFilesInPath(fs::path pcd_dir) -> std::multimap<double, fs::path> {
  auto result_set = std::multimap<double, fs::path>{};
  for (auto const & entry : boost::make_iterator_range(fs::directory_iterator{pcd_dir})) {
    if (fs::is_regular_file(entry.status())) {
      if (entry.path().extension() == ".pcd") {
        auto filename = entry.path().stem().string();
        auto T_pos = filename.find_last_of("T");
        if (T_pos != std::string::npos) {
          auto time_str = filename.substr(T_pos + 1);
          try {
            auto time = boost::lexical_cast<double>(time_str);
            result_set.emplace(time, entry);
          } catch (boost::bad_lexical_cast &e) {
            // Do nothing
          }
        }
      }
    }
  }
  return result_set;
}

auto main (int argc, char** argv) -> int {
  pcl::console::print_highlight (
      "PCL PCD cloud merging using saved extrinsics. "
                       "See %s -h for options.\n", argv[0]);

  auto help_flag_1 = pcl::console::find_switch (argc, argv, "-h");
  auto help_flag_2 = pcl::console::find_switch (argc, argv, "--help");

  if (help_flag_1 || help_flag_2 || argc > MAX_VALID_ARGS) {
    printHelp (argc, argv);
    return 0;
  }

  auto mls = pcl::console::find_switch(argc, argv, "-m");

  auto consumed_args = 1;
  if (mls) {
    pcl::console::print_info("Moving Least Squares smoothing active.\n");
    consumed_args++;
  }

  auto current_dir = fs::path {};
  if (consumed_args == argc)
    current_dir = fs::system_complete(argv[0]).parent_path();
  else {
    auto path_str = std::string{argv[consumed_args]};
    if(path_str[0] == '~' || path_str[0] == '/') {
      if(path_str[0] == '~') {
        path_str.replace(0, 1, getenv("HOME"));
      }
      current_dir = fs::path(path_str);
    }
    else {
      current_dir = fs::canonical(fs::path{path_str});
    }
  }

  if (!fs::exists(current_dir) || !fs::is_directory(current_dir) ) {
    pcl::console::print_error("A valid directory was not specified.\n");
    printHelp(argc, argv);
    return -1;
  }

  // Find the relevant yml files and test for the corresponding directories
  auto yml_files = std::vector<fs::path>{};
  for (auto const & entry : boost::make_iterator_range(fs::directory_iterator{current_dir})) {
    if (fs::is_regular_file(entry)) {
      auto file_type = entry.path().extension();
      if (file_type == ".yml") {
        auto file_name = entry.path().filename().string();
        // TODO: Hard coded - assumes num cameras < 10
        auto file_suffix = file_name.substr(0, 2);
        try {
          auto number = boost::lexical_cast<int>(file_suffix);
          yml_files.push_back(entry.path());
        } catch (boost::bad_lexical_cast &e) {
          // Do nothing
        }
      }
    }
  }

  std::sort(yml_files.begin(), yml_files.end());

  auto source_dirs = std::vector<fs::path>{};

  auto same_target = true;
  auto target_num = -1;

  for (const auto& d : yml_files) {
    if (target_num == -1) {
      target_num = std::stoi(d.filename().string().substr(1, 2));
    }
    else if (target_num != std::stoi(d.filename().string().substr(1, 2))) {
      pcl::console::print_error("Invalid calibration files detected.\n"
                                    "Make sure only relevant files are present.\n");
      printHelp(argc, argv);
      return -1;
    }
    source_dirs.push_back(current_dir / d.filename().string().substr(0,1));
  }

  auto target_dir = current_dir / std::to_string(target_num);

  auto valid_target_dir = fs::exists(target_dir) && fs::is_directory(target_dir);

  auto valid_all_source_dirs = true;

  for (const auto& d : source_dirs) {
    auto valid_source_dir = fs::exists(d) && fs::is_directory(d);
    valid_all_source_dirs = valid_all_source_dirs && valid_source_dir;
  }

  if (!valid_target_dir || !valid_all_source_dirs) {
    pcl::console::print_error("No valid directories located.\n"
                                  "Directories should be named 1/, 2/ etc.\n");
    printHelp(argc, argv);
    return -1;
  }

  auto ss = std::stringstream{};
  ss << "merged";
  if (mls)
    ss << "-mls";

  try {
    fs::create_directory(current_dir / ss.str());
  } catch(fs::filesystem_error e){
    pcl::console::print_error("Unable to create directories. "
                                  "Please ensure that the correct permissions are "
                                  "set for the target folder\n");
    printHelp(argc, argv);
    return -1;
  }

  auto target_set = getTimedPcdFilesInPath(target_dir);

  for (auto const & p : target_set)
    std::cout << p.first << " --- " << p.second << std::endl;

  return 0;
}