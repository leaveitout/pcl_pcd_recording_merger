#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/range/iterator_range.hpp>
#include <map>
#include <iomanip>
#include <queue>
#include <mutex>
#include <Logger.hpp>
#include <thread>
#include <boost/smart_ptr/make_shared.hpp>

#include "CameraExtrinsicsIO.hpp"

namespace fs = boost::filesystem;

constexpr int MAX_VALID_ARGS = 3;

std::mutex queue_mutex;

auto printHelp (int argc, char ** argv) -> void {
  using pcl::console::print_error;
  using pcl::console::print_info;

  print_error ("Syntax is: %s [-m] (<path-to-pcd-recordings>) | -h | --help)]\n", argv [0]);
  print_info ("%s -h | --help : shows this help\n", argv [0]);
  print_info ("%s -m : use moving least squares smoothing\n", argv [0]);
  print_info ("Path should contain individual folders for each camera and yml named xy.yml");
  print_info ("where x, y are the camera numbers (e.g. 12.yml).\n");
}

auto getTimedPcdFilesInPath(fs::path const & pcd_dir)
    -> std::multimap<double, fs::path> {
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

auto loadExtrinsics(std::vector<fs::path> const & file_paths) -> std::vector<Eigen::Matrix4f> {
  auto result = std::vector<Eigen::Matrix4f>{};

  for (auto const & path : file_paths) {
    auto mat = Eigen::Matrix4f{};
    if (CameraExtrinsicsIO::loadExtrinsics(path.string(), mat))
      result.emplace_back(std::move(mat));
  }

  return result;
}

auto associateTimings(std::multimap<double, fs::path> const & target_set,
                      std::vector<std::multimap<double, fs::path>> const & source_sets)
    -> std::queue<std::vector<fs::path>> {

  auto result_associated_paths = std::queue<std::vector<fs::path>>{};

  for (auto const & target_path : target_set) {
    auto merge_paths = std::vector<fs::path>{};
    merge_paths.emplace_back(target_path.second);

    for (auto const & source_set : source_sets) {
      auto lower = source_set.lower_bound(target_path.first);

      // Edge cases
      if (lower == source_set.end()) {
        merge_paths.emplace_back(source_set.rbegin()->second);
      } else if (lower == source_set.begin()) {
        merge_paths.emplace_back(lower->second);
      } else {
        auto prev = lower;
        --prev;
        if (target_path.first - prev->first < lower->first - target_path.first)
          merge_paths.emplace_back(prev->second);
        else
          merge_paths.emplace_back(lower->second);
      }
    }
    result_associated_paths.emplace(merge_paths);
  }

  return result_associated_paths;
}

// Thread-safe queue access
auto getFront(std::queue<std::vector<fs::path>> & paths_set_queue)
    -> std::vector<fs::path> {
  auto next_paths_set = std::vector<fs::path>{};
  {
    std::lock_guard<std::mutex> lock{queue_mutex};
    if (!paths_set_queue.empty()) {
      next_paths_set = paths_set_queue.front();
      paths_set_queue.pop();
      if (paths_set_queue.size() % 10 == 0) {
        auto ss = std::stringstream{};
        ss << paths_set_queue.size() << " items remaining in queue." << std::endl;

      }
    }
  }
  return next_paths_set;
}

auto mergeFiles(std::vector<fs::path> const & paths,
                fs::path const & result_dir,
                std::vector<Eigen::Matrix4f> const RT_matrices,
                bool smoothing = false) {
  auto ss = std::stringstream{};

  auto clouds = std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>{};
  auto error = false;

  for (auto const & p : paths) {
    auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (p.string(), *cloud) == -1) {
      ss << "Failed to load: " << p.string() << std::endl;
      error = true;
    }
    else {
      ss << "Loaded: " << p.string() << std::endl;
      clouds.push_back(cloud);
    }
  }

  ss << "----------------------------------------------------------------------------"
      << std::endl;

  if (error)
    Logger::log(Logger::ERROR, ss.str());
  else {
    auto result_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();

    // Add target cloud
    *result_cloud += *(clouds.at(0));

    // Add source clouds
    auto index = 1ul;
    for (auto const &RT : RT_matrices) {
      auto target_cloud = clouds.at(index);
      pcl::transformPointCloud(*target_cloud, *target_cloud, RT);
      *result_cloud += *target_cloud;
      ++index;
    }

    if (smoothing) {
      // Perform mls smoothing
      auto tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGBA>>();
      auto mls = pcl::MovingLeastSquares<pcl::PointXYZRGBA, pcl::PointXYZRGBA>{};
      auto mls_points = pcl::PointCloud<pcl::PointXYZRGBA>{};

      mls.setComputeNormals(true);
      mls.setInputCloud(result_cloud);
      mls.setPolynomialFit(true);
      mls.setSearchMethod(tree);
      // TODO: Add parameter for this value (3cm)
      mls.setSearchRadius(0.03);
      mls.process(mls_points);
    }

    // Write to disk
    auto result_file = result_dir / paths.at(0).filename();

    pcl::io::savePCDFileBinaryCompressed(result_file.string(), *result_cloud);

    ss << "Merged file: " << result_file.string() << std::endl;

    Logger::log(Logger::INFO, ss.str());
  }

}

auto mergingRunner(std::queue<std::vector<fs::path>> &paths_set_queue,
                   fs::path const result_dir,
                   std::vector<Eigen::Matrix4f> const RT_matrices,
                   bool smoothing) {
  auto merge_paths = getFront(paths_set_queue);

  while(!merge_paths.empty()) {
    mergeFiles(merge_paths,
               result_dir,
               RT_matrices,
               smoothing);
    merge_paths = getFront(paths_set_queue);
  }
  auto ss = std::stringstream{};
  ss << "Thread exiting : " << std::this_thread::get_id() << std::endl;
  Logger::log(Logger::INFO, ss.str());
  return true;
}

auto main (int argc, char** argv) -> int {
  pcl::console::print_highlight (
      "PCL PCD cloud merging using saved extrinsics. "
                       "See %s -h for options.\n", argv[0]);

  auto help_flag_1 = pcl::console::find_switch(argc, argv, "-h");
  auto help_flag_2 = pcl::console::find_switch(argc, argv, "--help");

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

  if (!fs::exists(current_dir) || !fs::is_directory(current_dir)) {
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

  auto RT_mats = loadExtrinsics(yml_files);
  if (RT_mats.empty()) {
    pcl::console::print_error("Failed to correctly load yml files.\n"
                                  "Please ensure that the file contents are correct.\n");
    printHelp(argc, argv);
    return -1;
  }

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

  auto result_dir = current_dir / ss.str();
  try {
    fs::create_directory(result_dir);
  } catch(fs::filesystem_error e){
    pcl::console::print_error("Unable to create directories. "
                                  "Please ensure that the correct permissions are "
                                  "set for the target folder\n");
    printHelp(argc, argv);
    return -1;
  }

  auto target_set = getTimedPcdFilesInPath(target_dir);

  auto source_sets = std::vector<std::multimap<double, fs::path>>{};

  for (auto const & dir : source_dirs) {
    source_sets.emplace_back(getTimedPcdFilesInPath(dir));
  }

  auto paths_to_merge = associateTimings(target_set, source_sets);

  auto runners = std::vector<std::shared_ptr<std::thread>>{};
  auto num_threads_to_use = std::thread::hardware_concurrency();

  for (auto i = 0u; i < num_threads_to_use; ++i) {
    auto runner = std::make_shared<std::thread>(mergingRunner,
                                                std::ref(paths_to_merge),
                                                result_dir,
                                                RT_mats,
                                                mls);
    runners.push_back(runner);
  }

  for (auto i = 0u; i < runners.size(); ++i)
    if(runners.at(i)->joinable())
      runners.at(i)->join();

  return 0;
}