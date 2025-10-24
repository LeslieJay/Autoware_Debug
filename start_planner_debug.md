<!--
 * @Autor: wei.canming
 * @Version: 1.0
 * @Date: 2025-10-24 16:10:24
 * @LastEditors: wei.canming
 * @LastEditTime: 2025-10-24 16:24:49
 * @Description: 
-->
# start plannner 核心函数

StartPlannerModule::run() ————> StartPlanner::Plan()

### "Not found safe pull out path, publish stop path"

if (!status_.found_pull_out_path) {
  RCLCPP_WARN_THROTTLE(
    getLogger(), *clock_, 5000, "Not found safe pull out path, publish stop path");
  const auto output = generateStopOutput();
  setDebugData();  // use status updated in generateStopOutput()
  updateRTCStatus(0, 0);
  return output;
}

### status_

status_会影响最后选择的路径