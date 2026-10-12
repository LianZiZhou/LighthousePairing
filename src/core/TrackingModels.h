#pragma once
// Adapters between the app and the tracking subsystem:
//  - StoreModelProvider: JSON config of the device behind a receiver (device serial from lighthouse_console,
//    config from DataDir\device_configs or SteamVR's cache).
//  - FileCalibrationCache: base station OOTX calibrations and the learned universe (base poses, tilt) in
//    DataDir\base_stations.json; also offers SteamVR's lighthousedb.json universes.
#include "core/Store.h"
#include "tracking/model/DeviceModel.h"
#include "tracking/service/DevicePipeline.h"

#include <map>
#include <mutex>

namespace lhp {

class StoreModelProvider final : public tracking::model::ModelProvider {
public:
    explicit StoreModelProvider(Store& store) : store_(store) {}
    /// Lookup order: config downloaded by this app (DataDir\device_configs), then SteamVR's cache.
    std::optional<std::string> ConfigFor(const std::string& receiverSerial) override;

private:
    Store& store_;
};

class FileCalibrationCache final : public tracking::service::CalibrationCache {
public:
    explicit FileCalibrationCache(std::string path);
    std::optional<tracking::lighthouse::BaseCalibration> Load(int channel) override;
    void Save(int channel, const tracking::lighthouse::BaseCalibration& cal) override;
    std::vector<tracking::solver::UniverseRecord> LoadUniverses() override;
    void SaveUniverse(const tracking::solver::UniverseRecord& u) override;

private:
    void WriteLocked();
    std::string path_;
    std::string universe_;   // our saved universe (JSON object text)
    std::mutex mtx_;
    std::map<int, std::vector<uint8_t>> raw_;   // channel -> OOTX payload
};

}  // namespace lhp
