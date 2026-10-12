#pragma once
#include "core/Model.h"

#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace lhp {

struct PairingRecord {
    std::string device, model, source;
    int64_t firstSeen = 0, lastSeen = 0;
};

/// Persistent knowledge: nicknames, notes, kind overrides and the history of which
/// device was seen on which receiver. Stored as JSON in the data directory.
class HistoryDb {
public:
    explicit HistoryDb(std::string dataDir);

    void Load();
    bool Save();

    void ApplyTo(Receiver& r);                 // fills nickname / note / pairing fallback / kind override
    void Observe(const Receiver& r);           // record live observations
    void RecordPairing(const std::string& receiver, const std::string& device, const std::string& model, const std::string& source);
    void ForgetPairing(const std::string& receiver);
    /// Remove every pairing record that mentions `device` (used for wired devices that were
    /// mis-recorded as paired, e.g. an HMD's lighthouse module).
    void ForgetDevice(const std::string& device);

    void SetNickname(const std::string& serial, const std::string& name);
    void SetNote(const std::string& serial, const std::string& note);
    void SetKindOverride(const std::string& serial, int kind);   // -1 = automatic
    std::string DeviceNickname(const std::string& device) const;
    void SetDeviceNickname(const std::string& device, const std::string& name);

    std::vector<PairingRecord> PairingsFor(const std::string& receiver) const;

    Settings LoadSettings() const;
    void SaveSettings(const Settings& s);

    /// Writes a snapshot of the current state to backups/<timestamp>-<op>.json and returns its path.
    std::string WriteBackup(const AppState& s, const std::string& op, const std::string& target);
    std::string Path() const { return path_; }
    std::string BackupDir() const { return dir_ + "\\backups"; }

private:
    nlohmann::json& Rec(const std::string& serial);
    std::string dir_, path_;
    mutable std::mutex mtx_;
    nlohmann::json j_;
    bool dirty_ = false;
};

nlohmann::json SnapshotJson(const AppState& s);

}  // namespace lhp
