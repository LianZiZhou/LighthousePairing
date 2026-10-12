#pragma once
#include "core/HistoryDb.h"
#include "core/Store.h"

#include <memory>
#include <string>
#include <vector>

namespace lhp {

struct Command {
    enum class Type {
        Refresh,
        StartMonitor,
        StopMonitor,
        MonitorAll,
        StopAllMonitors,
        ClearStats,
        Pair,
        CancelOp,
        Unpair,
        FactoryReset,
        Identify,
        PowerOff,
        KillConflicts,
        SetNickname,
        SetNote,
        SetKind,
        SetDeviceNickname,
        SaveSettings,
        RawCommand,
        DismissOp,
        SetFastPoll,   // value = 1 on / 0 off (shake identify)
    } type;
    std::string serial;
    std::string text;
    int value = 0;
    double seconds = 0;
};

class IService {
public:
    explicit IService(Store& s) : store(s) {}
    virtual ~IService() = default;
    virtual void Start() = 0;
    virtual void Shutdown() = 0;
    virtual void Post(Command c) = 0;
    virtual std::vector<PairingRecord> PairingsFor(const std::string& serial) = 0;
    virtual std::string BackupDir() = 0;
    virtual std::string HistoryPath() = 0;

    Store& store;
};

std::unique_ptr<IService> CreateRealService(Store& store);
std::unique_ptr<IService> CreateDemoService(Store& store);

}  // namespace lhp
