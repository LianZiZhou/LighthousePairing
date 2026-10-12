#pragma once
#include "core/Model.h"

#include <functional>
#include <mutex>

namespace lhp {

/// Single source of truth. Backend threads mutate under the lock; the UI thread
/// holds the lock while drawing a frame (short, bounded).
class Store {
public:
    std::mutex mutex;
    AppState state;

    template <typename F>
    auto With(F&& f) {
        std::scoped_lock lk(mutex);
        return f(state);
    }

    // Helpers – all lock internally.
    void Log(LogLevel level, std::string source, std::string text);
    void AddToast(Toast::Kind kind, std::string title, std::string text, float duration = 5.0f);

    // Unlocked variants (caller already holds the lock).
    static void LogUnlocked(AppState& s, LogLevel level, std::string source, std::string text);
    static void ToastUnlocked(AppState& s, Toast::Kind kind, std::string title, std::string text, float duration = 5.0f);
    static void PushSample(Receiver& r, const SignalSample& sample);
    static void PushEvent(Receiver& r, SignalEvent ev);
    static void SortReceivers(AppState& s);
};

}  // namespace lhp
