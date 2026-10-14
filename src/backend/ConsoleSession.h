#pragma once
#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace lhp {

/// Owns one lighthouse_console.exe child process and talks to its "lh> " prompt
/// over anonymous pipes. Output is unbuffered (verified), so commands complete
/// when the next prompt is printed.
class ConsoleSession {
public:
    using LineSink = std::function<void(const std::string& line)>;

    ConsoleSession(std::wstring exePath, std::string serial, LineSink sink);
    ~ConsoleSession();
    ConsoleSession(const ConsoleSession&) = delete;
    ConsoleSession& operator=(const ConsoleSession&) = delete;

    /// Extra command-line flags appended after "/serial X" (e.g. L" /pairstop"). Call before Start().
    void SetExtraArgs(std::wstring args) { extraArgs_ = std::move(args); }

    /// Spawn the process and wait for the first prompt.
    bool Start(std::string* error, double timeoutS = 10.0);

    /// Send a command and wait for the next prompt. Captured output (prompt
    /// stripped) is returned in `out`.
    bool Execute(std::string_view command, std::vector<std::string>* out, double timeoutS, std::string* error);

    /// Fire-and-forget command (does not wait).
    bool Send(std::string_view command);

    void Stop();
    bool Alive() const { return alive_.load(); }
    const std::vector<std::string>& StartupLines() const { return startup_; }
    const std::string& Serial() const { return serial_; }
    uint32_t Pid() const { return pid_; }

private:
    void ReaderLoop();
    void Feed(const char* data, size_t n);
    void EmitLine(std::string line);
    bool WriteRaw(std::string_view s);

    std::wstring exe_;
    std::string serial_;
    std::wstring extraArgs_;
    LineSink sink_;

    void* process_ = nullptr;
    void* stdinWrite_ = nullptr;
    void* stdoutRead_ = nullptr;
    uint32_t pid_ = 0;
    std::thread reader_;
    std::atomic<bool> alive_{false};

    std::mutex mtx_;
    std::condition_variable cv_;
    uint64_t prompts_ = 0;
    bool capturing_ = false;
    std::vector<std::string> capture_;
    std::string pending_;
    std::vector<std::string> startup_;
    bool inStartup_ = true;

    std::mutex execMtx_;
    std::mutex stopMtx_;
};

}  // namespace lhp
