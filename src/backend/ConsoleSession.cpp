#include "backend/ConsoleSession.h"

#include "util/Text.h"

#include <windows.h>

#include <chrono>

namespace lhp {

namespace {

HANDLE GlobalJob() {
    static HANDLE job = [] {
        HANDLE j = CreateJobObjectW(nullptr, nullptr);
        if (j) {
            JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
            info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
            SetInformationJobObject(j, JobObjectExtendedLimitInformation, &info, sizeof info);
        }
        return j;
    }();
    return job;
}

std::wstring DirOf(const std::wstring& p) {
    auto pos = p.find_last_of(L"\\/");
    return pos == std::wstring::npos ? std::wstring() : p.substr(0, pos);
}

}  // namespace

ConsoleSession::ConsoleSession(std::wstring exePath, std::string serial, LineSink sink)
    : exe_(std::move(exePath)), serial_(std::move(serial)), sink_(std::move(sink)) {}

ConsoleSession::~ConsoleSession() { Stop(); }

bool ConsoleSession::Start(std::string* error, double timeoutS) {
    SECURITY_ATTRIBUTES sa{sizeof sa, nullptr, TRUE};
    HANDLE inRead = nullptr, inWrite = nullptr, outRead = nullptr, outWrite = nullptr;
    if (!CreatePipe(&inRead, &inWrite, &sa, 0) || !CreatePipe(&outRead, &outWrite, &sa, 0)) {
        if (error) *error = "CreatePipe failed";
        return false;
    }
    SetHandleInformation(inWrite, HANDLE_FLAG_INHERIT, 0);
    SetHandleInformation(outRead, HANDLE_FLAG_INHERIT, 0);

    // Restrict inheritance to exactly our two pipe ends (several sessions may spawn concurrently).
    SIZE_T attrSize = 0;
    InitializeProcThreadAttributeList(nullptr, 1, 0, &attrSize);
    std::vector<char> attrBuf(attrSize);
    auto attrs = reinterpret_cast<LPPROC_THREAD_ATTRIBUTE_LIST>(attrBuf.data());
    InitializeProcThreadAttributeList(attrs, 1, 0, &attrSize);
    HANDLE inherit[2] = {inRead, outWrite};
    UpdateProcThreadAttribute(attrs, 0, PROC_THREAD_ATTRIBUTE_HANDLE_LIST, inherit, sizeof inherit, nullptr, nullptr);

    STARTUPINFOEXW si{};
    si.StartupInfo.cb = sizeof si;
    si.StartupInfo.dwFlags = STARTF_USESTDHANDLES;
    si.StartupInfo.hStdInput = inRead;
    si.StartupInfo.hStdOutput = outWrite;
    si.StartupInfo.hStdError = outWrite;
    si.lpAttributeList = attrs;

    std::wstring cmd = L"\"" + exe_ + L"\"";
    if (!serial_.empty()) cmd += L" /serial " + ToWide(serial_);
    std::wstring dir = DirOf(exe_);

    PROCESS_INFORMATION pi{};
    BOOL ok = CreateProcessW(nullptr, cmd.data(), nullptr, nullptr, TRUE, CREATE_NO_WINDOW | EXTENDED_STARTUPINFO_PRESENT | CREATE_SUSPENDED,
                             nullptr, dir.empty() ? nullptr : dir.c_str(), &si.StartupInfo, &pi);
    DeleteProcThreadAttributeList(attrs);
    CloseHandle(inRead);
    CloseHandle(outWrite);
    if (!ok) {
        CloseHandle(inWrite);
        CloseHandle(outRead);
        if (error) *error = "无法启动 lighthouse_console.exe (错误 " + std::to_string(GetLastError()) + ")";
        return false;
    }
    if (HANDLE job = GlobalJob()) AssignProcessToJobObject(job, pi.hProcess);
    ResumeThread(pi.hThread);
    CloseHandle(pi.hThread);

    process_ = pi.hProcess;
    pid_ = pi.dwProcessId;
    stdinWrite_ = inWrite;
    stdoutRead_ = outRead;
    alive_ = true;
    reader_ = std::thread([this] { ReaderLoop(); });

    std::unique_lock lk(mtx_);
    bool got = cv_.wait_for(lk, std::chrono::duration<double>(timeoutS), [&] { return prompts_ > 0 || !alive_; });
    inStartup_ = false;
    if (!got || prompts_ == 0) {
        lk.unlock();
        if (error) *error = alive_ ? "等待 lighthouse_console 启动超时" : "lighthouse_console 意外退出";
        Stop();
        return false;
    }
    return true;
}

void ConsoleSession::ReaderLoop() {
    char buf[4096];
    for (;;) {
        DWORD n = 0;
        if (!ReadFile((HANDLE)stdoutRead_, buf, sizeof buf, &n, nullptr) || n == 0) break;
        Feed(buf, n);
    }
    {
        std::scoped_lock lk(mtx_);
        if (!pending_.empty()) {
            std::string rest = std::move(pending_);
            pending_.clear();
            EmitLine(std::move(rest));
        }
        alive_ = false;
    }
    cv_.notify_all();
}

void ConsoleSession::Feed(const char* data, size_t n) {
    std::vector<std::string> toSink;
    {
        std::scoped_lock lk(mtx_);
        pending_.append(data, n);
        bool promptSeen = false;
        for (;;) {
            if (StartsWith(pending_, "lh> ")) {
                pending_.erase(0, 4);
                ++prompts_;
                promptSeen = true;
                continue;
            }
            if (pending_.empty() || std::string_view("lh> ").substr(0, pending_.size()) == pending_) break;  // partial prompt
            auto nl = pending_.find('\n');
            if (nl == std::string::npos) break;
            std::string line = pending_.substr(0, nl);
            pending_.erase(0, nl + 1);
            while (!line.empty() && line.back() == '\r') line.pop_back();
            if (inStartup_) startup_.push_back(line);
            if (capturing_) capture_.push_back(line);
            if (!line.empty()) toSink.push_back(std::move(line));
        }
        if (promptSeen) cv_.notify_all();
    }
    if (sink_)
        for (auto& l : toSink) sink_(l);
}

void ConsoleSession::EmitLine(std::string line) {
    if (capturing_) capture_.push_back(line);
}

bool ConsoleSession::WriteRaw(std::string_view s) {
    if (!alive_ || !stdinWrite_) return false;
    DWORD w = 0;
    return WriteFile((HANDLE)stdinWrite_, s.data(), (DWORD)s.size(), &w, nullptr) && w == s.size();
}

bool ConsoleSession::Send(std::string_view command) {
    std::string c(command);
    c += "\n";
    return WriteRaw(c);
}

bool ConsoleSession::Execute(std::string_view command, std::vector<std::string>* out, double timeoutS, std::string* error) {
    std::scoped_lock ex(execMtx_);
    uint64_t p0;
    {
        std::scoped_lock lk(mtx_);
        if (!alive_) {
            if (error) *error = "会话已结束";
            return false;
        }
        p0 = prompts_;
        capture_.clear();
        capturing_ = true;
    }
    if (!Send(command)) {
        std::scoped_lock lk(mtx_);
        capturing_ = false;
        if (error) *error = "写入命令失败";
        return false;
    }
    std::unique_lock lk(mtx_);
    bool ok = cv_.wait_for(lk, std::chrono::duration<double>(timeoutS), [&] { return prompts_ > p0 || !alive_; });
    capturing_ = false;
    if (out) *out = std::move(capture_);
    capture_.clear();
    if (!ok || prompts_ <= p0) {
        if (error) *error = alive_ ? "命令超时: " + std::string(command) : "lighthouse_console 已退出";
        return false;
    }
    return true;
}

void ConsoleSession::Stop() {
    std::scoped_lock stopLock(stopMtx_);
    if (process_) {
        if (alive_) {
            WriteRaw("quit\n");
            if (WaitForSingleObject((HANDLE)process_, 1500) != WAIT_OBJECT_0) TerminateProcess((HANDLE)process_, 1);
        }
        WaitForSingleObject((HANDLE)process_, 2000);
    }
    if (stdinWrite_) {
        CloseHandle((HANDLE)stdinWrite_);
        stdinWrite_ = nullptr;
    }
    if (reader_.joinable()) reader_.join();
    if (stdoutRead_) {
        CloseHandle((HANDLE)stdoutRead_);
        stdoutRead_ = nullptr;
    }
    if (process_) {
        CloseHandle((HANDLE)process_);
        process_ = nullptr;
    }
    alive_ = false;
}

}  // namespace lhp
