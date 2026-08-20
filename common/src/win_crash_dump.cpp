#include "scan_tracking/common/win_crash_dump.h"

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <Windows.h>
#include <DbgHelp.h>

#include <atomic>
#include <cstdio>
#include <exception>

#pragma comment(lib, "Dbghelp.lib")

namespace scan_tracking::common {
namespace {

std::atomic_bool g_writing{false};
std::atomic_bool g_installed{false};
wchar_t g_dumpDir[MAX_PATH] = {0};

bool EnsureDumpDirectory()
{
    if (g_dumpDir[0] == L'\0') {
        wchar_t modulePath[MAX_PATH] = {0};
        const DWORD len = GetModuleFileNameW(nullptr, modulePath, MAX_PATH);
        if (len == 0 || len >= MAX_PATH) {
            return false;
        }
        wchar_t* slash = wcsrchr(modulePath, L'\\');
        if (slash == nullptr) {
            return false;
        }
        *(slash + 1) = L'\0';
        if (wcslen(modulePath) + wcslen(L"crash_dumps") + 1 >= MAX_PATH) {
            return false;
        }
        wcscpy_s(g_dumpDir, modulePath);
        wcscat_s(g_dumpDir, L"crash_dumps");
    }
    if (CreateDirectoryW(g_dumpDir, nullptr) == FALSE) {
        const DWORD err = GetLastError();
        if (err != ERROR_ALREADY_EXISTS) {
            return false;
        }
    }
    return true;
}

bool BuildDumpPath(wchar_t* outPath, size_t outChars)
{
    if (!EnsureDumpDirectory() || outPath == nullptr || outChars < 16) {
        return false;
    }

    SYSTEMTIME st{};
    GetLocalTime(&st);
    const DWORD pid = GetCurrentProcessId();
    const int written = _snwprintf_s(
        outPath,
        outChars,
        _TRUNCATE,
        L"%s\\scan-tracking_%04u%02u%02u_%02u%02u%02u_%lu.dmp",
        g_dumpDir,
        static_cast<unsigned>(st.wYear),
        static_cast<unsigned>(st.wMonth),
        static_cast<unsigned>(st.wDay),
        static_cast<unsigned>(st.wHour),
        static_cast<unsigned>(st.wMinute),
        static_cast<unsigned>(st.wSecond),
        static_cast<unsigned long>(pid));
    return written > 0;
}

bool WriteMiniDump(EXCEPTION_POINTERS* exceptionPointers)
{
    if (g_writing.exchange(true)) {
        return false;
    }

    wchar_t dumpPath[MAX_PATH] = {0};
    if (!BuildDumpPath(dumpPath, MAX_PATH)) {
        g_writing.store(false);
        return false;
    }

    const HANDLE file = CreateFileW(
        dumpPath,
        GENERIC_WRITE,
        FILE_SHARE_READ,
        nullptr,
        CREATE_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (file == INVALID_HANDLE_VALUE) {
        g_writing.store(false);
        return false;
    }

    MINIDUMP_EXCEPTION_INFORMATION exceptionInfo{};
    MINIDUMP_EXCEPTION_INFORMATION* exceptionInfoPtr = nullptr;
    if (exceptionPointers != nullptr) {
        exceptionInfo.ThreadId = GetCurrentThreadId();
        exceptionInfo.ExceptionPointers = exceptionPointers;
        exceptionInfo.ClientPointers = FALSE;
        exceptionInfoPtr = &exceptionInfo;
    }

    const MINIDUMP_TYPE dumpType = static_cast<MINIDUMP_TYPE>(
        MiniDumpWithDataSegs | MiniDumpWithHandleData | MiniDumpWithThreadInfo |
        MiniDumpWithUnloadedModules | MiniDumpWithIndirectlyReferencedMemory |
        MiniDumpWithProcessThreadData);

    const BOOL ok = MiniDumpWriteDump(
        GetCurrentProcess(),
        GetCurrentProcessId(),
        file,
        dumpType,
        exceptionInfoPtr,
        nullptr,
        nullptr);
    CloseHandle(file);

    if (ok) {
        OutputDebugStringW(L"[scan-tracking] MiniDump written: ");
        OutputDebugStringW(dumpPath);
        OutputDebugStringW(L"\n");
        // Best-effort console hint when running interactively.
        fwprintf(stderr, L"\n[scan-tracking] crash dump: %s\n", dumpPath);
        fflush(stderr);
    }

    g_writing.store(false);
    return ok == TRUE;
}

LONG WINAPI UnhandledExceptionFilterImpl(EXCEPTION_POINTERS* info)
{
    WriteMiniDump(info);
    // Let Windows Error Reporting / LocalDumps also attempt a dump.
    return EXCEPTION_CONTINUE_SEARCH;
}

void TerminateHandler()
{
    WriteMiniDump(nullptr);
    std::abort();
}

}  // namespace

void installWinCrashDumpHandler()
{
    if (g_installed.exchange(true)) {
        return;
    }
    EnsureDumpDirectory();
    SetUnhandledExceptionFilter(UnhandledExceptionFilterImpl);
    std::set_terminate(TerminateHandler);
}

}  // namespace scan_tracking::common

#else  // !_WIN32

namespace scan_tracking::common {

void installWinCrashDumpHandler() {}

}  // namespace scan_tracking::common

#endif
