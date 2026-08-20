#pragma once

namespace scan_tracking::common {

/// Install process-local MiniDump writer under <exeDir>/crash_dumps.
/// Safe to call before QCoreApplication. No-op on non-Windows.
void installWinCrashDumpHandler();

}  // namespace scan_tracking::common
