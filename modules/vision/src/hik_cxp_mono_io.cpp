#include "scan_tracking/vision/hik_cxp_mono_io.h"

#include <QtCore/QDir>

namespace scan_tracking {
namespace vision {

QString buildCxpSmokeBmpPath(
    const QString& outputDir,
    const QString& roleName,
    const QString& timestamp)
{
    const QString safeRole = roleName.trimmed().isEmpty() ? QStringLiteral("cxp") : roleName.trimmed();
    const QString fileName = QStringLiteral("%1_%2.bmp").arg(safeRole, timestamp);
    return QDir(outputDir).absoluteFilePath(fileName);
}

}  // namespace vision
}  // namespace scan_tracking
