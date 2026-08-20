#pragma once

#include <QtCore/QString>

namespace scan_tracking {
namespace vision {

QString buildCxpSmokeBmpPath(
    const QString& outputDir,
    const QString& roleName,
    const QString& timestamp);

}  // namespace vision
}  // namespace scan_tracking
