// ===== apps/smartcart_app/main_qt.cpp =====
// Entry point Qt-приложения SmartCart.
// Лежит рядом с AppBootstrap — include прямой, без path-магии.
#include "AppBootstrap.hpp"  // ← теперь корректно: оба файла в apps/smartcart_app/

#include <QApplication>
#include <QMessageBox>
#include <filesystem>
#include <stdexcept>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("SmartCart");
    app.setApplicationVersion("1.0.0");
    app.setOrganizationName("SmartCart");

    // Пути относительно директории бинарника.
    // POST_BUILD копирует config/ и migrations/ рядом с исполняемым файлом.
    const std::filesystem::path binDir =
        std::filesystem::canonical(argv[0]).parent_path();

    const std::filesystem::path configPath    = binDir / "config" / "config.json";
    const std::filesystem::path migrationsDir = binDir / "migrations";

    try {
        AppBootstrap bootstrap(configPath, migrationsDir);
        bootstrap.launch();
        return app.exec();
    } catch (const std::exception& ex) {
        QMessageBox::critical(
            nullptr,
            QString::fromUtf8("Критическая ошибка"),
            QString::fromUtf8("Не удалось запустить SmartCart:\n%1")
                .arg(QString::fromUtf8(ex.what()))
        );
        return 1;
    }
}
