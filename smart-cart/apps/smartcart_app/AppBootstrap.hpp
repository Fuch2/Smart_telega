#pragma once
#include <memory>
#include <string>

// Forward declarations — MainWindow не знает о деталях инфраструктуры
namespace smartcart::infrastructure::config { struct AppConfig; }
namespace smartcart::infrastructure::db     { class SqliteConnection; }
namespace smartcart::infrastructure::db     { class ModuleRepositorySqlite; }
namespace smartcart::infrastructure::db     { class ReelRepositorySqlite; }
namespace smartcart::infrastructure::db     { class OperationRepositorySqlite; }
class AdminViewModel;
class MainWindow;

namespace smartcart::application {

// AppBootstrap — единственное место, где создаются все зависимости.
// Владеет временем жизни инфраструктурных объектов.
// MainWindow получает уже готовые ViewModel-ы.
class AppBootstrap {
public:
    explicit AppBootstrap(const std::string& configPath);
    ~AppBootstrap();

    // Создаёт MainWindow с подключёнными зависимостями.
    // Вызывается один раз из main().
    // Возвращает голый указатель — владение передаётся Qt (parent = nullptr,
    // но QApplication::exec() держит event loop, окно само себя удалит).
    MainWindow* createMainWindow();

    // Путь к БД — для диагностики и тестов
    const std::string& dbPath() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace smartcart::application
