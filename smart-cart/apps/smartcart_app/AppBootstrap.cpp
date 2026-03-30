#include "AppBootstrap.hpp"

#include "../../src/infrastructure/config/ConfigLoader.hpp"
#include "../../src/infrastructure/db/SqliteConnection.hpp"
#include "../../src/infrastructure/db/repositories/ModuleRepositorySqlite.hpp"
#include "../../src/infrastructure/db/repositories/ReelRepositorySqlite.hpp"
#include "../../src/infrastructure/db/repositories/OperationRepositorySqlite.hpp"

#include "../../src/presentation/qt/MainWindow.hpp"
#include "../../src/presentation/qt/viewmodels/AdminViewModel.hpp"

#include <stdexcept>


namespace smartcart::application {

// ── Pimpl: владеет всеми инфраструктурными объектами ─────────────────────────
struct AppBootstrap::Impl {
    infrastructure::config::AppConfig cfg;

    // Порядок важен: conn_ должен жить дольше репозиториев
    std::unique_ptr<infrastructure::db::SqliteConnection>           conn;
    std::unique_ptr<infrastructure::db::ModuleRepositorySqlite>     moduleRepo;
    std::unique_ptr<infrastructure::db::ReelRepositorySqlite>       reelRepo;
    std::unique_ptr<infrastructure::db::OperationRepositorySqlite>  opRepo;

    // ViewModel-ы живут здесь, но Qt-parent выставляется при передаче в View
    std::unique_ptr<AdminViewModel> adminVm;
};

// ── Ctor / Dtor ───────────────────────────────────────────────────────────────

AppBootstrap::AppBootstrap(const std::string& configPath)
    : impl_(std::make_unique<Impl>()) {

    // 1. Конфиг
    impl_->cfg = infrastructure::config::ConfigLoader::loadFromFile(configPath);

    // 2. БД + миграции
    impl_->conn = std::make_unique<infrastructure::db::SqliteConnection>(
        impl_->cfg.sqlitePath);
    impl_->conn->runMigrations(MIGRATIONS_DIR);

    // 3. Репозитории — получают SqliteConnection по ссылке
    impl_->moduleRepo = std::make_unique<infrastructure::db::ModuleRepositorySqlite>(
        *impl_->conn);
    impl_->reelRepo   = std::make_unique<infrastructure::db::ReelRepositorySqlite>(
        *impl_->conn);
    impl_->opRepo     = std::make_unique<infrastructure::db::OperationRepositorySqlite>(
        *impl_->conn);

    // 4. ViewModel-ы — получают репозитории по ссылке
    //    parent = nullptr: Qt-parent выставит MainWindow при addWidget()
    impl_->adminVm = std::make_unique<AdminViewModel>(
        *impl_->moduleRepo, nullptr);
}

AppBootstrap::~AppBootstrap() = default;

// ── createMainWindow ──────────────────────────────────────────────────────────

MainWindow* AppBootstrap::createMainWindow() {
    // MainWindow получает готовые ViewModel-ы — не знает о БД и конфиге
    auto* window = new MainWindow(
        impl_->adminVm.get()
        // сюда добавим workerVm в Приоритете 3
    );
    return window;
}

const std::string& AppBootstrap::dbPath() const {
    return impl_->cfg.sqlitePath;
}

} // namespace smartcart::application
