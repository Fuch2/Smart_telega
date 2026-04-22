#pragma once
#include <QObject>
#include <QVector>
#include <QString>
#include <memory>

#include "../../../application/ports/IModuleRepository.hpp"
#include "../../../application/services/AdminDiagnosticsService.hpp"

namespace smartcart::domain { struct ModuleInfo; }

// ── Плоская Qt-модель для таблицы ──────────────────────────────────────────
// Отражает domain::ModuleInfo, но с Qt-типами для удобства биндинга
struct ModuleItem {
    int     id{0};
    QString serial;
    int     slotCount{24};
    QString firmware;
    QString status;   // "ONLINE" / "OFFLINE" / "MAINT"
    QString kind;     // "REEL" / "FEEDER" / "UNKNOWN"
};

class AdminViewModel final : public QObject {
    Q_OBJECT
public:
    // Зависимость приходит снаружи — ViewModel не знает о SQLite
    explicit AdminViewModel(
        smartcart::application::ports::IModuleRepository& repo,
        smartcart::application::services::AdminDiagnosticsService& diagnosticsSvc,
        QObject* parent = nullptr);

    const QVector<ModuleItem>& items() const { return items_; }

public slots:
    void load();
    void addModule(const QString& serial, int slotCount,
                   const QString& firmware, const QString& status);
    void updateModule(int id, const QString& serial, int slotCount,
                      const QString& firmware, const QString& status);
    void removeModule(int id);
    void refreshDiagnostics();
    void resetTestCart();

signals:
    void modulesReset();
    void diagnosticsUpdated(const QString& text);
    void errorOccurred(const QString& message);
    void infoOccurred(const QString& message);

private:
    void reload();
    int  findIndexById(int id) const;

    smartcart::application::ports::IModuleRepository& repo_;
    smartcart::application::services::AdminDiagnosticsService& diagnosticsSvc_;
    QVector<ModuleItem> items_;
};
