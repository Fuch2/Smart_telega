#pragma once
#include <QObject>
#include <QVector>
#include <QString>

#include "../../../repositories/IModuleRepository.hpp"
#include "../../../infrastructure/persistence/SqliteModuleRepository.hpp"

#include <memory>


struct ModuleItem {
    int id{0};
    QString serial;
    int slotCount{24};
    QString firmware;
    QString status;   // ONLINE / OFFLINE / MAINT
};

class AdminViewModel final : public QObject {
    Q_OBJECT
public:
    explicit AdminViewModel(QObject* parent = nullptr);

    const QVector<ModuleItem>& items() const { return items_; }

public slots:
    void loadDemo();
    void addModule(const QString& serial, int slotCount, const QString& firmware, const QString& status);
    void updateModule(int id, const QString& serial, int slotCount, const QString& firmware, const QString& status);

    void removeModule(int id);

signals:
    void modulesReset();
    void errorOccurred(const QString& message);
    void infoOccurred(const QString& message);

private:
    QVector<ModuleItem> items_;
    int nextId_{1};

    int findIndexById(int id) const;
    bool serialExists(const QString& serial, int exceptId = 0) const;

    std::unique_ptr<IModuleRepository> repo_;

};
