#include "AdminViewModel.hpp"
#include "../../../infrastructure/persistence/SqliteModuleRepository.hpp"

AdminViewModel::AdminViewModel(QObject* parent) : QObject(parent) {
    repo_ = std::make_unique<SqliteModuleRepository>(DB_PATH);
}

int AdminViewModel::findIndexById(int id) const {
    for (int i = 0; i < items_.size(); ++i)
        if (items_[i].id == id) return i;
    return -1;
}

bool AdminViewModel::serialExists(const QString& serial, int exceptId) const {
    return repo_->existsBySerial(serial.toStdString(), exceptId);
}

static ModuleItem toItem(const ModuleEntity& m) {
    return {m.id,
            QString::fromStdString(m.serial),
            m.slotCount,
            QString::fromStdString(m.firmware),
            QString::fromStdString(m.status)};
}

void AdminViewModel::reload() {
    items_.clear();
    for (const auto& m : repo_->getAll())
        items_.push_back(toItem(m));
    emit modulesReset();
}

void AdminViewModel::loadDemo() {
    if (repo_->getAll().empty()) {
        repo_->add({0, "SN-DEMO-001", 24, "fw-1.0.0", "ONLINE"});
        repo_->add({0, "SN-DEMO-002", 24, "fw-1.0.1", "OFFLINE"});
        repo_->add({0, "SN-DEMO-003", 24, "fw-1.1.0", "MAINT"});
    }
    reload();
}

void AdminViewModel::addModule(const QString& serial, int slotCount,
                                const QString& firmware, const QString& status) {
    if (serial.trimmed().isEmpty()) {
        emit errorOccurred("Serial не может быть пустым");
        return;
    }
    if (repo_->existsBySerial(serial.trimmed().toStdString())) {
        emit errorOccurred("Serial уже существует");
        return;
    }

    ModuleEntity rec{0,
                     serial.trimmed().toStdString(),
                     slotCount,
                     firmware.trimmed().toStdString(),
                     status.trimmed().toStdString()};
    repo_->add(rec);
    reload();
    emit infoOccurred("Модуль добавлен");
}

void AdminViewModel::updateModule(int id, const QString& serial, int slotCount,
                                   const QString& firmware, const QString& status) {
    if (findIndexById(id) < 0) {
        emit errorOccurred("Модуль не найден");
        return;
    }
    if (serial.trimmed().isEmpty()) {
        emit errorOccurred("Serial не может быть пустым");
        return;
    }
    if (repo_->existsBySerial(serial.trimmed().toStdString(), id)) {
        emit errorOccurred("Serial уже существует");
        return;
    }

    ModuleEntity rec{id,
                     serial.trimmed().toStdString(),
                     slotCount,
                     firmware.trimmed().toStdString(),
                     status.trimmed().toStdString()};
    repo_->update(rec);
    reload();
    emit infoOccurred("Модуль обновлён");
}

void AdminViewModel::removeModule(int id) {
    if (findIndexById(id) < 0) {
        emit errorOccurred("Модуль не найден");
        return;
    }
    repo_->remove(id);
    reload();
    emit infoOccurred("Модуль удалён");
}
