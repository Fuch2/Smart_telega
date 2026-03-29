#include "AdminViewModel.hpp"

#include "../../../repositories/IModuleRepository.hpp"
#include "../../../infrastructure/persistence/SqliteModuleRepository.hpp"
#include <QCoreApplication>

AdminViewModel::AdminViewModel(QObject* parent) : QObject(parent) {
    repo_ = std::make_unique<SqliteModuleRepository>("smart_cart.db");
}

int AdminViewModel::findIndexById(int id) const {
    for (int i = 0; i < items_.size(); ++i) {
        if (items_[i].id == id) return i;
    }
    return -1;
}

bool AdminViewModel::serialExists(const QString& serial, int exceptId) const {
    for (const auto& m : items_) {
        if (m.serial.compare(serial, Qt::CaseInsensitive) == 0 && m.id != exceptId) {
            return true;
        }
    }
    return false;
}

void AdminViewModel::loadDemo() {
    auto all = repo_->getAll();
    if (all.empty()) {
        repo_->add({0, "SN-DEMO-001", 24, "fw-1.0.0", "ONLINE"});
        repo_->add({0, "SN-DEMO-002", 24, "fw-1.0.1", "OFFLINE"});
        repo_->add({0, "SN-DEMO-003", 24, "fw-1.1.0", "MAINT"});
        all = repo_->getAll();
    }

    items_.clear();
    for (const auto& m : all) {
        items_.push_back({m.id, QString::fromStdString(m.serial), m.slotCount,
                          QString::fromStdString(m.firmware), QString::fromStdString(m.status)});
    }
    emit modulesReset();
}



void AdminViewModel::addModule(const QString& serial, int slotCount, const QString& firmware, const QString& status) {
    if (serial.trimmed().isEmpty()) {
        emit errorOccurred("Serial не может быть пустым");
        return;
    }
    if (serialExists(serial)) {
        emit errorOccurred("Serial уже существует");
        return;
    }
    items_.push_back({nextId_++, serial.trimmed(), slotCount, firmware.trimmed(), status.trimmed()});
    emit modulesReset();
    emit infoOccurred("Модуль добавлен");
}

void AdminViewModel::updateModule(int id, const QString& serial, int slotCount, const QString& firmware, const QString& status) {
    const int idx = findIndexById(id);
    if (idx < 0) {
        emit errorOccurred("Модуль не найден");
        return;
    }
    if (serial.trimmed().isEmpty()) {
        emit errorOccurred("Serial не может быть пустым");
        return;
    }
    if (serialExists(serial, id)) {
        emit errorOccurred("Serial уже существует");
        return;
    }

    auto& m = items_[idx];
    m.serial = serial.trimmed();
    m.slotCount = slotCount;
    m.firmware = firmware.trimmed();
    m.status = status.trimmed();

    emit modulesReset();
    emit infoOccurred("Модуль обновлён");
}

void AdminViewModel::removeModule(int id) {
    const int idx = findIndexById(id);
    if (idx < 0) {
        emit errorOccurred("Модуль не найден");
        return;
    }
    items_.removeAt(idx);
    emit modulesReset();
    emit infoOccurred("Модуль удалён");
}
