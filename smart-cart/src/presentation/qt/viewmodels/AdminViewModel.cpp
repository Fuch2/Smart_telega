#include "AdminViewModel.hpp"
#include "../../../domain/entities/ModuleInfo.hpp"

using namespace smartcart::domain;
using namespace smartcart::application::ports;

// ── Конвертация domain → Qt ──────────────────────────────────────────────────
static ModuleItem toItem(const ModuleInfo& m) {
    return {
        m.id,
        QString::fromStdString(m.serial),
        m.slotCount,
        QString::fromStdString(m.firmware),
        QString::fromStdString(std::string{toString(m.status)})
    };
}

// ── Конвертация Qt → domain ──────────────────────────────────────────────────
static ModuleInfo toDomain(int id, const QString& serial, int slotCount,
                            const QString& firmware, const QString& status) {
    ModuleInfo m;
    m.id        = id;
    m.serial    = serial.trimmed().toStdString();
    m.slotCount = slotCount;
    m.firmware  = firmware.trimmed().toStdString();
    m.status    = moduleStatusFromString(status.toStdString());
    return m;
}

// ── Ctor ─────────────────────────────────────────────────────────────────────
AdminViewModel::AdminViewModel(IModuleRepository& repo, QObject* parent)
    : QObject(parent), repo_(repo) {}

// ── Private ──────────────────────────────────────────────────────────────────
void AdminViewModel::reload() {
    items_.clear();
    for (const auto& m : repo_.getAll())
        items_.push_back(toItem(m));
    emit modulesReset();
}

int AdminViewModel::findIndexById(int id) const {
    for (int i = 0; i < items_.size(); ++i)
        if (items_[i].id == id) return i;
    return -1;
}

// ── Public slots ─────────────────────────────────────────────────────────────
void AdminViewModel::load() {
    reload();
}

void AdminViewModel::addModule(const QString& serial, int slotCount,
                                const QString& firmware, const QString& status) {
    if (serial.trimmed().isEmpty()) {
        emit errorOccurred("Serial не может быть пустым");
        return;
    }
    if (repo_.existsBySerial(serial.trimmed().toStdString())) {
        emit errorOccurred("Serial уже существует");
        return;
    }
    repo_.add(toDomain(0, serial, slotCount, firmware, status));
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
    if (repo_.existsBySerial(serial.trimmed().toStdString(), id)) {
        emit errorOccurred("Serial уже существует");
        return;
    }
    repo_.update(toDomain(id, serial, slotCount, firmware, status));
    reload();
    emit infoOccurred("Модуль обновлён");
}

void AdminViewModel::removeModule(int id) {
    if (findIndexById(id) < 0) {
        emit errorOccurred("Модуль не найден");
        return;
    }
    repo_.remove(id);
    reload();
    emit infoOccurred("Модуль удалён");
}
